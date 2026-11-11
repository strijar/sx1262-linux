// SPDX-License-Identifier: GPL-2.0-only
#include <linux/compat.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include "sx1262_internal.h"

/* One ordered queue deliberately prevents simultaneous TX across all radios. */

static struct workqueue_struct *sx_tx_wq;
static DEFINE_IDA(sx_ids);

/* Protects ring indices, completion reservations and global queue capacity. */

static DEFINE_SPINLOCK(sx_queue_lock);
static DECLARE_WAIT_QUEUE_HEAD(sx_tx_space);
static unsigned int sx_jobs;

struct sx_job {
	struct work_struct work;
	struct sx1262 *radio;
	unsigned int generation;
	struct sx1262_tx_batch batch;
};

static void sx_free(struct kref *ref)
{
	struct sx1262 *s = container_of(ref, struct sx1262, ref);

	ida_free(&sx_ids, s->id);
	kvfree(s->rx_ring);
	kvfree(s->tx_ring);
	kfree(s);
}

void sx_rx_push(struct sx1262 *s, const struct sx1262_event *event)
{
	unsigned long flags;

	lockdep_assert_held(&s->io);
	spin_lock_irqsave(&sx_queue_lock, flags);

	if (s->opened && s->rx_count < SX_RX_DEPTH) {
		s->rx_ring[(s->rx_head + s->rx_count) % SX_RX_DEPTH] = *event;
		s->rx_count++;
	} else {
		s->stats.rx_dropped++;
	}

	spin_unlock_irqrestore(&sx_queue_lock, flags);
	wake_up_interruptible(&s->read_wait);
}

static void sx_tx_worker(struct work_struct *work)
{
	struct sx_job *job = container_of(work, struct sx_job, work);
	struct sx1262 *s = job->radio;
	struct sx1262_event event = {
		.version = SX1262_ABI_VERSION, .type = SX1262_EVENT_TX,
		.cookie = job->batch.cookie,
	};
	unsigned int i;
	unsigned long flags;
	int ret = 0, rx_ret;

	mutex_lock(&s->io);

	if (s->dead || s->generation != job->generation) {
		ret = s->dead ? -ENODEV : -ECANCELED;
		goto finish;
	}

	if (!s->online) {
		ret = -EIO;
		goto finish;
	}

	s->transmitting = true;

	for (i = 0; i < job->batch.count; i++) {
		u32 timeout = sx1262_airtime_ms(&s->config, job->batch.frames[i].length) + 1000;
		unsigned long deadline = jiffies + msecs_to_jiffies(timeout + 200);

		ret = sx_radio_tx(s, &job->batch.frames[i], timeout);

		if (ret)
			break;

		/* Do not hold io or the SPI bus while waiting for airtime. */

		for (;;) {
			mutex_unlock(&s->io);
			wait_for_completion_timeout(&s->tx_done, msecs_to_jiffies(20));
			mutex_lock(&s->io);

			if (s->dead || s->generation != job->generation) {
				ret = s->dead ? -ENODEV : -ECANCELED;
				break;
			}

			if (s->tx_result != -EINPROGRESS) {
				ret = s->tx_result;
				break;
			}

			/* A lost shared IRQ must not stall the global TX queue. */

			ret = sx_radio_irq(s, false);

			if (ret < 0)
				break;

			if (s->tx_result != -EINPROGRESS) {
				ret = s->tx_result;
				break;
			}

			if (time_after_eq(jiffies, deadline)) {
				ret = -ETIMEDOUT;
				break;
			}
		}

		if (ret)
			break;

		event.frames_sent++;
		s->stats.tx_packets++;
	}

	s->transmitting = false;

	if (!s->dead && s->generation == job->generation) {
		if (ret) {
			sx_radio_recover(s, true);
		} else {
			rx_ret = sx_radio_rx(s);

			if (rx_ret) {
				ret = rx_ret;
				sx_radio_recover(s, true);
			}
		}
	}

finish:
	if (ret) {
		s->stats.tx_errors++;
		s->stats.last_error = ret;
	}

	event.status = ret;
	event.timestamp_ns = ktime_get_boottime_ns();
	s->pending--;
	spin_lock_irqsave(&sx_queue_lock, flags);

	/* A credit was reserved on submission, so a TX result cannot overflow. */

	if (!s->dead && s->opened && s->generation == job->generation) {
		s->tx_ring[(s->tx_head + s->tx_count) % SX_TX_DEPTH] = event;
		s->tx_count++;
	} else {
		s->tx_credits--;
	}

	sx_jobs--;
	spin_unlock_irqrestore(&sx_queue_lock, flags);
	mutex_unlock(&s->io);
	wake_up_interruptible(&s->read_wait);
	wake_up_interruptible(&sx_tx_space);
	kref_put(&s->ref, sx_free);
	kfree(job);
}

static bool sx_space(struct sx1262 *s)
{
	return READ_ONCE(sx_jobs) < SX_GLOBAL_TX_DEPTH &&
		READ_ONCE(s->tx_credits) < SX_TX_DEPTH;
}

static int sx_submit(struct file *file, struct sx_job *job)
{
	struct sx1262 *s = file->private_data;
	unsigned long flags;
	unsigned int i;
	int ret;

	if (job->batch.version != SX1262_ABI_VERSION || !job->batch.count ||
	    job->batch.count > SX1262_MAX_BATCH)
		return -EINVAL;

	for (i = 0; i < job->batch.count; i++)
		if (!job->batch.frames[i].length || job->batch.frames[i].length > 255 ||
		    job->batch.frames[i].reserved)
			return -EINVAL;

	for (;;) {
		ret = mutex_lock_interruptible(&s->io);

		if (ret)
			return ret;

		if (s->dead || !s->configured || !s->online) {
			ret = s->dead ? -ENODEV : -EIO;
			goto unlock;
		}

		for (i = 0; i < job->batch.count; i++) {
			if (s->config.implicit_header &&
			    job->batch.frames[i].length != s->config.implicit_length) {
				ret = -EMSGSIZE;
				goto unlock;
			}
		}

		spin_lock_irqsave(&sx_queue_lock, flags);

		if (sx_space(s)) {
			sx_jobs++;
			s->tx_credits++;
			spin_unlock_irqrestore(&sx_queue_lock, flags);
			break;
		}

		spin_unlock_irqrestore(&sx_queue_lock, flags);
		mutex_unlock(&s->io);

		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(sx_tx_space, sx_space(s) || READ_ONCE(s->dead));

		if (ret)
			return ret;
	}

	s->pending++;
	job->radio = s;
	job->generation = s->generation;
	kref_get(&s->ref);
	INIT_WORK(&job->work, sx_tx_worker);
	queue_work(sx_tx_wq, &job->work);
	ret = 0;

unlock:
	mutex_unlock(&s->io);
	return ret;
}

static ssize_t sx_write(struct file *file, const char __user *data, size_t len, loff_t *pos)
{
	struct sx_job *job;
	int ret;

	if (!len || len > SX1262_MAX_PAYLOAD)
		return -EMSGSIZE;

	job = kzalloc(sizeof(*job), GFP_KERNEL);

	if (!job)
		return -ENOMEM;

	job->batch.version = SX1262_ABI_VERSION;
	job->batch.count = 1;
	job->batch.frames[0].length = len;

	if (copy_from_user(job->batch.frames[0].data, data, len)) {
		kfree(job);
		return -EFAULT;
	}

	ret = sx_submit(file, job);

	if (ret)
		kfree(job);

	return ret ?: len;
}

static bool sx_readable(struct sx1262 *s)
{
	return READ_ONCE(s->rx_count) || READ_ONCE(s->tx_count);
}

static ssize_t sx_read_event(struct file *file, char __user *data, size_t len, loff_t *pos)
{
	struct sx1262 *s = file->private_data;
	struct sx1262_event event;
	unsigned long flags;
	int ret;
	bool tx;

	if (len < sizeof(event))
		return -EMSGSIZE;

	ret = mutex_lock_interruptible(&s->reader);

	if (ret)
		return ret;

	for (;;) {
		spin_lock_irqsave(&sx_queue_lock, flags);
		tx = s->tx_count != 0;

		if (tx || s->rx_count) {
			event = tx ? s->tx_ring[s->tx_head] : s->rx_ring[s->rx_head];
			spin_unlock_irqrestore(&sx_queue_lock, flags);
			break;
		}

		spin_unlock_irqrestore(&sx_queue_lock, flags);

		if (READ_ONCE(s->dead)) {
			ret = -ENODEV;
			goto out;
		}

		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto out;
		}

		ret = wait_event_interruptible(s->read_wait, sx_readable(s) || READ_ONCE(s->dead));

		if (ret)
			goto out;
	}

	/* Keep the record queued on EFAULT. Only this reader removes records. */

	if (copy_to_user(data, &event, sizeof(event))) {
		ret = -EFAULT;
		goto out;
	}

	spin_lock_irqsave(&sx_queue_lock, flags);

	if (tx) {
		s->tx_head = (s->tx_head + 1) % SX_TX_DEPTH;
		s->tx_count--;
		s->tx_credits--;
	} else {
		s->rx_head = (s->rx_head + 1) % SX_RX_DEPTH;
		s->rx_count--;
	}

	spin_unlock_irqrestore(&sx_queue_lock, flags);

	if (tx)
		wake_up_interruptible(&sx_tx_space);

	ret = sizeof(event);

out:
	mutex_unlock(&s->reader);
	return ret;
}

static __poll_t sx_poll(struct file *file, poll_table *wait)
{
	struct sx1262 *s = file->private_data;
	__poll_t mask = 0;

	poll_wait(file, &s->read_wait, wait);
	poll_wait(file, &sx_tx_space, wait);

	if (sx_readable(s))
		mask |= EPOLLIN | EPOLLRDNORM;

	if (READ_ONCE(s->dead))
		return mask | EPOLLHUP | EPOLLERR;

	if (READ_ONCE(s->configured) && READ_ONCE(s->online) && sx_space(s))
		mask |= EPOLLOUT | EPOLLWRNORM;

	if (READ_ONCE(s->configured) && !READ_ONCE(s->online))
		mask |= EPOLLERR;

	return mask;
}

static long sx_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct sx1262 *s = file->private_data;
	void __user *ptr = (void __user *)arg;
	struct sx1262_config config;
	struct sx1262_stats stats;
	struct sx_job *job;
	u32 rx;
	int ret;

	if ((cmd == SX1262_IOC_TX_BATCH || cmd == SX1262_IOC_SET_CONFIG ||
	     cmd == SX1262_IOC_SET_RX || cmd == SX1262_IOC_RESET) &&
	    !(file->f_mode & FMODE_WRITE))
		return -EBADF;

	if (cmd == SX1262_IOC_TX_BATCH) {
		job = kzalloc(sizeof(*job), GFP_KERNEL);

		if (!job)
			return -ENOMEM;

		if (copy_from_user(&job->batch, ptr, sizeof(job->batch))) {
			kfree(job);
			return -EFAULT;
		}

		ret = sx_submit(file, job);

		if (ret)
			kfree(job);

		return ret;
	}

	if (cmd == SX1262_IOC_SET_CONFIG && copy_from_user(&config, ptr, sizeof(config)))
		return -EFAULT;

	if (cmd == SX1262_IOC_SET_RX && copy_from_user(&rx, ptr, sizeof(rx)))
		return -EFAULT;

	ret = mutex_lock_interruptible(&s->io);

	if (ret)
		return ret;

	if (s->dead) {
		ret = -ENODEV;
		goto out;
	}

	switch (cmd) {
	case SX1262_IOC_GET_CONFIG:
		config = s->config;
		break;
	case SX1262_IOC_GET_STATS:
		stats = s->stats;
		stats.version = SX1262_ABI_VERSION;
		stats.online = s->online;
		stats.tx_pending = s->pending;
		stats.rx_enabled = s->rx_enabled;
		break;
	case SX1262_IOC_SET_CONFIG:
		ret = sx_radio_validate(s, &config);

		if (ret)
			break;

		if (s->pending) {
			ret = -EBUSY;
			break;
		}

		/* A failed apply leaves the requested config offline for retry. */

		s->config = config;
		s->configured = true;
		s->strikes = 0;
		ret = sx_radio_init(s);
		break;

	case SX1262_IOC_SET_RX:
		if (rx > 1) {
			ret = -EINVAL;
			break;
		}

		if (s->pending) {
			ret = -EBUSY;
			break;
		}

		if (!s->configured || !s->online) {
			ret = -EIO;
			break;
		}

		s->rx_enabled = rx;
		ret = sx_radio_idle(s);
		ret = ret ?: sx_radio_rx(s);

		if (ret)
			sx_radio_quiesce(s);

		s->last_irq = jiffies;
		break;

	case SX1262_IOC_RESET:
		if (s->pending)
			ret = -EBUSY;
		else if (!s->configured)
			ret = -EIO;
		else
			ret = sx_radio_recover(s, true);
		break;

	default:
		ret = -ENOTTY;
	}

out:
	mutex_unlock(&s->io);
	wake_up_interruptible(&s->read_wait);
	wake_up_interruptible(&sx_tx_space);

	if (!ret && cmd == SX1262_IOC_GET_CONFIG)
		ret = copy_to_user(ptr, &config, sizeof(config)) ? -EFAULT : 0;

	if (!ret && cmd == SX1262_IOC_GET_STATS)
		ret = copy_to_user(ptr, &stats, sizeof(stats)) ? -EFAULT : 0;

	return ret;
}

static int sx_open(struct inode *inode, struct file *file)
{
	struct sx1262 *s = container_of(file->private_data, struct sx1262, misc);
	int ret = 0;

	mutex_lock(&s->io);

	if (s->dead)
		ret = -ENODEV;
	else if (s->opened)
		ret = -EBUSY;
	else {
		kref_get(&s->ref);
		s->opened = true;
		s->generation++;
		s->configured = false;
		s->rx_enabled = true;
		file->private_data = s;
	}

	mutex_unlock(&s->io);

	return ret ?: nonseekable_open(inode, file);
}

static int sx_release(struct inode *inode, struct file *file)
{
	struct sx1262 *s = file->private_data;
	unsigned long flags;

	mutex_lock(&s->io);
	s->opened = false;
	s->configured = false;
	s->generation++;
	s->rx_enabled = false;

	if (!s->dead)
		sx_radio_quiesce(s);

	s->tx_result = -ECANCELED;
	complete(&s->tx_done);
	spin_lock_irqsave(&sx_queue_lock, flags);
	s->rx_head = 0;
	s->rx_count = 0;
	s->tx_credits -= s->tx_count;
	s->tx_count = 0;
	s->tx_head = 0;
	spin_unlock_irqrestore(&sx_queue_lock, flags);
	mutex_unlock(&s->io);
	wake_up_interruptible(&sx_tx_space);
	kref_put(&s->ref, sx_free);

	return 0;
}

static const struct file_operations sx_fops = {
	.owner = THIS_MODULE,
	.open = sx_open,
	.release = sx_release,
	.read = sx_read_event,
	.write = sx_write,
	.poll = sx_poll,
	.unlocked_ioctl = sx_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = compat_ptr_ioctl,
#endif
};

static irqreturn_t sx_irq_thread(int irq, void *data)
{
	struct sx1262 *s = data;
	int ret = 0, dio = -1;

	mutex_lock(&s->io);

	if (s->dead || !s->online)
		goto out;

	if (s->dio1) {
		dio = gpiod_get_value_cansleep(s->dio1);

		if (!dio)
			goto out;

		if (dio < 0) {
			ret = dio;
			goto fault;
		}
	}

	ret = sx_radio_irq(s, true);

	/* Persistent high DIO1 without flags is not a healthy shared IRQ source. */

	if (!ret && dio == 1) {
		if (++s->empty_irqs >= 3)
			ret = -EIO;
	} else {
		s->empty_irqs = 0;
	}

fault:
	if (ret < 0) {
		s->stats.last_error = ret;
		s->tx_result = ret;
		complete(&s->tx_done);
		/* Deassert the offender; recovery runs outside the shared IRQ thread. */
		sx_radio_quiesce(s);
	}

out:
	mutex_unlock(&s->io);
	return ret ? IRQ_HANDLED : IRQ_NONE;
}

static void sx_watchdog(struct work_struct *work)
{
	struct sx1262 *s = container_of(to_delayed_work(work), struct sx1262, watchdog);
	bool expired, recovering = false;
	unsigned long delay = msecs_to_jiffies(100);
	int ret;

	mutex_lock(&s->io);

	if (s->dead)
		goto out;

	if (!s->configured || s->transmitting)
		goto again;

	if (!s->online) {
		if (time_after_eq(jiffies, s->retry_at)) {
			sx_radio_recover(s, true);
			recovering = true;
		}
		goto again;
	}

	if (!s->rx_enabled)
		goto again;

	ret = sx_radio_irq(s, false);

	if (ret < 0) {
		s->stats.last_error = ret;
		sx_radio_recover(s, true);
		recovering = true;
		goto again;
	}

	expired = s->rx_deadline && time_after_eq(jiffies, s->rx_deadline);

	/* Do not reset during a legitimately long frame/preamble. */

	if (expired || (!s->rx_deadline && s->config.silence_timeout_ms &&
	    time_after_eq(jiffies, s->last_irq + msecs_to_jiffies(s->config.silence_timeout_ms)))) {
		sx_radio_recover(s, false);
		recovering = true;
	}

again:
	/* Keep background polling at 100 ms, but wake at an earlier RX deadline. */
	if (s->online && !s->transmitting && s->rx_deadline) {
		unsigned long now = jiffies;

		delay = min(delay, time_after(s->rx_deadline, now) ? s->rx_deadline - now : 1UL);
	}

	schedule_delayed_work(&s->watchdog, delay);

out:
	mutex_unlock(&s->io);

	if (recovering) {
		wake_up_interruptible(&s->read_wait);
		wake_up_interruptible(&sx_tx_space);
	}
}

static int sx_probe(struct spi_device *spi)
{
	struct sx1262 *s;
	unsigned int trigger;
	int ret;

	if (spi->irq <= 0)
		return dev_err_probe(&spi->dev, -EINVAL, "DIO1 interrupt required\n");

	trigger = irq_get_trigger_type(spi->irq);

	if (trigger != IRQ_TYPE_LEVEL_HIGH && trigger != IRQ_TYPE_EDGE_RISING)
		return dev_err_probe(&spi->dev, -EINVAL, "use level-high or rising-edge DIO1 IRQ\n");

	s = kzalloc(sizeof(*s), GFP_KERNEL);

	if (!s)
		return -ENOMEM;

	s->id = ida_alloc_max(&sx_ids, SX1262_MAX_RADIOS - 1, GFP_KERNEL);

	if (s->id < 0) {
		ret = s->id;
		kfree(s);
		return ret;
	}

	kref_init(&s->ref);
	mutex_init(&s->io);
	mutex_init(&s->reader);
	init_completion(&s->tx_done);
	init_waitqueue_head(&s->read_wait);
	INIT_DELAYED_WORK(&s->watchdog, sx_watchdog);
	s->spi = spi;
	s->rx_ring = kvcalloc(SX_RX_DEPTH, sizeof(*s->rx_ring), GFP_KERNEL);
	s->tx_ring = kvcalloc(SX_TX_DEPTH, sizeof(*s->tx_ring), GFP_KERNEL);

	if (!s->rx_ring || !s->tx_ring) {
		ret = -ENOMEM;
		goto fail;
	}

	s->reset = devm_gpiod_get(&spi->dev, "reset", GPIOD_OUT_HIGH);

	if (IS_ERR(s->reset)) {
		ret = PTR_ERR(s->reset);
		goto fail;
	}

	s->busy = devm_gpiod_get(&spi->dev, "busy", GPIOD_IN);

	if (IS_ERR(s->busy)) {
		ret = PTR_ERR(s->busy);
		goto fail;
	}

	s->dio1 = devm_gpiod_get_optional(&spi->dev, "dio1", GPIOD_IN);

	if (IS_ERR(s->dio1)) {
		ret = PTR_ERR(s->dio1);
		goto fail;
	}

	s->rxen = devm_gpiod_get_optional(&spi->dev, "rxen", GPIOD_OUT_LOW);

	if (IS_ERR(s->rxen)) {
		ret = PTR_ERR(s->rxen);
		goto fail;
	}

	s->txen = devm_gpiod_get_optional(&spi->dev, "txen", GPIOD_OUT_LOW);

	if (IS_ERR(s->txen)) {
		ret = PTR_ERR(s->txen);
		goto fail;
	}

	ret = sx_radio_properties(s);

	if (ret)
		goto fail;

	if (spi->mode & (SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST)) {
		ret = -EINVAL;
		goto fail;
	}

	spi->bits_per_word = 8;
	spi->max_speed_hz = min(spi->max_speed_hz, 16000000U);
	ret = spi_setup(spi);

	if (ret)
		goto fail;

	s->config = (struct sx1262_config) {
		.version = SX1262_ABI_VERSION,
		.frequency_hz = clamp(868000000U, s->min_hz, s->max_hz),
		.bandwidth_hz = 125000, .tx_power_dbm = s->powers[0].dbm,
		.preamble_symbols = 25, .sync_word = 0x1424,
		.spreading_factor = 7, .coding_rate = 5, .crc = 1,
		.silence_timeout_ms = 60000, .hard_reset_after = 2,
	};

	snprintf(s->name, sizeof(s->name), "sx1262%d", s->id);

	s->misc = (struct miscdevice) {
		.minor = MISC_DYNAMIC_MINOR, .name = s->name,
		.fops = &sx_fops, .parent = &spi->dev, .mode = 0600,
	};

	spi_set_drvdata(spi, s);
	ret = request_threaded_irq(spi->irq, NULL, sx_irq_thread,
				   IRQF_ONESHOT | IRQF_SHARED, s->name, s);

	if (ret)
		goto fail;

	ret = misc_register(&s->misc);

	if (ret) {
		free_irq(spi->irq, s);
		goto fail;
	}

	schedule_delayed_work(&s->watchdog, msecs_to_jiffies(100));
	dev_info(&spi->dev, "/dev/%s, IRQ %d; configure via ioctl to start radio\n", s->name, spi->irq);
	return 0;

fail:
	kref_put(&s->ref, sx_free);
	return dev_err_probe(&spi->dev, ret, "SX1262 probe failed\n");
}

static void sx_shutdown(struct spi_device *spi)
{
	struct sx1262 *s = spi_get_drvdata(spi);

	mutex_lock(&s->io);
	s->dead = true;
	s->generation++;
	sx_radio_quiesce(s);
	s->tx_result = -ENODEV;
	complete(&s->tx_done);
	mutex_unlock(&s->io);
	cancel_delayed_work_sync(&s->watchdog);
	wake_up_interruptible(&s->read_wait);
	wake_up_interruptible(&sx_tx_space);
}

static void sx_remove(struct spi_device *spi)
{
	struct sx1262 *s = spi_get_drvdata(spi);

	sx_shutdown(spi);
	misc_deregister(&s->misc);
	free_irq(spi->irq, s);

	/* Open fds and queued jobs keep only the software object alive. */

	kref_put(&s->ref, sx_free);
}

static const struct of_device_id sx_of_match[] = {
	{ .compatible = "semtech,sx1262" },
	{ .compatible = "ebyte,e22-x00m30s" },
	{ }
};
MODULE_DEVICE_TABLE(of, sx_of_match);

static const struct spi_device_id sx_spi_ids[] = {
	{ "sx1262", 0 },
	{ "e22-x00m30s", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, sx_spi_ids);

static struct spi_driver sx_driver = {
	.driver = { .name = "sx1262", .of_match_table = sx_of_match },
	.probe = sx_probe,
	.remove = sx_remove,
	.shutdown = sx_shutdown,
	.id_table = sx_spi_ids,
};

static int __init sx_init(void)
{
	int ret;

	sx_tx_wq = alloc_ordered_workqueue("sx1262-tx", WQ_MEM_RECLAIM);

	if (!sx_tx_wq)
		return -ENOMEM;

	ret = spi_register_driver(&sx_driver);

	if (ret)
		destroy_workqueue(sx_tx_wq);

	return ret;
}

static void __exit sx_exit(void)
{
	spi_unregister_driver(&sx_driver);
	destroy_workqueue(sx_tx_wq);
	ida_destroy(&sx_ids);
}

module_init(sx_init);
module_exit(sx_exit);
MODULE_DESCRIPTION("SX1262 raw LoRa packet driver with shared IRQ and queued TX");
MODULE_AUTHOR("Belousov Oleg <belousov.oleg@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1.0");
