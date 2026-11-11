// SPDX-License-Identifier: GPL-2.0-only
/* SX1262 LoRa PHY. No RNode, Reticulum or LoRaWAN framing. */
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/property.h>
#include "sx1262_internal.h"

static int sx_busy(struct sx1262 *s)
{
	unsigned long end = jiffies + msecs_to_jiffies(200);
	int value;

	do {
		value = gpiod_get_value_cansleep(s->busy);
		if (value <= 0)
			return value;
		usleep_range(100, 250);
	} while (time_before(jiffies, end));

	return -ETIMEDOUT;
}

static int sx_command(struct sx1262 *s, const u8 *cmd, size_t len)
{
	struct spi_transfer xfer = { .tx_buf = s->tx_buf, .len = len };
	int ret;

	lockdep_assert_held(&s->io);

	if (len > sizeof(s->tx_buf))
		return -EINVAL;

	ret = sx_busy(s);

	if (ret)
		return ret;

	memcpy(s->tx_buf, cmd, len);
	ret = spi_sync_transfer(s->spi, &xfer, 1);

	if (ret)
		return ret;

	/* BUSY assertion can lag NSS rising by up to 600 ns. */

	udelay(1);
	return sx_busy(s);
}

#define SX_CMD(s, ...) sx_command(s, (const u8[]){ __VA_ARGS__ }, \
	sizeof((const u8[]){ __VA_ARGS__ }))

static int sx_read(struct sx1262 *s, const u8 *cmd, size_t hdr,
		   u8 *data, size_t len)
{
	struct spi_transfer xfer = {
		.tx_buf = s->tx_buf, .rx_buf = s->rx_buf, .len = hdr + len,
	};
	int ret;

	lockdep_assert_held(&s->io);

	if (hdr + len > sizeof(s->tx_buf))
		return -EINVAL;

	ret = sx_busy(s);

	if (ret)
		return ret;

	memset(s->tx_buf, 0, hdr + len);
	memcpy(s->tx_buf, cmd, hdr);
	ret = spi_sync_transfer(s->spi, &xfer, 1);

	if (!ret)
		memcpy(data, s->rx_buf + hdr, len);
	udelay(1);

	return ret;
}

static int sx_reg_write(struct sx1262 *s, u16 reg, u8 value)
{
	return SX_CMD(s, 0x0d, reg >> 8, reg, value);
}

static int sx_reg_update(struct sx1262 *s, u16 reg, u8 mask, u8 value)
{
	u8 cmd[] = { 0x1d, reg >> 8, reg, 0 }, old;
	int ret = sx_read(s, cmd, sizeof(cmd), &old, 1);

	return ret ? ret : sx_reg_write(s, reg, (old & ~mask) | (value & mask));
}

static void sx_antenna(struct sx1262 *s, bool rx, bool tx)
{
	gpiod_set_value_cansleep(s->rxen, 0);
	gpiod_set_value_cansleep(s->txen, 0);

	if (rx || tx) {
		if (s->rf_switch_us)
			usleep_range(s->rf_switch_us, s->rf_switch_us + 50);
		gpiod_set_value_cansleep(rx ? s->rxen : s->txen, 1);
	}
}

static int sx_packet_params(struct sx1262 *s, u8 len)
{
	const struct sx1262_config *c = &s->config;

	return SX_CMD(s, 0x8c, c->preamble_symbols >> 8, c->preamble_symbols,
		      c->implicit_header, len, c->crc, c->invert_iq);
}

int sx_radio_idle(struct sx1262 *s)
{
	int ret = SX_CMD(s, 0x80, 0);

	sx_antenna(s, false, false);
	s->rx_deadline = 0;
	s->rx_stage = SX_RX_WAITING;
	return ret;
}

int sx_radio_rx(struct sx1262 *s)
{
	int ret;

	if (!s->rx_enabled)
		return sx_radio_idle(s);

	ret = sx_packet_params(s, s->config.implicit_header ?
			       s->config.implicit_length : 255);

	if (ret)
		return ret;

	sx_antenna(s, true, false);
	s->rx_deadline = 0;
	s->rx_stage = SX_RX_WAITING;

	/* Continuous RX: do not stop/restart after every received frame. */

	return SX_CMD(s, 0x82, 0xff, 0xff, 0xff);
}

static const struct sx_power *sx_power_find(struct sx1262 *s, int dbm)
{
	unsigned int i;

	for (i = 0; i < s->power_count; i++)
		if (s->powers[i].dbm == dbm)
			return &s->powers[i];

	return NULL;
}

int sx_radio_validate(struct sx1262 *s, const struct sx1262_config *c)
{
	if (!sx1262_config_valid(c) || c->frequency_hz < s->min_hz ||
	    c->frequency_hz > s->max_hz || !sx_power_find(s, c->tx_power_dbm))
		return -EINVAL;

	return 0;
}

static int sx_tcxo_code(u32 uv)
{
	static const u32 values[] = {
		1600000, 1700000, 1800000, 2200000,
		2400000, 2700000, 3000000, 3300000,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(values); i++)
		if (uv == values[i])
			return i;

	return -EINVAL;
}

/* All optional integers are validated, including malformed properties. */
static int sx_property(struct device *dev, const char *name, u32 *out)
{
	if (!device_property_present(dev, name))
		return 0;

	return device_property_read_u32(dev, name, out);
}

int sx_radio_properties(struct sx1262 *s)
{
	struct device *dev = &s->spi->dev;
	bool e22 = device_is_compatible(dev, "ebyte,e22-x00m30s");
	u32 duty = 4, hp = 7, ramp = 4, mode = e22 ? 1 : 0;
	u32 table[SX_POWER_ENTRIES * 3];
	int ret, count, i, j;

	s->min_hz = e22 ? 410000000 : 150000000;
	s->max_hz = e22 ? 930000000 : 960000000;
	s->tcxo_uv = e22 ? 1800000 : 0;
	s->tcxo_us = 5000;
	s->rf_switch_us = 100;
	ret = sx_property(dev, "semtech,frequency-min-hz", &s->min_hz);
	ret = ret ?: sx_property(dev, "semtech,frequency-max-hz", &s->max_hz);
	ret = ret ?: sx_property(dev, "semtech,tcxo-microvolt", &s->tcxo_uv);
	ret = ret ?: sx_property(dev, "semtech,tcxo-startup-us", &s->tcxo_us);
	ret = ret ?: sx_property(dev, "semtech,rf-switch-delay-us", &s->rf_switch_us);
	ret = ret ?: sx_property(dev, "semtech,pa-duty-cycle", &duty);
	ret = ret ?: sx_property(dev, "semtech,pa-hp-max", &hp);
	ret = ret ?: sx_property(dev, "semtech,tx-ramp", &ramp);
	ret = ret ?: sx_property(dev, "semtech,regulator-mode", &mode);

	if (ret)
		return ret;

	if (s->min_hz < 150000000 || s->max_hz > 960000000 ||
	    s->min_hz > s->max_hz || duty > 4 || hp > 7 || ramp > 7 || mode > 1 ||
	    s->tcxo_us > 100000 || s->rf_switch_us > 10000 ||
	    (s->tcxo_uv && sx_tcxo_code(s->tcxo_uv) < 0))
		return -EINVAL;

	s->pa_duty = duty;
	s->hp_max = hp;
	s->ramp = ramp;
	s->regulator_mode = mode;
	s->dio2_rf_switch = device_property_read_bool(dev, "semtech,dio2-rf-switch");

	if (s->dio2_rf_switch && (s->rxen || s->txen))
		return -EINVAL;

	if (device_property_present(dev, "semtech,tx-power-table")) {
		count = device_property_count_u32(dev, "semtech,tx-power-table");

		if (count <= 0 || count > ARRAY_SIZE(table) || count % 3)
			return -EINVAL;

		ret = device_property_read_u32_array(dev, "semtech,tx-power-table", table, count);

		if (ret)
			return ret;

		s->power_count = count / 3;

		for (i = 0; i < s->power_count; i++) {
			s32 dbm = (s32)table[3 * i];
			s32 chip = (s32)table[3 * i + 1];

			if (dbm < -40 || dbm > 40 || chip < -9 || chip > 22 ||
			    !table[3 * i + 2] || table[3 * i + 2] > 0x3f)
				return -EINVAL;

			for (j = 0; j < i; j++)
				if (s->powers[j].dbm == dbm)
					return -EINVAL;

			s->powers[i] = (struct sx_power){dbm, chip, table[3 * i + 2]};
		}
	} else {
		s->power_count = e22 ? 23 : 32;

		for (i = 0; i < s->power_count; i++) {
			int dbm = e22 ? 8 + i : -9 + i;

			s->powers[i] = (struct sx_power){
				dbm, e22 ? dbm - 17 : dbm,
				e22 && dbm <= 22 ? 0x18 : 0x38,
			};
		}
	}
	return 0;
}

int sx_radio_init(struct sx1262 *s)
{
	const struct sx1262_config *c = &s->config;
	const struct sx_power *power = sx_power_find(s, c->tx_power_dbm);
	u32 frf = div_u64((u64)c->frequency_hz << 25, 32000000);
	u32 ticks = DIV_ROUND_UP(s->tcxo_us * 64, 1000);
	u32 mhz = c->frequency_hz / 1000000;
	u8 read_sync[] = { 0x1d, 0x07, 0x40, 0 }, sync[2];
	u8 errors_cmd[] = { 0x17, 0 }, errors[2];
	int ret;

	s->online = false;
	sx_antenna(s, false, false);
	gpiod_set_value_cansleep(s->reset, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(s->reset, 0);
	usleep_range(10000, 11000);
	ret = SX_CMD(s, 0x80, 0);               /* standby RC */
	ret = ret ?: SX_CMD(s, 0x07, 0, 0);     /* clear startup errors */
	ret = ret ?: SX_CMD(s, 0x96, s->regulator_mode);

	if (!ret && s->tcxo_uv)
		ret = SX_CMD(s, 0x97, sx_tcxo_code(s->tcxo_uv), ticks >> 16, ticks >> 8, ticks);

	ret = ret ?: SX_CMD(s, 0x89, 0x7f);     /* calibrate after oscillator setup */
	ret = ret ?: SX_CMD(s, 0x8a, 1);        /* LoRa only */
	ret = ret ?: SX_CMD(s, 0x9d, s->dio2_rf_switch);
	ret = ret ?: SX_CMD(s, 0x8f, 0, 0);     /* separate modes share the 256-byte buffer */

	/* Calibrate an interval containing the actual carrier, 4 MHz steps. */

	ret = ret ?: SX_CMD(s, 0x98, (mhz - 4) / 4, DIV_ROUND_UP(mhz + 5, 4));
	ret = ret ?: SX_CMD(s, 0x86, frf >> 24, frf >> 16, frf >> 8, frf);
	ret = ret ?: SX_CMD(s, 0x8b, c->spreading_factor, sx1262_bw_code(c->bandwidth_hz),
			   c->coding_rate - 4, sx1262_ldro(c));
	ret = ret ?: SX_CMD(s, 0x0d, 0x07, 0x40, c->sync_word >> 8, c->sync_word);
	ret = ret ?: SX_CMD(s, 0x95, s->pa_duty, s->hp_max, 0, 1);
	ret = ret ?: sx_reg_write(s, 0x08e7, power->ocp);
	ret = ret ?: sx_reg_update(s, 0x08d8, 0x1e, 0x1e); /* TX clamp */
	ret = ret ?: SX_CMD(s, 0x8e, (u8)power->chip_dbm, s->ramp);
	ret = ret ?: sx_reg_update(s, 0x0889, 4, c->bandwidth_hz == 500000 ? 0 : 4);
	ret = ret ?: sx_reg_update(s, 0x0736, 4, c->invert_iq ? 0 : 4);
	ret = ret ?: SX_CMD(s, 0x93, 0x20); /* TX fallback: standby RC */
	ret = ret ?: SX_CMD(s, 0x02, 0xff, 0xff);
	ret = ret ?: SX_CMD(s, 0x08, SX_IRQ_MASK >> 8, SX_IRQ_MASK & 0xff,
			   SX_IRQ_MASK >> 8, SX_IRQ_MASK & 0xff, 0, 0, 0, 0);
	ret = ret ?: sx_read(s, read_sync, sizeof(read_sync), sync, sizeof(sync));

	if (!ret && (sync[0] != (c->sync_word >> 8) || sync[1] != (c->sync_word & 0xff)))
		ret = -EIO;

	ret = ret ?: sx_read(s, errors_cmd, sizeof(errors_cmd), errors, sizeof(errors));

	if (!ret && (errors[0] || errors[1]))
		ret = -EIO;

	ret = ret ?: sx_radio_rx(s);
	s->last_irq = jiffies;
	s->rx_deadline = 0;
	s->rx_stage = SX_RX_WAITING;
	s->stats.last_error = ret;
	s->online = !ret;

	if (ret)
		sx_radio_quiesce(s);

	return ret;
}

int sx_radio_tx(struct sx1262 *s, const struct sx1262_frame *f, u32 timeout_ms)
{
	u8 buffer[257] = { 0x0e, 0 };
	u32 ticks = min(timeout_ms * 64, 0xfffffeU);
	int ret;

	ret = sx_radio_idle(s);

	/* Drain an RX_DONE latched just before standby before overwriting RAM. */

	ret = ret ?: sx_radio_irq(s, false);

	if (ret < 0)
		return ret;

	memcpy(buffer + 2, f->data, f->length);
	ret = sx_command(s, buffer, f->length + 2);
	ret = ret ?: sx_packet_params(s, f->length);
	ret = ret ?: SX_CMD(s, 0x02, 0xff, 0xff);

	if (ret)
		return ret;

	sx_antenna(s, false, true);
	reinit_completion(&s->tx_done);
	s->tx_result = -EINPROGRESS;

	return SX_CMD(s, 0x83, ticks >> 16, ticks >> 8, ticks);
}

static int sx_receive(struct sx1262 *s)
{
	struct sx1262_event event = {
		.version = SX1262_ABI_VERSION, .type = SX1262_EVENT_RX,
		.timestamp_ns = ktime_get_boottime_ns(),
	};
	u8 buffer_cmd[] = { 0x13, 0 }, buffer[2];
	u8 packet_cmd[] = { 0x14, 0 }, packet[3];
	u8 read_cmd[] = { 0x1e, 0, 0 };
	int ret = sx_read(s, buffer_cmd, sizeof(buffer_cmd), buffer, sizeof(buffer));

	if (ret)
		return ret;

	event.length = s->config.implicit_header ? s->config.implicit_length : buffer[0];
	read_cmd[1] = buffer[1];
	ret = sx_read(s, read_cmd, sizeof(read_cmd), event.data, event.length);
	ret = ret ?: sx_read(s, packet_cmd, sizeof(packet_cmd), packet, sizeof(packet));

	if (ret)
		return ret;

	event.rssi_half_dbm = -(s16)packet[0];
	event.snr_quarter_db = (s8)packet[1];
	s->stats.rx_packets++;
	sx_rx_push(s, &event);

	return 0;
}

/* Returns 1 if any radio IRQ was consumed, 0 if none, negative errno. */

int sx_radio_irq(struct sx1262 *s, bool from_irq)
{
	u8 cmd[] = { 0x12, 0 }, data[2];
	u16 flags;
	int ret, handled = 0, rounds;

	/* Bounded drain: let the shared IRQ service the other modules as well. */

	for (rounds = 0; rounds < 8; rounds++) {
		enum sx_rx_stage next;

		ret = sx_read(s, cmd, sizeof(cmd), data, sizeof(data));

		if (ret)
			return ret;

		flags = (data[0] << 8) | data[1];

		if (!flags)
			break;

		if (flags & ~0x03ff)
			return -EIO;

		handled = 1;

		/* Acknowledge the snapshot first, so a following RX_DONE stays latched. */

		ret = SX_CMD(s, 0x02, flags >> 8, flags);

		if (ret)
			return ret;

		if (from_irq) {
			s->last_irq = jiffies;
			s->strikes = 0;
		} else {
			s->stats.polled_irqs++;
		}

		next = sx1262_rx_next_stage(s->rx_stage, flags);

		if (next == SX_RX_WAITING) {
			s->rx_deadline = 0;
		} else if (next != s->rx_stage) {
			/* Repeated preamble/header IRQs must not extend a stuck reception. */
			s->rx_deadline = jiffies + msecs_to_jiffies(
				sx1262_rx_timeout_ms(&s->config, next));
		}

		s->rx_stage = next;

		if (flags & (SX_IRQ_CRC_ERR | SX_IRQ_HEADER_ERR))
			s->stats.rx_crc_errors++;

		if ((flags & SX_IRQ_RX_DONE) &&
		    !(flags & (SX_IRQ_CRC_ERR | SX_IRQ_HEADER_ERR))) {
			ret = sx_receive(s);
			if (ret)
				return ret;
		}

		if (s->transmitting && (flags & (SX_IRQ_TX_DONE | SX_IRQ_TIMEOUT))) {
			s->tx_result = flags & SX_IRQ_TX_DONE ? 0 : -ETIMEDOUT;
			complete(&s->tx_done);
		}
	}

	return handled;
}

void sx_radio_quiesce(struct sx1262 *s)
{
	/* Reset also deasserts DIO1 when SPI/BUSY is no longer usable. */

	sx_antenna(s, false, false);
	gpiod_set_value_cansleep(s->reset, 1);
	s->online = false;
	s->rx_deadline = 0;
	s->rx_stage = SX_RX_WAITING;
	s->retry_at = jiffies + HZ;
}

int sx_radio_recover(struct sx1262 *s, bool hard)
{
	int ret;

	if (!hard && ++s->strikes < s->config.hard_reset_after) {
		s->stats.soft_recoveries++;
		ret = sx_radio_idle(s);
		ret = ret ?: SX_CMD(s, 0x02, 0xff, 0xff);
		ret = ret ?: sx_radio_rx(s);

		if (!ret) {
			s->last_irq = jiffies;
			return 0;
		}
	}

	s->stats.hard_recoveries++;
	ret = sx_radio_init(s);
	s->strikes = 0;

	if (ret)
		dev_warn_ratelimited(&s->spi->dev, "radio recovery failed: %d\n", ret);

	return ret;
}
