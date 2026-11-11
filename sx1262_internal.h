/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SX1262_INTERNAL_H
#define SX1262_INTERNAL_H

#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/sx1262.h>
#include "sx1262_logic.h"

#define SX_RX_DEPTH 128
#define SX_TX_DEPTH 32
#define SX_GLOBAL_TX_DEPTH 64
#define SX_POWER_ENTRIES 32

#define SX_IRQ_MASK (SX_IRQ_TX_DONE | SX_IRQ_RX_DONE | SX_IRQ_PREAMBLE | \
	SX_IRQ_HEADER | SX_IRQ_HEADER_ERR | SX_IRQ_CRC_ERR | SX_IRQ_TIMEOUT)

struct sx_power {
	s32 dbm;
	s8 chip_dbm;
	u8 ocp;
};

struct sx1262 {
	struct spi_device *spi;
	struct miscdevice misc;
	struct kref ref;
	/* io serializes all chip access, configuration and lifecycle changes. */
	struct mutex io;
	struct mutex reader;
	struct gpio_desc *reset, *busy, *dio1, *rxen, *txen;
	struct completion tx_done;
	struct delayed_work watchdog;
	wait_queue_head_t read_wait;
	struct sx1262_config config;
	struct sx1262_stats stats;
	struct sx1262_event *rx_ring, *tx_ring;
	unsigned int rx_head, rx_count, tx_head, tx_count, tx_credits;
	unsigned int pending;
	unsigned int generation;
	bool dead, opened, configured, online, rx_enabled, transmitting;
	int tx_result;
	int id;
	char name[24];
	unsigned long last_irq, rx_deadline, retry_at;
	enum sx_rx_stage rx_stage;
	unsigned int strikes, empty_irqs;
	u32 min_hz, max_hz, tcxo_uv, tcxo_us, rf_switch_us;
	u8 pa_duty, hp_max, ramp, regulator_mode;
	bool dio2_rf_switch;
	struct sx_power powers[SX_POWER_ENTRIES];
	unsigned int power_count;
	/* SPI buffers must not live on the stack (DMA-capable controllers). */
	u8 tx_buf[260] __aligned(ARCH_DMA_MINALIGN);
	u8 rx_buf[260] __aligned(ARCH_DMA_MINALIGN);
};

int sx_radio_properties(struct sx1262 *s);
int sx_radio_validate(struct sx1262 *s, const struct sx1262_config *c);
int sx_radio_init(struct sx1262 *s);
int sx_radio_rx(struct sx1262 *s);
int sx_radio_idle(struct sx1262 *s);
int sx_radio_tx(struct sx1262 *s, const struct sx1262_frame *f, u32 timeout_ms);
int sx_radio_irq(struct sx1262 *s, bool from_irq);
int sx_radio_recover(struct sx1262 *s, bool hard);
void sx_radio_quiesce(struct sx1262 *s);
void sx_rx_push(struct sx1262 *s, const struct sx1262_event *event);

#endif
