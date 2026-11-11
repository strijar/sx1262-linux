// SPDX-License-Identifier: GPL-2.0-only
#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include "../sx1262_logic.h"

_Static_assert(sizeof(struct sx1262_config) == 40, "config ABI");
_Static_assert(sizeof(struct sx1262_frame) == 258, "frame ABI");
_Static_assert(sizeof(struct sx1262_tx_batch) == 2080, "batch ABI");
_Static_assert(sizeof(struct sx1262_event) == 296, "event ABI");
_Static_assert(sizeof(struct sx1262_stats) == 88, "stats ABI");
_Static_assert(offsetof(struct sx1262_event, cookie) == 16, "cookie alignment");
_Static_assert(offsetof(struct sx1262_event, data) == 36, "payload alignment");

static void rx_timeout_test(void)
{
	struct sx1262_config c = {
		.bandwidth_hz = 125000, .spreading_factor = 7, .coding_rate = 5,
		.preamble_symbols = 25, .crc = 1,
	};
	const unsigned int terminal[] = {
		SX_IRQ_RX_DONE, SX_IRQ_CRC_ERR, SX_IRQ_HEADER_ERR, SX_IRQ_TIMEOUT,
	};
	unsigned int i;
	__u32 header_ms;

	/* Default PHY: previously both stages waited 1836 ms. */
	assert(sx1262_rx_timeout_ms(&c, SX_RX_PREAMBLE) == 100);
	assert(sx1262_rx_timeout_ms(&c, SX_RX_HEADER) == 607);
	assert(!sx1262_rx_timeout_ms(&c, SX_RX_WAITING));
	assert(sx1262_rx_next_stage(SX_RX_WAITING, SX_IRQ_PREAMBLE) == SX_RX_PREAMBLE);
	assert(sx1262_rx_next_stage(SX_RX_PREAMBLE, SX_IRQ_HEADER) == SX_RX_HEADER);
	/* A polled HEADER_VALID can be observed without the initial preamble IRQ. */
	assert(sx1262_rx_next_stage(SX_RX_WAITING, SX_IRQ_HEADER) == SX_RX_HEADER);
	assert(sx1262_rx_next_stage(SX_RX_WAITING, SX_IRQ_PREAMBLE | SX_IRQ_HEADER) == SX_RX_HEADER);
	/* No transition means no deadline renewal; noise cannot prolong RX forever. */
	assert(sx1262_rx_next_stage(SX_RX_PREAMBLE, SX_IRQ_PREAMBLE) == SX_RX_PREAMBLE);
	assert(sx1262_rx_next_stage(SX_RX_HEADER, SX_IRQ_HEADER) == SX_RX_HEADER);
	assert(sx1262_rx_next_stage(SX_RX_HEADER, SX_IRQ_PREAMBLE) == SX_RX_HEADER);
	assert(sx1262_rx_next_stage(SX_RX_HEADER, 0) == SX_RX_HEADER);
	for (i = 0; i < sizeof(terminal) / sizeof(terminal[0]); i++) {
		assert(sx1262_rx_next_stage(SX_RX_PREAMBLE, terminal[i]) == SX_RX_WAITING);
		assert(sx1262_rx_next_stage(SX_RX_HEADER, terminal[i] | SX_IRQ_HEADER) == SX_RX_WAITING);
	}
	/* Header timeout excludes even an unusually long configured preamble. */
	c.preamble_symbols = 1000;
	assert(sx1262_rx_timeout_ms(&c, SX_RX_HEADER) == 607);
	assert(sx1262_rx_timeout_ms(&c, SX_RX_PREAMBLE) > 1000);
	c.preamble_symbols = 25;
	c.spreading_factor = 12;
	header_ms = sx1262_rx_timeout_ms(&c, SX_RX_HEADER);
	assert(sx1262_rx_timeout_ms(&c, SX_RX_PREAMBLE) == 1463);
	assert(header_ms == 12952);
	c.ldro = 1;
	assert(sx1262_rx_timeout_ms(&c, SX_RX_HEADER) < header_ms);
	/* Implicit mode must finish even if the chip never asserts HEADER_VALID. */
	c.ldro = 0;
	c.implicit_header = 1;
	c.implicit_length = 255;
	assert(sx1262_rx_timeout_ms(&c, SX_RX_PREAMBLE) > sx1262_airtime_ms(&c, 255));
	c.implicit_length = 1;
	assert(sx1262_rx_timeout_ms(&c, SX_RX_HEADER) < header_ms);
	/* The partial-RX guard is independent of the silence-watchdog switch. */
	c.silence_timeout_ms = 0;
	assert(sx1262_rx_timeout_ms(&c, SX_RX_PREAMBLE) > 0);
}

int main(void)
{
	struct sx1262_config c = {
		.version = 1, .frequency_hz = 868000000, .bandwidth_hz = 125000,
		.tx_power_dbm = 14, .preamble_symbols = 8, .silence_timeout_ms = 60000,
		.sync_word = 0x1424, .spreading_factor = 7, .coding_rate = 5,
		.crc = 1, .hard_reset_after = 2,
	};
	unsigned int sf, bw, length;
	const unsigned int bands[] = {
		7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000, 500000,
	};

	assert(sx1262_config_valid(&c));
	/* Independently calculated PHY examples, rounded up to milliseconds. */
	assert(sx1262_airtime_ms(&c, 20) == 57);
	c.spreading_factor = 12;
	assert(sx1262_ldro(&c));
	assert(sx1262_airtime_ms(&c, 20) == 1319);
	c.spreading_factor = 5;
	c.preamble_symbols = 12;
	assert(sx1262_airtime_ms(&c, 20) == 19);
	c.spreading_factor = 11;
	c.bandwidth_hz = 125000;
	assert(sx1262_ldro(&c));
	c.bandwidth_hz = 250000;
	assert(!sx1262_ldro(&c));
	c.spreading_factor = 12;
	assert(sx1262_ldro(&c));
	c.ldro = 1;
	assert(!sx1262_ldro(&c));
	c.ldro = 0;
	c.preamble_symbols = 25;
	for (sf = 5; sf <= 12; sf++) {
		c.spreading_factor = sf;
		for (bw = 0; bw < sizeof(bands) / sizeof(bands[0]); bw++) {
			unsigned int previous = 0;

			c.bandwidth_hz = bands[bw];
			assert(sx1262_config_valid(&c));
			for (length = 1; length <= 255; length++) {
				unsigned int ms = sx1262_airtime_ms(&c, length);

				assert(ms && ms >= previous && ms <= 240000);
				previous = ms;
			}
		}
	}
	c.bandwidth_hz = 7800;
	c.preamble_symbols = 65535;
	assert(!sx1262_config_valid(&c)); /* Hardware timer overflow prevented. */
	c.preamble_symbols = 25;
	c.coding_rate = 4;
	assert(!sx1262_config_valid(&c));
	c.coding_rate = 5;
	c.reserved[2] = 1;
	assert(!sx1262_config_valid(&c));
	c.reserved[2] = 0;
	c.implicit_header = 1;
	assert(!sx1262_config_valid(&c));
	c.implicit_length = 255;
	assert(sx1262_config_valid(&c));
	c.silence_timeout_ms = 0;
	assert(sx1262_config_valid(&c));
	c.silence_timeout_ms = 999;
	assert(!sx1262_config_valid(&c));
	assert(sx1262_bw_code(7812) == sx1262_bw_code(7800));
	assert(sx1262_bw_code(12345) < 0);
	rx_timeout_test();
	puts("ABI, airtime, LDRO, RX stage/timeouts and configuration tests passed");
	return 0;
}
