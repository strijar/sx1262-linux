/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SX1262_LOGIC_H
#define SX1262_LOGIC_H

#include <linux/sx1262.h>
#ifdef __KERNEL__
#include <linux/math64.h>
#else
#include <stdint.h>
#define div_u64(n, d) ((n) / (d))
#endif

#define SX_IRQ_TX_DONE (1U << 0)
#define SX_IRQ_RX_DONE (1U << 1)
#define SX_IRQ_PREAMBLE (1U << 2)
#define SX_IRQ_HEADER (1U << 4)
#define SX_IRQ_HEADER_ERR (1U << 5)
#define SX_IRQ_CRC_ERR (1U << 6)
#define SX_IRQ_TIMEOUT (1U << 9)

enum sx_rx_stage {
	SX_RX_WAITING,
	SX_RX_PREAMBLE,
	SX_RX_HEADER,
};

static inline enum sx_rx_stage sx1262_rx_next_stage(enum sx_rx_stage stage, __u16 flags)
{
	if (flags & (SX_IRQ_RX_DONE | SX_IRQ_CRC_ERR | SX_IRQ_HEADER_ERR | SX_IRQ_TIMEOUT))
		return SX_RX_WAITING;

	if (stage == SX_RX_HEADER || (flags & SX_IRQ_HEADER))
		return SX_RX_HEADER;

	if (flags & SX_IRQ_PREAMBLE)
		return SX_RX_PREAMBLE;

	return stage;
}

static inline int sx1262_bw_code(__u32 hz)
{
	switch (hz) {
	case 7800: case 7812: return 0;
	case 10400: case 10417: return 8;
	case 15600: case 15625: return 1;
	case 20800: case 20833: return 9;
	case 31250: return 2;
	case 41700: case 41667: return 10;
	case 62500: return 3;
	case 125000: return 4;
	case 250000: return 5;
	case 500000: return 6;
	default: return -1;
	}
}

/* Exact bandwidth = 500000 / divisor, including fractional narrow bands. */
static inline unsigned int sx1262_bw_div(__u32 hz)
{
	switch (sx1262_bw_code(hz)) {
	case 0: return 64;
	case 8: return 48;
	case 1: return 32;
	case 9: return 24;
	case 2: return 16;
	case 10: return 12;
	case 3: return 8;
	case 4: return 4;
	case 5: return 2;
	default: return 1;
	}
}

static inline int sx1262_ldro(const struct sx1262_config *c)
{
	if (c->ldro)
		return c->ldro == 2;

	return (1U << c->spreading_factor) * sx1262_bw_div(c->bandwidth_hz) >= 8192;
}

/* Integer LoRa time on air, including SX126x SF5/SF6 symbol accounting. */
static inline __u64 sx1262_airtime_quarters(const struct sx1262_config *c, unsigned int len)
{
	int sf = c->spreading_factor;
	int bits = 8 * len + 16 * c->crc - 4 * sf + 20 * !c->implicit_header;
	unsigned int denominator = 4 * sf;

	if (sf >= 7) {
		bits += 8;
		denominator -= 8 * sx1262_ldro(c);
	}

	if (bits < 0)
		bits = 0;

	return 4ULL * (c->preamble_symbols + 12 + (sf <= 6 ? 2 : 0) +
		((bits + denominator - 1) / denominator) * c->coding_rate) + 1;
}

static inline __u32 sx1262_airtime_ms(const struct sx1262_config *c, unsigned int len)
{
	return div_u64(sx1262_airtime_quarters(c, len) * (1U << c->spreading_factor) *
		       sx1262_bw_div(c->bandwidth_hz) + 1999, 2000);
}

static inline __u32 sx1262_rx_timeout_ms(const struct sx1262_config *c,
				       enum sx_rx_stage stage)
{
	__u64 preamble_quarters = 4ULL * c->preamble_symbols + 17;
	__u64 quarters;
	__u32 duration_ms, timeout_ms;

	if (stage == SX_RX_WAITING)
		return 0;

	if (stage == SX_RX_PREAMBLE && !c->implicit_header) {
		quarters = preamble_quarters;
	} else {
		quarters = sx1262_airtime_quarters(c, c->implicit_header ? c->implicit_length : 255);
		/* Implicit RX may never produce HEADER_VALID: allow its entire frame. */
		if (stage == SX_RX_HEADER)
			quarters -= preamble_quarters;
	}

	duration_ms = div_u64(quarters * (1U << c->spreading_factor) *
			      sx1262_bw_div(c->bandwidth_hz) + 1999, 2000);
	/* Original recovery margins: 1.5 times the stage duration + 25 ms, min 100 ms. */
	timeout_ms = duration_ms * 3 / 2 + 25;

	return timeout_ms < 100 ? 100 : timeout_ms;
}

static inline int sx1262_config_valid(const struct sx1262_config *c)
{
	unsigned int i;

	if (c->version != SX1262_ABI_VERSION || c->frequency_hz < 150000000 ||
	    c->frequency_hz > 960000000 || sx1262_bw_code(c->bandwidth_hz) < 0 ||
	    c->spreading_factor < 5 || c->spreading_factor > 12 ||
	    c->coding_rate < 5 || c->coding_rate > 8 || c->crc > 1 ||
	    c->invert_iq > 1 || c->implicit_header > 1 || c->ldro > 2 ||
	    (c->implicit_header && !c->implicit_length) ||
	    c->preamble_symbols < (c->spreading_factor <= 6 ? 12U : 4U) ||
	    c->preamble_symbols > 65535 || !c->hard_reset_after ||
	    c->hard_reset_after > 100 ||
	    (c->silence_timeout_ms && c->silence_timeout_ms < 1000) ||
	    c->silence_timeout_ms > 86400000)
		return 0;

	for (i = 0; i < sizeof(c->reserved); i++)
		if (c->reserved[i])
			return 0;
	/* Keep hardware TX timer and software waits finite (24-bit, 64 kHz). */
	return sx1262_airtime_ms(c, 255) <= 240000;
}
#endif
