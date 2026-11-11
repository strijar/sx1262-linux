// SPDX-License-Identifier: GPL-2.0-only
/* Minimal raw-PHY exerciser. Does not implement any network protocol. */
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/sx1262.h>

static void fail(const char *what)
{
	perror(what);
	exit(EXIT_FAILURE);
}

static long number(const char *s, long min, long max)
{
	char *end;
	long value;

	errno = 0;
	value = strtol(s, &end, 0);
	if (errno || !*s || *end || value < min || value > max) {
		fprintf(stderr, "Invalid number: %s\n", s);
		exit(EXIT_FAILURE);
	}
	return value;
}

static int hex_digit(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}

int main(int argc, char **argv)
{
	struct sx1262_config config;
	struct sx1262_tx_batch batch = { .version = 1, .cookie = 1 };
	struct sx1262_event event;
	struct sx1262_stats stats;
	struct pollfd pfd;
	int fd, i, result = EXIT_SUCCESS;
	unsigned int j;

	if (argc < 6 || argc > 6 + SX1262_MAX_BATCH) {
		fprintf(stderr, "Usage: %s DEVICE HZ BW SF POWER_DBM [HEX_FRAME ...]\n"
			"Without frames: continuous RX. With frames: one atomic TX batch.\n"
			"Other PHY fields use driver defaults (preamble=25, sync=1424, CRC, CR=5).\n",
			argv[0]);
		return EXIT_FAILURE;
	}
	fd = open(argv[1], O_RDWR | O_NONBLOCK);
	if (fd < 0)
		fail("open");
	if (ioctl(fd, SX1262_IOC_GET_CONFIG, &config) < 0)
		fail("GET_CONFIG");
	config.frequency_hz = number(argv[2], 150000000, 960000000);
	config.bandwidth_hz = number(argv[3], 7800, 500000);
	config.spreading_factor = number(argv[4], 5, 12);
	config.tx_power_dbm = number(argv[5], -40, 40);
	if (ioctl(fd, SX1262_IOC_SET_CONFIG, &config) < 0)
		fail("SET_CONFIG");
	batch.count = argc - 6;
	for (i = 6; i < argc; i++) {
		struct sx1262_frame *f = &batch.frames[i - 6];
		size_t len = strlen(argv[i]);

		if (!len || len % 2 || len > SX1262_MAX_PAYLOAD * 2) {
			fprintf(stderr, "Frame must contain 1..255 bytes in hex\n");
			return EXIT_FAILURE;
		}
		f->length = len / 2;
		for (j = 0; j < f->length; j++) {
			int hi = hex_digit(argv[i][2 * j]);
			int lo = hex_digit(argv[i][2 * j + 1]);

			if (hi < 0 || lo < 0) {
				fprintf(stderr, "Invalid hexadecimal frame\n");
				return EXIT_FAILURE;
			}
			f->data[j] = hi * 16 + lo;
		}
	}
	if (batch.count && ioctl(fd, SX1262_IOC_TX_BATCH, &batch) < 0)
		fail("TX_BATCH");
	pfd = (struct pollfd){ .fd = fd, .events = POLLIN };
	for (;;) {
		ssize_t size;

		if (poll(&pfd, 1, -1) < 0) {
			if (errno == EINTR)
				continue;
			fail("poll");
		}
		if (pfd.revents & POLLHUP) {
			fprintf(stderr, "Device removed\n");
			result = EXIT_FAILURE;
			break;
		}
		if (!(pfd.revents & POLLIN) && (pfd.revents & POLLERR)) {
			fprintf(stderr, "Radio offline\n");
			result = EXIT_FAILURE;
			break;
		}
		size = read(fd, &event, sizeof(event));
		if (size < 0 && errno == EAGAIN)
			continue;
		if (size < 0)
			fail("read");
		if (size != sizeof(event) || event.version != SX1262_ABI_VERSION) {
			fprintf(stderr, "Unexpected event ABI\n");
			return EXIT_FAILURE;
		}
		if (event.type == SX1262_EVENT_RX) {
			printf("RX len=%u RSSI=%.1f dBm SNR=%.2f dB ", event.length,
			       event.rssi_half_dbm / 2.0, event.snr_quarter_db / 4.0);
			for (j = 0; j < event.length; j++)
				printf("%02x", event.data[j]);
			putchar('\n');
			fflush(stdout);
		} else if (event.type == SX1262_EVENT_TX) {
			printf("TX cookie=%" PRIu64 " sent=%u status=%d\n",
			       (uint64_t)event.cookie, event.frames_sent, event.status);
			result = event.status ? EXIT_FAILURE : EXIT_SUCCESS;
			break;
		}
	}
	if (!ioctl(fd, SX1262_IOC_GET_STATS, &stats))
		printf("RX=%" PRIu64 " dropped=%" PRIu64 " TX=%" PRIu64
		       " recoveries=%" PRIu64 "/%" PRIu64 " polled=%" PRIu64 "\n",
		       (uint64_t)stats.rx_packets, (uint64_t)stats.rx_dropped,
		       (uint64_t)stats.tx_packets, (uint64_t)stats.soft_recoveries,
		       (uint64_t)stats.hard_recoveries, (uint64_t)stats.polled_irqs);
	close(fd);
	return result;
}
