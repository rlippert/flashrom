/*
 * This file is part of the flashrom project.
 *
 * Copyright (C) 2011 Sven Schnelle <svens@stackframe.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/fcntl.h>
#include <errno.h>
#include <ctype.h>
#include <unistd.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "flash.h"
#include "chipdrivers.h"
#include "programmer.h"
#include "spi.h"



/* TODO: this information should come from the SPI master driver. */
#define SPI_DMA_SIZE 64


static int fd = -1;

static int linux_spi_shutdown(void *data);
static int linux_spi_send_command(unsigned int writecnt, unsigned int readcnt,
			const unsigned char *txbuf, unsigned char *rxbuf);
static int linux_spi_read(struct flashchip *flash, uint8_t *buf,
			  unsigned int start, unsigned int len);
static int linux_spi_write_256(struct flashchip *flash, uint8_t *buf,
			       unsigned int start, unsigned int len);

static const struct spi_programmer spi_programmer_linux = {
	.type		= SPI_CONTROLLER_LINUX,
	.max_data_read	= MAX_DATA_UNSPECIFIED, /* TODO? */
	.max_data_write	= MAX_DATA_UNSPECIFIED, /* TODO? */
	.command	= linux_spi_send_command,
	.multicommand	= default_spi_send_multicommand,
	.read		= linux_spi_read,
	.write_256	= linux_spi_write_256,
};


/* Detect the SPI chip behind /dev/spidevX.Y.
 * We scan every /dev/spidevX.0 and if it's /dev/spidevX.1 is a MTD device
 * (by looking /sys/bus/spi/devices/spidevX.1/modalias), then we assume the
 * spidevX.0 is the SPI flash.
 */
static char* linux_spi_probe(void)
{
	char name0[] = "/sys/bus/spi/devices/spiX.0";
	char name1[] = "/sys/bus/spi/devices/spiX.1/modalias";
	int X = strchr(name0, 'X') - name0;  /* point to the X char */
	int x;  /* for for-loop */
	struct stat sb;

	for (x = '0'; x <= '9'; x++) {
		name0[X] = x;
		if (stat(name0, &sb) < 0) {
			msg_pdbg("stat(%s) < 0, stop scanning.\n", name0);
			return NULL;
		}

		if ((sb.st_mode & S_IFMT) == S_IFDIR) {
			int fd;
			char buf[8];  // 8 is long enough for modalias

			name1[X] = x;
			if ((fd = open(name1, O_RDONLY)) < 0 ||
			    read(fd, buf, sizeof(buf)) < 0) {
				msg_pdbg("read(%s) < 0, try next.\n", name0);
				continue;
			}

			if (!strncmp(buf, "m25p80", 6)) {
				static char name[] = "/dev/spidevX.0";
				*strchr(name, 'X') = x;
				msg_pdbg("Detected linux_spi:dev=%s\n", name);
				return name;
			}

			close(fd);
		}
	}

	return NULL;
}


int linux_spi_init(void)
{
	char *p, *endp, *dev;
	uint32_t speed = 0;

	dev = extract_programmer_param("dev");
	if (!dev) {
		dev = linux_spi_probe();
	}
	if (!dev || !strlen(dev)) {
		msg_perr("No SPI device given. Use flashrom -p "
			 "linux_spi:dev=/dev/spidevX.Y\n");
		return 1;
	}

	p = extract_programmer_param("speed");
	if (p && strlen(p)) {
		speed = (uint32_t)strtoul(p, &endp, 10) * 1024;
		if (p == endp) {
			msg_perr("%s: invalid clock: %s kHz\n", __func__, p);
			return 1;
		}
	}

	msg_pdbg("Using device %s\n", dev);
	if ((fd = open(dev, O_RDWR)) == -1) {
		msg_perr("%s: failed to open %s: %s\n", __func__,
			 dev, strerror(errno));
		return 1;
	}

	if (speed > 0) {
		if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1) {
			msg_perr("%s: failed to set speed %dHz: %s\n",
				 __func__, speed, strerror(errno));
			close(fd);
			return 1;
		}

		msg_pdbg("Using %d kHz clock\n", speed);
	}

	if (register_shutdown(linux_spi_shutdown, NULL))
		return 1;

	register_spi_programmer(&spi_programmer_linux);

	return 0;
}

static int linux_spi_shutdown(void *data)
{
	if (fd != -1) {
		close(fd);
		fd = -1;
	}
	return 0;
}

static int linux_spi_send_command(unsigned int writecnt, unsigned int readcnt,
			const unsigned char *txbuf, unsigned char *rxbuf)
{
	int msg_start = 0, msg_count = 0;
	struct spi_ioc_transfer msg[2] = {
		{
			.tx_buf = (uint64_t)(ptrdiff_t)txbuf,
			.len = writecnt,
		},
		{
			.rx_buf = (uint64_t)(ptrdiff_t)rxbuf,
			.len = readcnt,
		},
	};

	if (fd == -1)
		return -1;

	/* Only pass necessary msg[] to ioctl() to avoid the empty message
	 * which drives an un-expected CS line and clocks. */
	if (writecnt) {
		msg_start = 0;  /* tx: msg[0] */
		msg_count++;
		if (readcnt) {
			msg_count++;
		}
	} else if (readcnt) {
		msg_start = 1;  /* rx: msg[1] */
		msg_count++;
	} else {
		msg_cerr("%s: both writecnt and readcnt are 0.\n",
			 __func__);
		return -1;
	}

	if (ioctl(fd, SPI_IOC_MESSAGE(msg_count), &msg[msg_start]) == -1) {
		msg_cerr("%s: ioctl: %s\n", __func__, strerror(errno));
		return -1;
	}
	return 0;
}

static int linux_spi_read(struct flashchip *flash, uint8_t *buf,
			  unsigned int start, unsigned int len)
{
	return spi_read_chunked(flash, buf, start, len, SPI_DMA_SIZE);
}

static int linux_spi_write_256(struct flashchip *flash, uint8_t *buf,
			       unsigned int start, unsigned int len)
{
	return spi_write_chunked(flash, buf, start, len,
				 SPI_DMA_SIZE - 5 /* WREN, Page Program */);
}
