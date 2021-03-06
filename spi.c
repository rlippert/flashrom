/*
 * This file is part of the flashrom project.
 *
 * Copyright (C) 2007, 2008, 2009 Carl-Daniel Hailfinger
 * Copyright (C) 2008 coresystems GmbH
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

/*
 * Contains the generic SPI framework
 */

#include <strings.h>
#include <string.h>
#include "flash.h"
#include "flashchips.h"
#include "chipdrivers.h"
#include "programmer.h"
#include "spi.h"

const struct spi_programmer spi_programmer_none = {
	.type = SPI_CONTROLLER_NONE,
	.max_data_read = MAX_DATA_UNSPECIFIED,
	.max_data_write = MAX_DATA_UNSPECIFIED,
	.command = NULL,
	.multicommand = NULL,
	.read = NULL,
	.write_256 = NULL,
};

const struct spi_programmer *spi_programmer = &spi_programmer_none;

int spi_send_command(const struct flashctx *flash, unsigned int writecnt, unsigned int readcnt,
		const unsigned char *writearr, unsigned char *readarr)
{
	if (!spi_programmer->command) {
		msg_perr("%s called, but SPI is unsupported on this "
			 "hardware. Please report a bug at "
			 "flashrom@flashrom.org\n", __func__);
		return 1;
	}

	return spi_programmer->command(flash, writecnt, readcnt,
						      writearr, readarr);
}

int spi_send_multicommand(const struct flashctx *flash, struct spi_command *cmds)
{
	if (!spi_programmer->multicommand) {
		msg_perr("%s called, but SPI is unsupported on this "
			 "hardware. Please report a bug at "
			 "flashrom@flashrom.org\n", __func__);
		return 1;
	}

	return spi_programmer->multicommand(flash, cmds);
}

int default_spi_send_command(const struct flashctx *flash, unsigned int writecnt, unsigned int readcnt,
			     const unsigned char *writearr, unsigned char *readarr)
{
	struct spi_command cmd[] = {
	{
		.writecnt = writecnt,
		.readcnt = readcnt,
		.writearr = writearr,
		.readarr = readarr,
	}, {
		.writecnt = 0,
		.writearr = NULL,
		.readcnt = 0,
		.readarr = NULL,
	}};

	return spi_send_multicommand(flash, cmd);
}

int default_spi_send_multicommand(const struct flashctx *flash, struct spi_command *cmds)
{
	int result = 0;
	for (; (cmds->writecnt || cmds->readcnt) && !result; cmds++) {
		result = spi_send_command(flash, cmds->writecnt, cmds->readcnt,
					  cmds->writearr, cmds->readarr);
	}
	return result;
}

int default_spi_read(struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len)
{
	unsigned int max_data = spi_programmer->max_data_read;
	int rc;
	if (max_data == MAX_DATA_UNSPECIFIED) {
		msg_perr("%s called, but SPI read chunk size not defined "
			 "on this hardware. Please report a bug at "
			 "flashrom@flashrom.org\n", __func__);
		return 1;
	}
	if (flash->feature_bits & FEATURE_UNBOUND_READ)
		rc = spi_read_unbound(flash, buf, start, len, max_data);
	else
		rc = spi_read_chunked(flash, buf, start, len, max_data);
	/* translate SPI-specific access denied error to generic error */
	if (rc == SPI_ACCESS_DENIED)
		rc = ACCESS_DENIED;
	return rc;
}

int default_spi_write_256(struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len)
{
	unsigned int max_data = spi_programmer->max_data_write;
	int rc;
	if (max_data == MAX_DATA_UNSPECIFIED) {
		msg_perr("%s called, but SPI write chunk size not defined "
			 "on this hardware. Please report a bug at "
			 "flashrom@flashrom.org\n", __func__);
		return 1;
	}
	rc = spi_write_chunked(flash, buf, start, len, max_data);
	/* translate SPI-specific access denied error to generic error */
	if (rc == SPI_ACCESS_DENIED)
		rc = ACCESS_DENIED;
	return rc;
}

int spi_chip_read(struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len)
{
	unsigned int addrbase = 0;
	if (!spi_programmer->read) {
		msg_perr("%s called, but SPI read is unsupported on this "
			 "hardware. Please report a bug at "
			 "flashrom@flashrom.org\n", __func__);
		return 1;
	}

	/* Check if the chip fits between lowest valid and highest possible
	 * address. Highest possible address with the current SPI implementation
	 * means 0xffffff, the highest unsigned 24bit number.
	 */
	addrbase = spi_get_valid_read_addr(flash);
	if (addrbase + flash->total_size * 1024 > (1 << 24)) {
		msg_perr("Flash chip size exceeds the allowed access window. ");
		msg_perr("Read will probably fail.\n");
		/* Try to get the best alignment subject to constraints. */
		addrbase = (1 << 24) - flash->total_size * 1024;
	}
	/* Check if alignment is native (at least the largest power of two which
	 * is a factor of the mapped size of the chip).
	 */
	if (ffs(flash->total_size * 1024) > (ffs(addrbase) ? : 33)) {
		msg_perr("Flash chip is not aligned natively in the allowed "
			 "access window.\n");
		msg_perr("Read will probably return garbage.\n");
	}
	return spi_programmer->read(flash, buf, addrbase + start, len);
}


int spi_chip_read_4b(struct flashctx *flash, uint8_t *buf, unsigned int start,
			unsigned int len)
{
	const int chunk_size = 1 << 24;
	int addr = spi_get_valid_read_addr(flash) + start;
	int ret = 0;

	/* If the programmer supports 4 byte addresses use it directly. */
	if (spi_programmer->feature_bits & SPI_FEAT_READ_4BA)
	{
		return spi_programmer->read(flash, buf, start, len);
	}

	/* Otherwise fallback to setting the EAR to access individual 16MB
	 * regions. */
	while (len > 0) {
		const uint8_t ear = (addr & 0xff000000) >> 24;
		unsigned int chunk_addr = addr & 0xffffff;
		unsigned int chunk_len = min(len, chunk_size - chunk_addr);

		ret = spi_write_extended_address(flash, ear);
		if (ret) {
			return ret;
		}

		ret = spi_programmer->read(flash, buf,
						chunk_addr,
						chunk_len);
		if (ret) {
			return ret;
		}

		len -= chunk_len;
		addr += chunk_len;
		buf += chunk_len;
	}

	return ret;
}


/*
 * Program chip using page (256 bytes) programming.
 * Some SPI masters can't do this, they use single byte programming instead.
 * The redirect to single byte programming is achieved by setting
 * .write_256 = spi_chip_write_1
 */
/* real chunksize is up to 256, logical chunksize is 256 */
int spi_chip_write_256(struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len)
{
	if (!spi_programmer->write_256) {
		msg_perr("%s called, but SPI page write is unsupported on this "
			 "hardware. Please report a bug at "
			 "flashrom@flashrom.org\n", __func__);
		return 1;
	}

	return spi_programmer->write_256(flash, buf, start, len);
}

int spi_chip_write_256_4b(struct flashctx *flash, uint8_t *buf,
				unsigned int start,
				unsigned int len)
{
	const int chunk_size = 1 << 24;
	int addr = start;
	int ret = 0;

	/* If the programmer supports 4 byte addresses use it directly. */
	if (spi_programmer->feature_bits & SPI_FEAT_WRITE_4BA)
	{
		return spi_programmer->write_256(flash, buf, start, len);
	}

	/* Otherwise fallback to setting the EAR to access individual 16MB
	 * regions. */
	while (len > 0) {
		const uint8_t ear = (addr & 0xff000000) >> 24;
		unsigned int chunk_addr = addr & 0xffffff;
		unsigned int chunk_len = min(len, chunk_size - chunk_addr);

		ret = spi_write_extended_address(flash, ear);
		if (ret) {
			return ret;
		}

		ret = spi_programmer->write_256(flash, buf,
						chunk_addr,
						chunk_len);
		if (ret) {
			return ret;
		}

		len -= chunk_len;
		addr += chunk_len;
		buf += chunk_len;
	}

	return ret;
}


/*
 * Get the lowest allowed address for read accesses. This often happens to
 * be the lowest allowed address for all commands which take an address.
 * This is a programmer limitation.
 */
uint32_t spi_get_valid_read_addr(struct flashctx *flash)
{
	switch (spi_programmer->type) {
#if CONFIG_INTERNAL == 1
#if defined(__i386__) || defined(__x86_64__)
	case SPI_CONTROLLER_ICH7:
		/* Return BBAR for ICH chipsets. */
		return ichspi_bbar;
#endif
#endif
	default:
		return 0;
	}
}

void register_spi_programmer(const struct spi_programmer *pgm)
{
	spi_programmer = pgm;
	buses_supported |= BUS_SPI;
}
