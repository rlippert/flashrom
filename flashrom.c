/*
 * This file is part of the flashrom project.
 *
 * Copyright (C) 2000 Silicon Integrated System Corporation
 * Copyright (C) 2004 Tyan Corp <yhlu@tyan.com>
 * Copyright (C) 2005-2008 coresystems GmbH
 * Copyright (C) 2008,2009 Carl-Daniel Hailfinger
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
#include <sys/types.h>
#ifndef __LIBPAYLOAD__
#include <fcntl.h>
#include <sys/stat.h>
#endif
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <getopt.h>
#if HAVE_UTSNAME == 1
#include <sys/utsname.h>
#endif
#include <unistd.h>
#include "flash.h"
#include "flashchips.h"
#include "layout.h"
#include "programmer.h"

const char flashrom_version[] = FLASHROM_VERSION;
char *chip_to_probe = NULL;
int verbose = 0;

unsigned int required_erase_size = 0;	/* see comment in flash.h */

/* Set if any erase/write operation is to be done. This will be used to
 * decide if final verification is needed. */
static int content_has_changed = 0;

/* error handling stuff */
enum error_action access_denied_action = error_ignore;

int ignore_error(int err) {
	int rc = 0;

	switch(err) {
	case ACCESS_DENIED:
		if (access_denied_action == error_ignore)
			rc = 1;
		break;
	default:
		break;
	}

	return rc;
}

static enum programmer programmer = PROGRAMMER_INVALID;

static char *programmer_param = NULL;

/* Supported buses for the current programmer. */
enum chipbustype buses_supported;

/*
 * Programmers supporting multiple buses can have differing size limits on
 * each bus. Store the limits for each bus in a common struct.
 */
struct decode_sizes max_rom_decode;

/* If nonzero, used as the start address of bottom-aligned flash. */
unsigned long flashbase;

/* Is writing allowed with this programmer? */
int programmer_may_write;

const struct programmer_entry programmer_table[] = {
#if CONFIG_INTERNAL == 1
	{
		.name			= "internal",
		.init			= internal_init,
		.map_flash_region	= physmap,
		.unmap_flash_region	= physunmap,
		.delay			= internal_delay,

		/*
		 * "Internal" implies in-system programming on a live system, so
		 * handle with paranoia to catch errors early. If something goes
		 * wrong then hopefully the system will still be recoverable.
		 */
		.paranoid		= 1,
	},
#endif

#if CONFIG_DUMMY == 1
	{
		.name			= "dummy",
		.init			= dummy_init,
		.map_flash_region	= dummy_map,
		.unmap_flash_region	= dummy_unmap,
		.delay			= internal_delay,
	},
#endif

#if CONFIG_NIC3COM == 1
	{
		.name			= "nic3com",
		.init			= nic3com_init,
		.map_flash_region	= fallback_map,
		.unmap_flash_region	= fallback_unmap,
		.delay			= internal_delay,
	},
#endif

#if CONFIG_NICREALTEK == 1
	{
		/* This programmer works for Realtek RTL8139 and SMC 1211. */
		.name			= "nicrealtek",
		//.name			= "nicsmc1211",
		.init			= nicrealtek_init,
		.map_flash_region	= fallback_map,
		.unmap_flash_region	= fallback_unmap,
		.delay			= internal_delay,
	},
#endif

#if CONFIG_NICNATSEMI == 1
	{
		.name			= "nicnatsemi",
		.init			= nicnatsemi_init,
		.map_flash_region	= fallback_map,
		.unmap_flash_region	= fallback_unmap,
		.delay			= internal_delay,
	},
#endif

#if CONFIG_GFXNVIDIA == 1
	{
		.name			= "gfxnvidia",
		.init			= gfxnvidia_init,
		.map_flash_region	= fallback_map,
		.unmap_flash_region	= fallback_unmap,
		.delay			= internal_delay,
	},
#endif

#if CONFIG_DRKAISER == 1
	{
		.name			= "drkaiser",
		.init			= drkaiser_init,
		.map_flash_region	= fallback_map,
		.unmap_flash_region	= fallback_unmap,
		.delay			= internal_delay,
	},
#endif

#if CONFIG_SATASII == 1
	{
		.name			= "satasii",
		.init			= satasii_init,
		.map_flash_region	= fallback_map,
		.unmap_flash_region	= fallback_unmap,
		.delay			= internal_delay,
	},
#endif

#if CONFIG_ATAHPT == 1
	{
		.name			= "atahpt",
		.init			= atahpt_init,
		.map_flash_region	= fallback_map,
		.unmap_flash_region	= fallback_unmap,
		.delay			= internal_delay,
	},
#endif

#if CONFIG_FT2232_SPI == 1
	{
		.name			= "ft2232_spi",
		.init			= ft2232_spi_init,
		.map_flash_region	= fallback_map,
		.unmap_flash_region	= fallback_unmap,
		.delay			= internal_delay,
	},
#endif

#if CONFIG_SERPROG == 1
	{
		.name			= "serprog",
		.init			= serprog_init,
		.map_flash_region	= fallback_map,
		.unmap_flash_region	= fallback_unmap,
		.delay			= serprog_delay,
	},
#endif

#if CONFIG_BUSPIRATE_SPI == 1
	{
		.name			= "buspirate_spi",
		.init			= buspirate_spi_init,
		.map_flash_region	= fallback_map,
		.unmap_flash_region	= fallback_unmap,
		.delay			= internal_delay,
	},
#endif

#if CONFIG_RAIDEN_DEBUG_SPI == 1
	{
		.name			= "raiden_debug_spi",
		.init			= raiden_debug_spi_init,
		.map_flash_region	= fallback_map,
		.unmap_flash_region	= fallback_unmap,
		.delay			= internal_delay,
	},
#endif

#if CONFIG_DEDIPROG == 1
	{
		.name			= "dediprog",
		.init			= dediprog_init,
		.map_flash_region	= fallback_map,
		.unmap_flash_region	= fallback_unmap,
		.delay			= internal_delay,
	},
#endif

#if CONFIG_RAYER_SPI == 1
	{
		.name			= "rayer_spi",
		.init			= rayer_spi_init,
		.map_flash_region	= fallback_map,
		.unmap_flash_region	= fallback_unmap,
		.delay			= internal_delay,
	},
#endif

#if CONFIG_NICINTEL == 1
	{
		.name			= "nicintel",
		.init			= nicintel_init,
		.map_flash_region	= fallback_map,
		.unmap_flash_region	= fallback_unmap,
		.delay			= internal_delay,
	},
#endif

#if CONFIG_NICINTEL_SPI == 1
	{
		.name			= "nicintel_spi",
		.init			= nicintel_spi_init,
		.map_flash_region	= fallback_map,
		.unmap_flash_region	= fallback_unmap,
		.delay			= internal_delay,
	},
#endif

#if CONFIG_OGP_SPI == 1
	{
		.name			= "ogp_spi",
		.init			= ogp_spi_init,
		.map_flash_region	= fallback_map,
		.unmap_flash_region	= fallback_unmap,
		.delay			= internal_delay,
	},
#endif

#if CONFIG_SATAMV == 1
	{
		.name			= "satamv",
		.init			= satamv_init,
		.map_flash_region	= fallback_map,
		.unmap_flash_region	= fallback_unmap,
		.delay			= internal_delay,
	},
#endif

#if CONFIG_LINUX_MTD == 1
	{
		.name			= "linux_mtd",
		.init			= linux_mtd_init,
		.map_flash_region	= fallback_map,
		.unmap_flash_region	= fallback_unmap,
		.delay			= internal_delay,
	},
#endif

#if CONFIG_LINUX_SPI == 1
	{
		.name			= "linux_spi",
		.init			= linux_spi_init,
		.map_flash_region	= fallback_map,
		.unmap_flash_region	= fallback_unmap,
		.delay			= internal_delay,
	},
#endif

#if CONFIG_LPC2SPI_SFC == 1
	{
		.name			= "lpc2spi_sfc",
		.init			= flashrom_sfc_init,
		.map_flash_region	= fallback_map,
		.unmap_flash_region	= fallback_unmap,
		.delay			= internal_delay,
		.probe_chip		= flashrom_sfc_probe_chip,
	},
#endif
	{}, /* This entry corresponds to PROGRAMMER_INVALID. */
};

#define CHIP_RESTORE_MAXFN 4
static int chip_restore_fn_count = 0;
struct chip_restore_func_data {
	CHIP_RESTORE_CALLBACK;
	struct flashctx *flash;
	uint8_t status;
} static chip_restore_fn[CHIP_RESTORE_MAXFN];


#define SHUTDOWN_MAXFN 32
static int shutdown_fn_count = 0;
struct shutdown_func_data {
	int (*func) (void *data);
	void *data;
} static shutdown_fn[SHUTDOWN_MAXFN];
/* Initialize to 0 to make sure nobody registers a shutdown function before
 * programmer init.
 */
static int may_register_shutdown = 0;

static int check_block_eraser(const struct flashctx *flash, int k, int log);

/* Register a function to be executed on programmer shutdown.
 * The advantage over atexit() is that you can supply a void pointer which will
 * be used as parameter to the registered function upon programmer shutdown.
 * This pointer can point to arbitrary data used by said function, e.g. undo
 * information for GPIO settings etc. If unneeded, set data=NULL.
 * Please note that the first (void *data) belongs to the function signature of
 * the function passed as first parameter.
 */
int register_shutdown(int (*function) (void *data), void *data)
{
	if (shutdown_fn_count >= SHUTDOWN_MAXFN) {
		msg_perr("Tried to register more than %i shutdown functions.\n",
			 SHUTDOWN_MAXFN);
		return 1;
	}
	if (!may_register_shutdown) {
		msg_perr("Tried to register a shutdown function before "
			 "programmer init.\n");
		return 1;
	}
	shutdown_fn[shutdown_fn_count].func = function;
	shutdown_fn[shutdown_fn_count].data = data;
	shutdown_fn_count++;

	return 0;
}

//int register_chip_restore(int (*function) (void *data), void *data)
int register_chip_restore(CHIP_RESTORE_CALLBACK,
                          struct flashctx *flash, uint8_t status)
{
	if (chip_restore_fn_count >= CHIP_RESTORE_MAXFN) {
		msg_perr("Tried to register more than %i chip restore"
		         " functions.\n", CHIP_RESTORE_MAXFN);
		return 1;
	}
	chip_restore_fn[chip_restore_fn_count].func = func;	/* from macro */
	chip_restore_fn[chip_restore_fn_count].flash = flash;
	chip_restore_fn[chip_restore_fn_count].status = status;
	chip_restore_fn_count++;

	return 0;
}

int programmer_init(enum programmer prog, char *param)
{
	int ret;

	if (prog >= PROGRAMMER_INVALID) {
		msg_perr("Invalid programmer specified!\n");
		return -1;
	}
	programmer = prog;
	/* Initialize all programmer specific data. */
	/* Default to unlimited decode sizes. */
	max_rom_decode = (const struct decode_sizes) {
		.parallel	= 0xffffffff,
		.lpc		= 0xffffffff,
		.fwh		= 0xffffffff,
		.spi		= 0xffffffff,
	};
	buses_supported = BUS_NONE;
	/* Default to top aligned flash at 4 GB. */
	flashbase = 0;
	/* Registering shutdown functions is now allowed. */
	may_register_shutdown = 1;
	/* Default to allowing writes. Broken programmers set this to 0. */
	programmer_may_write = 1;

	programmer_param = param;
	msg_pdbg("Initializing %s programmer\n",
		 programmer_table[programmer].name);
	ret = programmer_table[programmer].init();
	if (programmer_param && strlen(programmer_param)) {
		msg_perr("Unhandled programmer parameters: %s\n",
			 programmer_param);
		/* Do not error out here, the init itself was successful. */
	}
	return ret;
}

int chip_restore()
{
	int rc = 0;

	while (chip_restore_fn_count > 0) {
		int i = --chip_restore_fn_count;
		rc |= chip_restore_fn[i].func(chip_restore_fn[i].flash,
		                              chip_restore_fn[i].status);
	}

	return rc;
}

int programmer_shutdown(void)
{
	int ret = 0;

	/* Registering shutdown functions is no longer allowed. */
	may_register_shutdown = 0;
	while (shutdown_fn_count > 0) {
		int i = --shutdown_fn_count;
		ret |= shutdown_fn[i].func(shutdown_fn[i].data);
	}
	return ret;
}

void *programmer_map_flash_region(const char *descr, unsigned long phys_addr,
				  size_t len)
{
	return programmer_table[programmer].map_flash_region(descr,
							     phys_addr, len);
}

void programmer_unmap_flash_region(void *virt_addr, size_t len)
{
	programmer_table[programmer].unmap_flash_region(virt_addr, len);
}

void chip_writeb(const struct flashctx *flash, uint8_t val, chipaddr addr)
{
	par_programmer->chip_writeb(flash, val, addr);
}

void chip_writew(const struct flashctx *flash, uint16_t val, chipaddr addr)
{
	par_programmer->chip_writew(flash, val, addr);
}

void chip_writel(const struct flashctx *flash, uint32_t val, chipaddr addr)
{
	par_programmer->chip_writel(flash, val, addr);
}

void chip_writen(const struct flashctx *flash, uint8_t *buf, chipaddr addr, size_t len)
{
	par_programmer->chip_writen(flash, buf, addr, len);
}

uint8_t chip_readb(const struct flashctx *flash, const chipaddr addr)
{
	return par_programmer->chip_readb(flash, addr);
}

uint16_t chip_readw(const struct flashctx *flash, const chipaddr addr)
{
	return par_programmer->chip_readw(flash, addr);
}

uint32_t chip_readl(const struct flashctx *flash, const chipaddr addr)
{
	return par_programmer->chip_readl(flash, addr);
}

void chip_readn(const struct flashctx *flash, uint8_t *buf, chipaddr addr, size_t len)
{
	par_programmer->chip_readn(flash, buf, addr, len);
}

void programmer_delay(int usecs)
{
	programmer_table[programmer].delay(usecs);
}

void map_flash_registers(struct flashctx *flash)
{
	size_t size = flash->total_size * 1024;
	/* Flash registers live 4 MByte below the flash. */
	/* FIXME: This is incorrect for nonstandard flashbase. */
	flash->virtual_registers = (chipaddr)programmer_map_flash_region("flash chip registers", (0xFFFFFFFF - 0x400000 - size + 1), size);
}

int read_memmapped(struct flashctx *flash, uint8_t *buf, unsigned int start, int unsigned len)
{
	chip_readn(flash, buf, flash->virtual_memory + start, len);

	return 0;
}

int min(int a, int b)
{
	return (a < b) ? a : b;
}

int max(int a, int b)
{
	return (a > b) ? a : b;
}

int bitcount(unsigned long a)
{
	int i = 0;
	for (; a != 0; a >>= 1)
		if (a & 1)
			i++;
	return i;
}

void tolower_string(char *str)
{
	for (; *str != '\0'; str++)
		*str = (char)tolower((unsigned char)*str);
}

char *strcat_realloc(char *dest, const char *src)
{
	dest = realloc(dest, strlen(dest) + strlen(src) + 1);
	if (!dest) {
		msg_gerr("Out of memory!\n");
		return NULL;
	}
	strcat(dest, src);
	return dest;
}

/* This is a somewhat hacked function similar in some ways to strtok().
 * It will look for needle with a subsequent '=' in haystack, return a copy of
 * needle and remove everything from the first occurrence of needle to the next
 * delimiter from haystack.
 */
char *extract_param(char **haystack, const char *needle, const char *delim)
{
	char *param_pos, *opt_pos, *rest;
	char *opt = NULL;
	int optlen;
	int needlelen;

	needlelen = strlen(needle);
	if (!needlelen) {
		msg_gerr("%s: empty needle! Please report a bug at "
			 "flashrom@flashrom.org\n", __func__);
		return NULL;
	}
	/* No programmer parameters given. */
	if (*haystack == NULL)
		return NULL;
	param_pos = strstr(*haystack, needle);
	do {
		if (!param_pos)
			return NULL;
		/* Needle followed by '='? */
		if (param_pos[needlelen] == '=') {

			/* Beginning of the string? */
			if (param_pos == *haystack)
				break;
			/* After a delimiter? */
			if (strchr(delim, *(param_pos - 1)))
				break;
		}
		/* Continue searching. */
		param_pos++;
		param_pos = strstr(param_pos, needle);
	} while (1);

	if (param_pos) {
		/* Get the string after needle and '='. */
		opt_pos = param_pos + needlelen + 1;
		optlen = strcspn(opt_pos, delim);
		/* Return an empty string if the parameter was empty. */
		opt = malloc(optlen + 1);
		if (!opt) {
			msg_gerr("Out of memory!\n");
			exit(1);
		}
		strncpy(opt, opt_pos, optlen);
		opt[optlen] = '\0';
		rest = opt_pos + optlen;
		/* Skip all delimiters after the current parameter. */
		rest += strspn(rest, delim);
		memmove(param_pos, rest, strlen(rest) + 1);
		/* We could shrink haystack, but the effort is not worth it. */
	}

	return opt;
}

char *extract_programmer_param(const char *param_name)
{
	return extract_param(&programmer_param, param_name, ",");
}

/* Returns the number of well-defined erasers for a chip. */
static unsigned int count_usable_erasers(const struct flashctx *flash)
{
	unsigned int usable_erasefunctions = 0;
	int k;
	for (k = 0; k < NUM_ERASEFUNCTIONS; k++) {
		if (!check_block_eraser(flash, k, 0))
			usable_erasefunctions++;
	}
	return usable_erasefunctions;
}

/* start is an offset to the base address of the flash chip */
int check_erased_range(struct flashctx *flash, unsigned int start, unsigned int len)
{
	int ret;
	uint8_t *cmpbuf = malloc(len);

	if (!cmpbuf) {
		msg_gerr("Could not allocate memory!\n");
		exit(1);
	}
	memset(cmpbuf, flash_erase_value(flash), len);
	ret = verify_range(flash, cmpbuf, start, len, "ERASE");
	free(cmpbuf);
	return ret;
}

static int compare_chunk(uint8_t *readbuf, uint8_t *cmpbuf, unsigned int start,
					unsigned int len, const char *message)
{
	int failcount = 0, i;

	for (i = 0; i < len; i++) {
		if (cmpbuf[i] != readbuf[i]) {
			if (!failcount) {
				msg_cerr("%s FAILED at 0x%08x! "
					"Expected=0x%02x, Read=0x%02x,",
					message, start + i,
					cmpbuf[i], readbuf[i]);
			}
			failcount++;
		}
	}

	return failcount;
}

/*
 * @cmpbuf	buffer to compare against, cmpbuf[0] is expected to match the
 *		flash content at location start
 * @start	offset to the base address of the flash chip
 * @len		length of the verified area
 * @message	string to print in the "FAILED" message
 * @return	0 for success, -1 for failure
 */
int verify_range(struct flashctx *flash, uint8_t *cmpbuf, unsigned int start, unsigned int len,
		 const char *message)
{
	uint8_t *readbuf = malloc(len);
	int ret = 0, failcount = 0;

	if (!len)
		goto out_free;

	if (!flash->read) {
		msg_cerr("ERROR: flashrom has no read function for this flash chip.\n");
		return 1;
	}
	if (!readbuf) {
		msg_gerr("Could not allocate memory!\n");
		exit(1);
	}

	if (start + len > flash->total_size * 1024) {
		msg_gerr("Error: %s called with start 0x%x + len 0x%x >"
			" total_size 0x%x\n", __func__, start, len,
			flash->total_size * 1024);
		ret = -1;
		goto out_free;
	}
	if (!message)
		message = "VERIFY";

	if (programmer_table[programmer].paranoid) {
		unsigned int i, chunksize;

		/* limit chunksize in order to catch errors early */
		for (i = 0, chunksize = 0; i < len; i += chunksize) {
			int tmp;

			chunksize = min(flash->page_size, len - i);
			tmp = flash->read(flash, readbuf + i, start + i, chunksize);
			if (tmp) {
				ret = tmp;
				if (ignore_error(tmp))
					continue;
				else
					goto out_free;
			}

			failcount = compare_chunk(readbuf + i, cmpbuf + i, start + i,
					chunksize, message);
			if (failcount)
				break;
		}
	} else {
		int tmp;

		/* read as much as we can to reduce transaction overhead */
		tmp = flash->read(flash, readbuf, start, len);
		if (tmp && !ignore_error(tmp)) {
			ret = tmp;
			goto out_free;
		}

		failcount = compare_chunk(readbuf, cmpbuf, start, len, message);
	}

	if (failcount) {
		msg_cerr(" failed byte count from 0x%08x-0x%08x: 0x%x\n",
			 start, start + len - 1, failcount);
		ret = -1;
	}

out_free:
	free(readbuf);
	return ret;
}

/*
 * Check if the buffer @have can be programmed to the content of @want without
 * erasing. This is only possible if all chunks of size @gran are either kept
 * as-is or changed from an all-ones state to any other state.
 *
 * The following write granularities (enum @gran) are known:
 * - 1 bit. Each bit can be cleared individually.
 * - 1 byte. A byte can be written once. Further writes to an already written
 *   byte cause the contents to be either undefined or to stay unchanged.
 * - 128 bytes. If less than 128 bytes are written, the rest will be
 *   erased. Each write to a 128-byte region will trigger an automatic erase
 *   before anything is written. Very uncommon behaviour and unsupported by
 *   this function.
 * - 256 bytes. If less than 256 bytes are written, the contents of the
 *   unwritten bytes are undefined.
 * Warning: This function assumes that @have and @want point to naturally
 * aligned regions.
 *
 * @have        buffer with current content
 * @want        buffer with desired content
 * @len		length of the checked area
 * @gran	write granularity (enum, not count)
 * @return      0 if no erase is needed, 1 otherwise
 */
static int need_erase(struct flashctx *flash, uint8_t *have, uint8_t *want,
		      unsigned int len, enum write_granularity gran)
{
	int result = 0;
	unsigned int i, j, limit;
	int erase_value = flash_erase_value(flash);

	switch (gran) {
	case write_gran_1bit:
		for (i = 0; i < len; i++)
			if ((have[i] & want[i]) != want[i]) {
				result = 1;
				break;
			}
		break;
	case write_gran_1byte:
		for (i = 0; i < len; i++)
			if ((have[i] != want[i]) && (have[i] != erase_value)) {
				result = 1;
				break;
			}
		break;
	case write_gran_256bytes:
		for (j = 0; j < len / 256; j++) {
			limit = min (256, len - j * 256);
			/* Are 'have' and 'want' identical? */
			if (!memcmp(have + j * 256, want + j * 256, limit))
				continue;
			/* have needs to be in erased state. */
			for (i = 0; i < limit; i++)
				if (have[j * 256 + i] != erase_value) {
					result = 1;
					break;
				}
			if (result)
				break;
		}
		break;
	default:
		msg_cerr("%s: Unsupported granularity! Please report a bug at "
			 "flashrom@flashrom.org\n", __func__);
	}
	return result;
}

/**
 * Check if the buffer @have needs to be programmed to get the content of @want.
 * If yes, return 1 and fill in first_start with the start address of the
 * write operation and first_len with the length of the first to-be-written
 * chunk. If not, return 0 and leave first_start and first_len undefined.
 *
 * Warning: This function assumes that @have and @want point to naturally
 * aligned regions.
 *
 * @have	buffer with current content
 * @want	buffer with desired content
 * @len		length of the checked area
 * @gran	write granularity (enum, not count)
 * @first_start	offset of the first byte which needs to be written (passed in
 *		value is increased by the offset of the first needed write
 *		relative to have/want or unchanged if no write is needed)
 * @return	length of the first contiguous area which needs to be written
 *		0 if no write is needed
 *
 * FIXME: This function needs a parameter which tells it about coalescing
 * in relation to the max write length of the programmer and the max write
 * length of the chip.
 */
static unsigned int get_next_write(uint8_t *have, uint8_t *want, unsigned int len,
			  unsigned int *first_start,
			  enum write_granularity gran)
{
	int need_write = 0;
	unsigned int rel_start = 0, first_len = 0;
	unsigned int i, limit, stride;

	switch (gran) {
	case write_gran_1bit:
	case write_gran_1byte:
		stride = 1;
		break;
	case write_gran_256bytes:
		stride = 256;
		break;
	default:
		msg_cerr("%s: Unsupported granularity! Please report a bug at "
			 "flashrom@flashrom.org\n", __func__);
		/* Claim that no write was needed. A write with unknown
		 * granularity is too dangerous to try.
		 */
		return 0;
	}
	for (i = 0; i < len / stride; i++) {
		limit = min(stride, len - i * stride);
		/* Are 'have' and 'want' identical? */
		if (memcmp(have + i * stride, want + i * stride, limit)) {
			if (!need_write) {
				/* First location where have and want differ. */
				need_write = 1;
				rel_start = i * stride;
			}
		} else {
			if (need_write) {
				/* First location where have and want
				 * do not differ anymore.
				 */
				break;
			}
		}
	}
	if (need_write)
		first_len = min(i * stride - rel_start, len);
	*first_start += rel_start;
	return first_len;
}

/* This function generates various test patterns useful for testing controller
 * and chip communication as well as chip behaviour.
 *
 * If a byte can be written multiple times, each time keeping 0-bits at 0
 * and changing 1-bits to 0 if the new value for that bit is 0, the effect
 * is essentially an AND operation. That's also the reason why this function
 * provides the result of AND between various patterns.
 *
 * Below is a list of patterns (and their block length).
 * Pattern 0 is 05 15 25 35 45 55 65 75 85 95 a5 b5 c5 d5 e5 f5 (16 Bytes)
 * Pattern 1 is 0a 1a 2a 3a 4a 5a 6a 7a 8a 9a aa ba ca da ea fa (16 Bytes)
 * Pattern 2 is 50 51 52 53 54 55 56 57 58 59 5a 5b 5c 5d 5e 5f (16 Bytes)
 * Pattern 3 is a0 a1 a2 a3 a4 a5 a6 a7 a8 a9 aa ab ac ad ae af (16 Bytes)
 * Pattern 4 is 00 10 20 30 40 50 60 70 80 90 a0 b0 c0 d0 e0 f0 (16 Bytes)
 * Pattern 5 is 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f (16 Bytes)
 * Pattern 6 is 00 (1 Byte)
 * Pattern 7 is ff (1 Byte)
 * Patterns 0-7 have a big-endian block number in the last 2 bytes of each 256
 * byte block.
 *
 * Pattern 8 is 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11... (256 B)
 * Pattern 9 is ff fe fd fc fb fa f9 f8 f7 f6 f5 f4 f3 f2 f1 f0 ef ee... (256 B)
 * Pattern 10 is 00 00 00 01 00 02 00 03 00 04... (128 kB big-endian counter)
 * Pattern 11 is ff ff ff fe ff fd ff fc ff fb... (128 kB big-endian downwards)
 * Pattern 12 is 00 (1 Byte)
 * Pattern 13 is ff (1 Byte)
 * Patterns 8-13 have no block number.
 *
 * Patterns 0-3 are created to detect and efficiently diagnose communication
 * slips like missed bits or bytes and their repetitive nature gives good visual
 * cues to the person inspecting the results. In addition, the following holds:
 * AND Pattern 0/1 == Pattern 4
 * AND Pattern 2/3 == Pattern 5
 * AND Pattern 0/1/2/3 == AND Pattern 4/5 == Pattern 6
 * A weakness of pattern 0-5 is the inability to detect swaps/copies between
 * any two 16-byte blocks except for the last 16-byte block in a 256-byte bloc.
 * They work perfectly for detecting any swaps/aliasing of blocks >= 256 bytes.
 * 0x5 and 0xa were picked because they are 0101 and 1010 binary.
 * Patterns 8-9 are best for detecting swaps/aliasing of blocks < 256 bytes.
 * Besides that, they provide for bit testing of the last two bytes of every
 * 256 byte block which contains the block number for patterns 0-6.
 * Patterns 10-11 are special purpose for detecting subblock aliasing with
 * block sizes >256 bytes (some Dataflash chips etc.)
 * AND Pattern 8/9 == Pattern 12
 * AND Pattern 10/11 == Pattern 12
 * Pattern 13 is the completely erased state.
 * None of the patterns can detect aliasing at boundaries which are a multiple
 * of 16 MBytes (but such chips do not exist anyway for Parallel/LPC/FWH/SPI).
 */
int generate_testpattern(uint8_t *buf, uint32_t size, int variant)
{
	int i;

	if (!buf) {
		msg_gerr("Invalid buffer!\n");
		return 1;
	}

	switch (variant) {
	case 0:
		for (i = 0; i < size; i++)
			buf[i] = (i & 0xf) << 4 | 0x5;
		break;
	case 1:
		for (i = 0; i < size; i++)
			buf[i] = (i & 0xf) << 4 | 0xa;
		break;
	case 2:
		for (i = 0; i < size; i++)
			buf[i] = 0x50 | (i & 0xf);
		break;
	case 3:
		for (i = 0; i < size; i++)
			buf[i] = 0xa0 | (i & 0xf);
		break;
	case 4:
		for (i = 0; i < size; i++)
			buf[i] = (i & 0xf) << 4;
		break;
	case 5:
		for (i = 0; i < size; i++)
			buf[i] = i & 0xf;
		break;
	case 6:
		memset(buf, 0x00, size);
		break;
	case 7:
		memset(buf, 0xff, size);
		break;
	case 8:
		for (i = 0; i < size; i++)
			buf[i] = i & 0xff;
		break;
	case 9:
		for (i = 0; i < size; i++)
			buf[i] = ~(i & 0xff);
		break;
	case 10:
		for (i = 0; i < size % 2; i++) {
			buf[i * 2] = (i >> 8) & 0xff;
			buf[i * 2 + 1] = i & 0xff;
		}
		if (size & 0x1)
			buf[i * 2] = (i >> 8) & 0xff;
		break;
	case 11:
		for (i = 0; i < size % 2; i++) {
			buf[i * 2] = ~((i >> 8) & 0xff);
			buf[i * 2 + 1] = ~(i & 0xff);
		}
		if (size & 0x1)
			buf[i * 2] = ~((i >> 8) & 0xff);
		break;
	case 12:
		memset(buf, 0x00, size);
		break;
	case 13:
		memset(buf, 0xff, size);
		break;
	}

	if ((variant >= 0) && (variant <= 7)) {
		/* Write block number in the last two bytes of each 256-byte
		 * block, big endian for easier reading of the hexdump.
		 * Note that this wraps around for chips larger than 2^24 bytes
		 * (16 MB).
		 */
		for (i = 0; i < size / 256; i++) {
			buf[i * 256 + 254] = (i >> 8) & 0xff;
			buf[i * 256 + 255] = i & 0xff;
		}
	}

	return 0;
}

int check_max_decode(enum chipbustype buses, uint32_t size)
{
	int limitexceeded = 0;

	if ((buses & BUS_PARALLEL) && (max_rom_decode.parallel < size)) {
		limitexceeded++;
		msg_pdbg("Chip size %u kB is bigger than supported "
			 "size %u kB of chipset/board/programmer "
			 "for %s interface, "
			 "probe/read/erase/write may fail. ", size / 1024,
			 max_rom_decode.parallel / 1024, "Parallel");
	}
	if ((buses & BUS_LPC) && (max_rom_decode.lpc < size)) {
		limitexceeded++;
		msg_pdbg("Chip size %u kB is bigger than supported "
			 "size %u kB of chipset/board/programmer "
			 "for %s interface, "
			 "probe/read/erase/write may fail. ", size / 1024,
			 max_rom_decode.lpc / 1024, "LPC");
	}
	if ((buses & BUS_FWH) && (max_rom_decode.fwh < size)) {
		limitexceeded++;
		msg_pdbg("Chip size %u kB is bigger than supported "
			 "size %u kB of chipset/board/programmer "
			 "for %s interface, "
			 "probe/read/erase/write may fail. ", size / 1024,
			 max_rom_decode.fwh / 1024, "FWH");
	}
	if ((buses & BUS_SPI) && (max_rom_decode.spi < size)) {
		limitexceeded++;
		msg_pdbg("Chip size %u kB is bigger than supported "
			 "size %u kB of chipset/board/programmer "
			 "for %s interface, "
			 "probe/read/erase/write may fail. ", size / 1024,
			 max_rom_decode.spi / 1024, "SPI");
	}
	if (!limitexceeded)
		return 0;
	/* Sometimes chip and programmer have more than one bus in common,
	 * and the limit is not exceeded on all buses. Tell the user.
	 */
	if (bitcount(buses) > limitexceeded)
		/* FIXME: This message is designed towards CLI users. */
		msg_pdbg("There is at least one common chip/programmer "
			 "interface which can support a chip of this size. "
			 "You can try --force at your own risk.\n");
	return 1;
}

int probe_flash(int startchip, struct flashctx *fill_flash, int force)
{
	const struct flashchip *flash, *flash_list;
	unsigned long base = 0;
	char location[64];
	uint32_t size;
	enum chipbustype buses_common;
	char *tmp;

	/* Based on the host controller interface that a platform
	 * needs to use (hwseq or swseq),
	 * set the flashchips list here.
	 */
	switch (ich_generation) {
	case CHIPSET_100_SERIES_SUNRISE_POINT:
		flash_list = flashchips_hwseq;
		break;
	default:
		flash_list = flashchips;
		break;
	}

	for (flash = flash_list + startchip; flash && flash->name; flash++) {
		if (chip_to_probe && strcmp(flash->name, chip_to_probe) != 0)
			continue;
		buses_common = buses_supported & flash->bustype;
		if (!buses_common) {
			msg_gspew("Probing for %s %s, %d kB: skipped. ",
			         flash->vendor, flash->name, flash->total_size);
			tmp = flashbuses_to_text(buses_supported);
			msg_gspew("Host bus type %s ", tmp);
			free(tmp);
			tmp = flashbuses_to_text(flash->bustype);
			msg_gspew("and chip bus type %s are incompatible.",
				  tmp);
			free(tmp);
			msg_gspew("\n");
			continue;
		}
		msg_gdbg("Probing for %s %s, %d kB: ",
			     flash->vendor, flash->name, flash->total_size);
		if (!flash->probe && !force) {
			msg_gdbg("failed! flashrom has no probe function for "
				 "this flash chip.\n");
			continue;
		}

		size = flash->total_size * 1024;
		check_max_decode(buses_common, size);

		/* Start filling in the dynamic data. */
		memcpy(fill_flash, flash, sizeof(struct flashchip));

		base = flashbase ? flashbase : (0xffffffff - size + 1);
		fill_flash->virtual_memory = (chipaddr)programmer_map_flash_region("flash chip", base, size);

		if (force)
			break;

		if (fill_flash->probe(fill_flash) != 1)
			goto notfound;

		/* If this is the first chip found, accept it.
		 * If this is not the first chip found, accept it only if it is
		 * a non-generic match.
		 * We could either make chipcount global or provide it as
		 * parameter, or we assume that startchip==0 means this call to
		 * probe_flash() is the first one and thus no chip has been
		 * found before.
		 */
		if (startchip == 0 || fill_flash->model_id != GENERIC_DEVICE_ID)
			break;

notfound:
		programmer_unmap_flash_region((void *)fill_flash->virtual_memory, size);
	}

	if (!flash || !flash->name)
		return -1;

#if CONFIG_INTERNAL == 1
	if (programmer_table[programmer].map_flash_region == physmap)
		snprintf(location, sizeof(location), "at physical address 0x%lx", base);
	else
#endif
		snprintf(location, sizeof(location), "on %s", programmer_table[programmer].name);

	tmp = flashbuses_to_text(flash->bustype);
	msg_cdbg("%s %s flash chip \"%s\" (%d kB, %s) %s.\n",
		 force ? "Assuming" : "Found", fill_flash->vendor,
		 fill_flash->name, fill_flash->total_size, tmp, location);
	free(tmp);

	/* Allow the programmer to massage/override chip functionality if
	 * necessary
	 */
	if (programmer_table[programmer].probe_chip) {
		int rc = programmer_table[programmer].probe_chip(fill_flash);
		if (rc < 0)
			return rc;
	}

	/* Flash registers will not be mapped if the chip was forced. Lock info
	 * may be stored in registers, so avoid lock info printing.
	 */
	if (!force)
		if (fill_flash->printlock)
			fill_flash->printlock(fill_flash);

	/* Return position of matching chip. */
	return flash - flash_list;
}

int verify_flash(struct flashctx *flash, uint8_t *buf, int verify_it)
{
	int ret;
	unsigned int total_size = flash->total_size * 1024;

	msg_cinfo("Verifying flash... ");

	if (specified_partition() && verify_it == VERIFY_PARTIAL) {
		ret = handle_partial_verify(flash, buf, verify_range);
	} else {
		ret = verify_range(flash, buf, 0, total_size, NULL);
	}

	if (ret == ACCESS_DENIED) {
		msg_gdbg("Could not fully verify due to access error, ");
		if (access_denied_action == error_ignore) {
			msg_gdbg("ignoring\n");
			ret = 0;
		} else {
			msg_gdbg("aborting\n");
		}
	}

	if (!ret)
		msg_cinfo("VERIFIED.          \n");

	return ret;
}

int read_buf_from_file(unsigned char *buf, unsigned long size,
		       const char *filename)
{
	unsigned long numbytes;
	FILE *image;
	struct stat image_stat;

	if (!strncmp(filename, "-", sizeof("-")))
		image = fdopen(STDIN_FILENO, "rb");
	else
		image = fopen(filename, "rb");
	if (image == NULL) {
		perror(filename);
		return 1;
	}
	if (fstat(fileno(image), &image_stat) != 0) {
		perror(filename);
		fclose(image);
		return 1;
	}
	if ((image_stat.st_size != size) &&
	    (strncmp(filename, "-", sizeof("-")))) {
		msg_gerr("Error: Image size doesn't match: stat %ld bytes, "
			 "wanted %ld!\n", image_stat.st_size, size);
		fclose(image);
		return 1;
	}
	numbytes = fread(buf, 1, size, image);
	if (fclose(image)) {
		perror(filename);
		return 1;
	}
	if (numbytes != size) {
		msg_gerr("Error: Failed to read complete file. Got %ld bytes, "
			 "wanted %ld!\n", numbytes, size);
		return 1;
	}
	return 0;
}

int write_buf_to_file(unsigned char *buf, unsigned long size,
		      const char *filename)
{
	unsigned long numbytes;
	FILE *image;

	if (!filename) {
		msg_gerr("No filename specified.\n");
		return 1;
	}
	if (!strncmp(filename, "-", sizeof("-")))
		image = fdopen(STDOUT_FILENO, "wb");
	else
		image = fopen(filename, "wb");
	if (image == NULL) {
		perror(filename);
		return 1;
	}

	numbytes = fwrite(buf, 1, size, image);
	fclose(image);
	if (numbytes != size) {
		msg_gerr("File %s could not be written completely.\n",
			 filename);
		return 1;
	}
	return 0;
}

/*
 * read_flash - wrapper for flash->read() with additional high-level policy
 *
 * @flash	flash chip
 * @buf		buffer to store data in
 * @start	start address
 * @len		number of bytes to read
 *
 * This wrapper simplifies most cases when the flash chip needs to be read
 * since policy decisions such as non-fatal error handling is centralized.
 */
int read_flash(struct flashctx *flash, uint8_t *buf,
		unsigned int start, unsigned int len)
{
	int ret;

	if (!flash || !flash->read)
		return -1;

	ret = flash->read(flash, buf, start, len);
	if (ret) {
		if (ignore_error(ret)) {
			msg_gdbg("ignoring error when reading 0x%x-0x%x\n",
					start, start + len - 1);
			ret = 0;
		} else {
			msg_gdbg("failed to read 0x%x-0x%x\n",
					start, start + len - 1);
		}
	}

	return ret;
}

/*
 * write_flash - wrapper for flash->write() with additional high-level policy
 *
 * @flash	flash chip
 * @buf		buffer to write to flash
 * @start	start address in flash
 * @len		number of bytes to write
 *
 * TODO: Look up regions that are write-protected and avoid attempt to write
 * to them at all.
 */
int write_flash(struct flashctx *flash, uint8_t *buf,
		unsigned int start, unsigned int len)
{
	if (!flash || !flash->write)
		return -1;

	return flash->write(flash, buf, start, len);
}

int read_flash_to_file(struct flashctx *flash, const char *filename)
{
	unsigned long size = flash->total_size * 1024;
	unsigned char *buf = calloc(size, sizeof(char));
	int ret = 0;

	msg_cinfo("Reading flash... ");
	if (!buf) {
		msg_gerr("Memory allocation failed!\n");
		msg_cinfo("FAILED.\n");
		return 1;
	}

	/* To support partial read, fill buffer to all 0xFF at beginning to make
	 * debug easier. */
	memset(buf, flash_erase_value(flash), size);

	if (!flash->read) {
		msg_cerr("No read function available for this flash chip.\n");
		ret = 1;
		goto out_free;
	}

	/* First try to handle partial read case, rather than read the whole
	 * flash, which is slow. */
	ret = handle_partial_read(flash, buf, read_flash, 1);
	if (ret < 0) {
		msg_cerr("Partial read operation failed!\n");
		ret = 1;
		goto out_free;
	} else if (ret > 0) {
		int num_regions = get_num_include_args();

		if (ret != num_regions) {
			msg_cerr("Requested %d regions, but only read %d\n",
					num_regions, ret);
			ret = 1;
			goto out_free;
		}

		ret = 0;
	} else {
		if (read_flash(flash, buf, 0, size)) {
			msg_cerr("Read operation failed!\n");
			ret = 1;
			goto out_free;
		}
	}

	if (filename)
		ret = write_buf_to_file(buf, size, filename);

out_free:
	free(buf);
	if (ret)
		msg_cerr("FAILED.");
	else
		msg_cdbg("done.");
	return ret;
}

/* This function shares a lot of its structure with erase_and_write_flash() and
 * walk_eraseregions().
 * Even if an error is found, the function will keep going and check the rest.
 */
static int selfcheck_eraseblocks(const struct flashchip *flash)
{
	int i, j, k;
	int ret = 0;

	for (k = 0; k < NUM_ERASEFUNCTIONS; k++) {
		unsigned int done = 0;
		struct block_eraser eraser = flash->block_erasers[k];

		for (i = 0; i < NUM_ERASEREGIONS; i++) {
			/* Blocks with zero size are bugs in flashchips.c. */
			if (eraser.eraseblocks[i].count &&
			    !eraser.eraseblocks[i].size) {
				msg_gerr("ERROR: Flash chip %s erase function "
					"%i region %i has size 0. Please report"
					" a bug at flashrom@flashrom.org\n",
					flash->name, k, i);
				ret = 1;
			}
			/* Blocks with zero count are bugs in flashchips.c. */
			if (!eraser.eraseblocks[i].count &&
			    eraser.eraseblocks[i].size) {
				msg_gerr("ERROR: Flash chip %s erase function "
					"%i region %i has count 0. Please report"
					" a bug at flashrom@flashrom.org\n",
					flash->name, k, i);
				ret = 1;
			}
			done += eraser.eraseblocks[i].count *
				eraser.eraseblocks[i].size;
		}
		/* Empty eraseblock definition with erase function.  */
		if (!done && eraser.block_erase)
			msg_gspew("Strange: Empty eraseblock definition with "
				  "non-empty erase function. Not an error.\n");
		if (!done)
			continue;
		if (done != flash->total_size * 1024) {
			msg_gerr("ERROR: Flash chip %s erase function %i "
				"region walking resulted in 0x%06x bytes total,"
				" expected 0x%06x bytes. Please report a bug at"
				" flashrom@flashrom.org\n", flash->name, k,
				done, flash->total_size * 1024);
			ret = 1;
		}
		if (!eraser.block_erase)
			continue;
		/* Check if there are identical erase functions for different
		 * layouts. That would imply "magic" erase functions. The
		 * easiest way to check this is with function pointers.
		 */
		for (j = k + 1; j < NUM_ERASEFUNCTIONS; j++) {
			if (eraser.block_erase ==
			    flash->block_erasers[j].block_erase) {
				msg_gerr("ERROR: Flash chip %s erase function "
					"%i and %i are identical. Please report"
					" a bug at flashrom@flashrom.org\n",
					flash->name, k, j);
				ret = 1;
			}
		}
	}
	return ret;
}

static int erase_and_write_block_helper(struct flashctx *flash,
					unsigned int start, unsigned int len,
					uint8_t *curcontents,
					uint8_t *newcontents,
					int (*erasefn) (struct flashctx *flash,
							unsigned int addr,
							unsigned int len))
{
	unsigned int starthere = 0, lenhere = 0;
	int ret = 0, skip = 1, writecount = 0;
	int block_was_erased = 0;
	enum write_granularity gran = write_gran_256bytes; /* FIXME */

	/* curcontents and newcontents are opaque to walk_eraseregions, and
	 * need to be adjusted here to keep the impression of proper abstraction
	 */
	curcontents += start;
	newcontents += start;
	msg_cdbg(":");
	/* FIXME: Assume 256 byte granularity for now to play it safe. */
	if (need_erase(flash, curcontents, newcontents, len, gran)) {
		content_has_changed |= 1;
		msg_cdbg("E");
		ret = erasefn(flash, start, len);
		if (ret) {
			if (ret == ACCESS_DENIED)
				msg_cdbg("D");
			else
				msg_cerr("ERASE FAILED!\n");
			return ret;
		}

		if (programmer_table[programmer].paranoid) {
			if (check_erased_range(flash, start, len)) {
				msg_cerr("ERASE FAILED!\n");
				return -1;
			}
		}

		/* Erase was successful. Adjust curcontents. */
		memset(curcontents, flash_erase_value(flash), len);
		skip = 0;
		block_was_erased = 1;
	}
	/* get_next_write() sets starthere to a new value after the call. */
	while ((lenhere = get_next_write(curcontents + starthere,
					 newcontents + starthere,
					 len - starthere, &starthere, gran))) {
		content_has_changed |= 1;
		if (!writecount++)
			msg_cdbg("W");
		/* Needs the partial write function signature. */
		ret = write_flash(flash, newcontents + starthere,
				   start + starthere, lenhere);
		if (ret) {
			if (ret == ACCESS_DENIED)
				msg_cdbg("D");
			return ret;
		}

		/*
		 * If the block needed to be erased and was erased successfully
		 * then we can assume that we didn't run into any write-
		 * protected areas. Otherwise, we need to verify each page to
		 * ensure it was successfully written and abort if we encounter
		 * any errors.
		 */
		if (programmer_table[programmer].paranoid && !block_was_erased) {
			if (verify_range(flash, newcontents + starthere,
					start + starthere, lenhere, "WRITE"))
				return -1;
		}

		starthere += lenhere;
		skip = 0;
	}
	if (skip)
		msg_cdbg("S");
	return ret;
}

static int walk_eraseregions(struct flashctx *flash, int erasefunction,
			     int (*do_something) (struct flashctx *flash,
						  unsigned int addr,
						  unsigned int len,
						  uint8_t *param1,
						  uint8_t *param2,
						  int (*erasefn) (
							struct flashctx *flash,
							unsigned int addr,
							unsigned int len)),
			     void *param1, void *param2)
{
	int i, j, rc = -1;
	unsigned int start = 0;
	unsigned int len;
	struct block_eraser eraser = flash->block_erasers[erasefunction];

	for (i = 0; i < NUM_ERASEREGIONS; i++) {
		/* count==0 and size==0 for all automatically initialized array
		 * members so the loop below won't be executed for them.
		 */
		len = eraser.eraseblocks[i].size;
		if (!len)
			continue;

		if (required_erase_size && (len != required_erase_size)) {
			msg_cdbg("%u does not meet erase alignment", len);
			rc = -1;
			break;
		}

		for (j = 0; j < eraser.eraseblocks[i].count; j++) {
			/* Print this for every block except the first one. */
			if (i || j)
				msg_cdbg(", ");
			msg_cdbg("0x%06x-0x%06x", start,
				     start + len - 1);
			rc = do_something(flash, start, len, param1, param2,
			                  eraser.block_erase);
			if (rc) {
				if (ignore_error(rc))
					rc = 0;
				else
					return rc;
			}
			start += len;
		}
	}
	msg_cdbg("\n");
	return rc;
}

static int check_block_eraser(const struct flashctx *flash, int k, int log)
{
	struct block_eraser eraser = flash->block_erasers[k];

	if (!eraser.block_erase && !eraser.eraseblocks[0].count) {
		if (log)
			msg_cdbg("not defined. ");
		return 1;
	}
	if (!eraser.block_erase && eraser.eraseblocks[0].count) {
		if (log)
			msg_cdbg("eraseblock layout is known, but matching "
				 "block erase function is not implemented. ");
		return 1;
	}
	if (eraser.block_erase && !eraser.eraseblocks[0].count) {
		if (log)
			msg_cdbg("block erase function found, but "
				 "eraseblock layout is not defined. ");
		return 1;
	}
	return 0;
}

int erase_and_write_flash(struct flashctx *flash, uint8_t *oldcontents,
			  uint8_t *newcontents)
{
	int k, ret = 1;
	uint8_t *curcontents;
	unsigned long size = flash->total_size * 1024;
	unsigned int usable_erasefunctions = count_usable_erasers(flash);

	msg_cinfo("Erasing and writing flash chip... ");
	curcontents = malloc(size);
	if (!curcontents) {
		msg_gerr("Out of memory!\n");
		exit(1);
	}
	/* Copy oldcontents to curcontents to avoid clobbering oldcontents. */
	memcpy(curcontents, oldcontents, size);

	for (k = 0; k < NUM_ERASEFUNCTIONS; k++) {
		if (k != 0)
			msg_cdbg("Looking for another erase function.\n");
		if (!usable_erasefunctions) {
			msg_cdbg("No usable erase functions left.\n");
			break;
		}
		msg_cdbg("Trying erase function %i... ", k);
		if (check_block_eraser(flash, k, 1))
			continue;
		usable_erasefunctions--;
		ret = walk_eraseregions(flash, k, &erase_and_write_block_helper,
					curcontents, newcontents);
		/* If everything is OK, don't try another erase function. */
		if (!ret)
			break;
		/* Write/erase failed, so try to find out what the current chip
		 * contents are. If no usable erase functions remain, we can
		 * skip this: the next iteration will break immediately anyway.
		 */
		if (!usable_erasefunctions)
			continue;
		/* Reading the whole chip may take a while, inform the user even
		 * in non-verbose mode.
		 */
		msg_cinfo("Reading current flash chip contents... ");
		if (read_flash(flash, curcontents, 0, size)) {
			/* Now we are truly screwed. Read failed as well. */
			msg_cerr("Can't read anymore! Aborting.\n");
			/* We have no idea about the flash chip contents, so
			 * retrying with another erase function is pointless.
			 */
			break;
		}
		msg_cdbg("done. ");
	}
	/* Free the scratchpad. */
	free(curcontents);

	if (ret) {
		msg_cerr("FAILED!\n");
	} else {
		msg_cdbg("SUCCESS.\n");
	}
	return ret;
}

void nonfatal_help_message(void)
{
	msg_gerr("Writing to the flash chip apparently didn't do anything.\n"
		"This means we have to add special support for your board, "
		  "programmer or flash chip.\n"
		"Please report this on IRC at irc.freenode.net (channel "
		  "#flashrom) or\n"
		"mail flashrom@flashrom.org!\n"
		"-------------------------------------------------------------"
		  "------------------\n"
		"You may now reboot or simply leave the machine running.\n");
}

void emergency_help_message(void)
{
	msg_gerr("Your flash chip is in an unknown state.\n"
		"Get help on IRC at irc.freenode.net (channel #flashrom) or\n"
		"mail flashrom@flashrom.org with FAILED: your board name in "
		  "the subject line!\n"
		"-------------------------------------------------------------"
		  "------------------\n"
		"DO NOT REBOOT OR POWEROFF!\n");
}

/* The way to go if you want a delimited list of programmers */
void list_programmers(const char *delim)
{
	enum programmer p;
	for (p = 0; p < PROGRAMMER_INVALID; p++) {
		msg_ginfo("%s", programmer_table[p].name);
		if (p < PROGRAMMER_INVALID - 1)
			msg_ginfo("%s", delim);
	}
	msg_ginfo("\n");
}

void list_programmers_linebreak(int startcol, int cols, int paren)
{
	const char *pname;
	int pnamelen;
	int remaining = 0, firstline = 1;
	enum programmer p;
	int i;

	for (p = 0; p < PROGRAMMER_INVALID; p++) {
		pname = programmer_table[p].name;
		pnamelen = strlen(pname);
		if (remaining - pnamelen - 2 < 0) {
			if (firstline)
				firstline = 0;
			else
				printf("\n");
			for (i = 0; i < startcol; i++)
				printf(" ");
			remaining = cols - startcol;
		} else {
			printf(" ");
			remaining--;
		}
		if (paren && (p == 0)) {
			printf("(");
			remaining--;
		}
		printf("%s", pname);
		remaining -= pnamelen;
		if (p < PROGRAMMER_INVALID - 1) {
			printf(",");
			remaining--;
		} else {
			if (paren)
				printf(")");
			printf("\n");
		}
	}
}

void print_sysinfo(void)
{
	/* send to stderr for chromium os */
#if HAVE_UTSNAME == 1
	struct utsname osinfo;
	uname(&osinfo);

	msg_gerr(" on %s %s (%s)", osinfo.sysname, osinfo.release,
		  osinfo.machine);
#else
	msg_gerr(" on unknown machine");
#endif
	msg_gerr(", built with");
#if NEED_PCI == 1
#ifdef PCILIB_VERSION
	msg_gerr(" libpci %s,", PCILIB_VERSION);
#else
	msg_gerr(" unknown PCI library,");
#endif
#endif
#ifdef __clang__
	msg_gerr(" LLVM Clang");
#ifdef __clang_version__
	msg_gerr(" %s,", __clang_version__);
#else
	msg_gerr(" unknown version (before r102686),");
#endif
#elif defined(__GNUC__)
	msg_gerr(" GCC");
#ifdef __VERSION__
	msg_gerr(" %s,", __VERSION__);
#else
	msg_gerr(" unknown version,");
#endif
#else
	msg_gerr(" unknown compiler,");
#endif
#if defined (__FLASHROM_LITTLE_ENDIAN__)
	msg_gerr(" little endian");
#else
	msg_gerr(" big endian");
#endif
	msg_gerr("\n");
}

void print_version(void)
{
	/* send to stderr for chromium os */
	msg_gerr("flashrom v%s", flashrom_version);
	print_sysinfo();
}

void print_banner(void)
{
	msg_ginfo("flashrom is free software, get the source code at "
		  "http://www.flashrom.org\n");
	msg_ginfo("\n");
}

int selfcheck(void)
{
	int ret = 0;
	const struct flashchip *flash;

	/* Safety check. Instead of aborting after the first error, check
	 * if more errors exist.
	 */
	if (ARRAY_SIZE(programmer_table) - 1 != PROGRAMMER_INVALID) {
		msg_gerr("Programmer table miscompilation!\n");
		ret = 1;
	}
	/* It would be favorable if we could also check for correct termination
	 * of the following arrays, but we don't know their sizes in here...
	 * For 'flashchips' we check the first element to be non-null. In the
	 * other cases there exist use cases where the first element can be
	 * null. */
	if (flashchips[0].vendor == NULL) {
		msg_gerr("Flashchips table miscompilation!\n");
		ret = 1;
	}
	/* Check that virtual_memory in struct flashctx is placed directly
	 * after the members copied from struct flashchip.
	 */
	if (sizeof(struct flashchip) != offsetof(struct flashctx, virtual_memory)) {
		msg_gerr("struct flashctx broken!\n");
		ret = 1;
	}
	for (flash = flashchips; flash && flash->name; flash++)
		if (selfcheck_eraseblocks(flash))
			ret = 1;

	return ret;
}

void check_chip_supported(const struct flashctx *flash)
{
	if (TEST_OK_MASK != (flash->tested & TEST_OK_MASK)) {
		msg_cdbg("===\n");
		if (flash->tested & TEST_BAD_MASK) {
			msg_cdbg("This flash part has status NOT WORKING for operations:");
			if (flash->tested & TEST_BAD_PROBE)
				msg_cdbg(" PROBE");
			if (flash->tested & TEST_BAD_READ)
				msg_cdbg(" READ");
			if (flash->tested & TEST_BAD_ERASE)
				msg_cdbg(" ERASE");
			if (flash->tested & TEST_BAD_WRITE)
				msg_cdbg(" WRITE");
			if (flash->tested & TEST_BAD_UREAD)
				msg_cdbg(" UNBOUNDED READ");
			msg_cdbg("\n");
		}
		if ((!(flash->tested & TEST_BAD_PROBE) && !(flash->tested & TEST_OK_PROBE)) ||
		    (!(flash->tested & TEST_BAD_READ) && !(flash->tested & TEST_OK_READ)) ||
		    (!(flash->tested & TEST_BAD_ERASE) && !(flash->tested & TEST_OK_ERASE)) ||
		    (!(flash->tested & TEST_BAD_WRITE) && !(flash->tested & TEST_OK_WRITE))) {
			msg_cdbg("This flash part has status UNTESTED for operations:");
			if (!(flash->tested & TEST_BAD_PROBE) && !(flash->tested & TEST_OK_PROBE))
				msg_cdbg(" PROBE");
			if (!(flash->tested & TEST_BAD_READ) && !(flash->tested & TEST_OK_READ))
				msg_cdbg(" READ");
			if (!(flash->tested & TEST_BAD_ERASE) && !(flash->tested & TEST_OK_ERASE))
				msg_cdbg(" ERASE");
			if (!(flash->tested & TEST_BAD_WRITE) && !(flash->tested & TEST_OK_WRITE))
				msg_cdbg(" WRITE");
			if (!(flash->tested & TEST_BAD_UREAD) && !(flash->tested & TEST_OK_UREAD))
				msg_cdbg(" UNBOUNDED READ");
			msg_cdbg("\n");
		}
		/* FIXME: This message is designed towards CLI users. */
		msg_cdbg("The test status of this chip may have been updated "
			    "in the latest development\n"
			  "version of flashrom. If you are running the latest "
			    "development version,\n"
			  "please email a report to flashrom@flashrom.org if "
			    "any of the above operations\n"
			  "work correctly for you with this flash part. Please "
			    "include the flashrom\n"
			  "output with the additional -V option for all "
			    "operations you tested (-V, -Vr,\n"
			  "-Vw, -VE), and mention which mainboard or "
			    "programmer you tested.\n"
			  "Please mention your board in the subject line. "
			    "Thanks for your help!\n");
	}
}

/* FIXME: This function signature needs to be improved once doit() has a better
 * function signature.
 */
int chip_safety_check(struct flashctx *flash, int force, int read_it, int write_it, int erase_it, int verify_it)
{
	if (!programmer_may_write && (write_it || erase_it)) {
		msg_perr("Write/erase is not working yet on your programmer in "
			 "its current configuration.\n");
		/* --force is the wrong approach, but it's the best we can do
		 * until the generic programmer parameter parser is merged.
		 */
		if (!force)
			return 1;
		msg_cerr("Continuing anyway.\n");
	}

	if (read_it || erase_it || write_it || verify_it) {
		/* Everything needs read. */
		if (flash->tested & TEST_BAD_READ) {
			msg_cerr("Read is not working on this chip. ");
			if (!force)
				return 1;
			msg_cerr("Continuing anyway.\n");
		}
		if (!flash->read) {
			msg_cerr("flashrom has no read function for this "
				 "flash chip.\n");
			return 1;
		}
	}
	if (erase_it || write_it) {
		/* Write needs erase. */
		if (flash->tested & TEST_BAD_ERASE) {
			msg_cerr("Erase is not working on this chip. ");
			if (!force)
				return 1;
			msg_cerr("Continuing anyway.\n");
		}
		if(count_usable_erasers(flash) == 0) {
			msg_cerr("flashrom has no erase function for this "
				 "flash chip.\n");
			return 1;
		}
	}
	if (write_it) {
		if (flash->tested & TEST_BAD_WRITE) {
			msg_cerr("Write is not working on this chip. ");
			if (!force)
				return 1;
			msg_cerr("Continuing anyway.\n");
		}
		if (!flash->write) {
			msg_cerr("flashrom has no write function for this "
				 "flash chip.\n");
			return 1;
		}
	}
	return 0;
}

/* This function signature is horrible. We need to design a better interface,
 * but right now it allows us to split off the CLI code.
 * Besides that, the function itself is a textbook example of abysmal code flow.
 */
int doit(struct flashctx *flash, int force, const char *filename, int read_it,
	 int write_it, int erase_it, int verify_it, int extract_it,
	 const char *diff_file)
{
	uint8_t *oldcontents;
	uint8_t *newcontents;
	int ret = 0;
	unsigned long size = flash->total_size * 1024;

	if (chip_safety_check(flash, force, read_it, write_it, erase_it, verify_it)) {
		msg_cerr("Aborting.\n");
		ret = 1;
		goto out_nofree;
	}

	/* Given the existence of read locks, we want to unlock for read,
	 * erase and write.
	 */
	if (flash->unlock)
		flash->unlock(flash);

	/* add entries for regions specified in flashmap */
	if (!set_ignore_fmap && add_fmap_entries(flash) < 0) {
		ret = 1;
		goto out_nofree;
	}

	if (extract_it) {
		ret = extract_regions(flash);
		goto out_nofree;
	}

	/* mark entries included using -i argument as "included" if they are
	   found in the master rom_entries list */
	if (process_include_args() < 0) {
		ret = 1;
		goto out_nofree;
	}

	if (read_it) {
		ret = read_flash_to_file(flash, filename);
		goto out_nofree;
	}

	oldcontents = malloc(size);
	if (!oldcontents) {
		msg_gerr("Out of memory!\n");
		exit(1);
	}
	/* Assume worst case: All blocks are not erased. */
	memset(oldcontents, flash_unerased_value(flash), size);
	newcontents = malloc(size);
	if (!newcontents) {
		msg_gerr("Out of memory!\n");
		exit(1);
	}
	/* Assume best case: All blocks are erased. */
	memset(newcontents, flash_erase_value(flash), size);
	/* Side effect of the assumptions above: Default write action is erase
	 * because newcontents looks like a completely erased chip, and
	 * oldcontents being completely unerased means we have to erase
	 * everything before we can write.
	 */

	if (write_it || verify_it) {
		/*
		 * Note: This must be done before any files specified by -i
		 * arguments are processed merged into the newcontents since
		 * -i files take priority. See http://crbug.com/263495.
		 */
		if (filename) {
			if (read_buf_from_file(newcontents, size, filename)) {
				ret = 1;
				goto out;
			}
		} else {
			/* Content will be read from -i args, so they must
			 * not overlap. */
			if (included_regions_overlap()) {
				msg_gerr("Error: Included regions must "
						"not overlap.\n");
				ret = 1;
				goto out;
			}
		}

#if 0
		/*
		 * FIXME: show_id() causes failure if vendor:mainboard do not
		 * match. This may happen if codenames are in flux.
		 * See chrome-os-partner:10414.
		 */
#if CONFIG_INTERNAL == 1
		if (programmer == PROGRAMMER_INTERNAL)
			show_id(newcontents, size, force);
#endif
#endif
	}

	/* Obtain a reference image so that we can check whether regions need
	 * to be erased and to give better diagnostics in case write fails.
	 * If --fast-verify is used then only the regions which are included
	 * using -i will be read.
	 */
	if (diff_file) {
		msg_cdbg("Reading old contents from file... ");
		if (read_buf_from_file(oldcontents, size, diff_file)) {
			ret = 1;
			msg_cdbg("FAILED.\n");
			goto out;
		}
	} else {
		msg_cdbg("Reading old contents from flash chip... ");
		if (verify_it == VERIFY_PARTIAL) {
			if (handle_partial_read(flash, oldcontents,
						read_flash, 0) < 0) {
				ret = 1;
				msg_cdbg("FAILED.\n");
				goto out;
			}
		} else {
			if (read_flash(flash, oldcontents, 0, size)) {
				ret = 1;
				msg_cdbg("FAILED.\n");
				goto out;
			}
		}
	}
	msg_cdbg("done.\n");

	/*
	 * Note: This must be done after reading the file specified for the
	 * -w/-v argument, if any, so that files specified using -i end up
	 * in the "newcontents" buffer before being written.
	 * See http://crbug.com/263495.
	 */
	if (handle_romentries(flash, oldcontents, newcontents)) {
		ret = 1;
		msg_cerr("Error handling ROM entries.\n");
		goto out;
	}

	if (erase_it) {
		/* FIXME: Do we really want the scary warning if erase failed?
		 * After all, after erase the chip is either blank or partially
		 * blank or it has the old contents. A blank chip won't boot,
		 * so if the user wanted erase and reboots afterwards, the user
		 * knows very well that booting won't work.
		 */
		if (erase_and_write_flash(flash, oldcontents, newcontents)) {
			emergency_help_message();
			ret = 1;
		}
		goto out;
	}

	if (write_it) {
		// parse the new fmap
		if ((ret = cros_ec_prepare(newcontents, size))) {
			msg_cerr("CROS_EC prepare failed, ret=%d.\n", ret);
			goto out;
		}

		if (erase_and_write_flash(flash, oldcontents, newcontents)) {
			msg_cerr("Uh oh. Erase/write failed. Checking if "
				 "anything changed.\n");
			if (!read_flash(flash, newcontents, 0, size)) {
				if (!memcmp(oldcontents, newcontents, size)) {
					msg_cinfo("Good. It seems nothing was "
						  "changed.\n");
					nonfatal_help_message();
					ret = 1;
					goto out;
				}
			}
			emergency_help_message();
			ret = 1;
			goto out;
		}

		ret = cros_ec_need_2nd_pass();
		if (ret < 0) {
			// Jump failed
			msg_cerr("cros_ec_need_2nd_pass() failed. Stop.\n");
			emergency_help_message();
			ret = 1;
			goto out;
		} else if (ret > 0) {
			// Need 2nd pass. Get the just written content.
			msg_pdbg("CROS_EC needs 2nd pass.\n");
			if (read_flash(flash, oldcontents, 0, size)) {
				msg_cerr("Uh oh. Cannot get latest content.\n");
				emergency_help_message();
				ret = 1;
				goto out;
			}
			// write 2nd pass
			if (erase_and_write_flash(flash, oldcontents,
			                          newcontents)) {
				msg_cerr("Uh oh. CROS_EC 2nd pass failed.\n");
				emergency_help_message();
				ret = 1;
				goto out;
			}
			ret = 0;
		}

		if (cros_ec_finish() < 0) {
			msg_cerr("cros_ec_finish() failed. Stop.\n");
			emergency_help_message();
			ret = 1;
			goto out;
		}
	}

	if (verify_it) {
		if ((write_it || erase_it) && !content_has_changed) {
			msg_gdbg("Nothing was erased or written, skipping "
				"verification\n");
		} else {
			/* Work around chips which need some time to calm down. */
			if (write_it && verify_it != VERIFY_PARTIAL)
				programmer_delay(1000*1000);

			ret = verify_flash(flash, newcontents, verify_it);

			/* If we tried to write, and verification now fails, we
			 * might have an emergency situation.
			 */
			if (ret && write_it)
				emergency_help_message();
		}
	}

out:
	free(oldcontents);
	free(newcontents);
out_nofree:
	chip_restore();	/* must be done before programmer_shutdown() */
	/*
	 * programmer_shutdown() call is moved to cli_mfg() in chromium os
	 * tree. This is because some operations, such as write protection,
	 * requires programmer_shutdown() but does not call doit().
	 */
//	programmer_shutdown();
	return ret;
}
