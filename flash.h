/*
 * This file is part of the flashrom project.
 *
 * Copyright (C) 2000 Silicon Integrated System Corporation
 * Copyright (C) 2000 Ronald G. Minnich <rminnich@gmail.com>
 * Copyright (C) 2005-2009 coresystems GmbH
 * Copyright (C) 2006-2009 Carl-Daniel Hailfinger
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

#ifndef __FLASH_H__
#define __FLASH_H__ 1

#include <stdint.h>
#include <stddef.h>
#include "hwaccess.h"
#ifdef _WIN32
#include <windows.h>
#undef min
#undef max
#endif

/* Are timers broken? */
extern int broken_timer;

struct flashctx; /* forward declare */
#define ERROR_PTR ((void*)-1)

/* Error codes */
#define TIMEOUT_ERROR	-101

/* for verify_it variable in flashrom.c and cli_mfg.c */
enum {
	VERIFY_OFF = 0,
	VERIFY_FULL,
	VERIFY_PARTIAL,
};

/*
 * This is shared between handle_partial_read() and erase_and_write_flash().
 * If a partial write is to be performed, the read function needs to guess what
 * the eraseable block size is in case the region specified is not aligned. Then
 * it can ensure any data within the same block but outside the specified region
 * is read and later restored. The erase/write function will need to find a
 * usable erase function with the same size.
 *
 * Some chips support multiple opcodes to erase a particular block size, so
 * we'll leave that guesswork to erase_and_write_flash(). Any opcode is allowed
 * so long as the alignment used during partial read and erase are the same.
 */
extern unsigned int required_erase_size;

typedef unsigned long chipaddr;

int register_shutdown(int (*function) (void *data), void *data);
#define CHIP_RESTORE_CALLBACK	int (*func) (struct flashctx *flash, uint8_t status)

int register_chip_restore(CHIP_RESTORE_CALLBACK, struct flashctx *flash, uint8_t status);
void *programmer_map_flash_region(const char *descr, unsigned long phys_addr,
				  size_t len);
void programmer_unmap_flash_region(void *virt_addr, size_t len);
void programmer_delay(int usecs);

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

enum chipbustype {
	BUS_NONE	= 0,
	BUS_PARALLEL	= 1 << 0,
	BUS_LPC		= 1 << 1,
	BUS_FWH		= 1 << 2,
	BUS_SPI		= 1 << 3,
	BUS_PROG	= 1 << 4,
	BUS_NONSPI	= BUS_PARALLEL | BUS_LPC | BUS_FWH,
};

/* used to select bus which target chip resides */
extern enum chipbustype target_bus;

/*
 * How many different contiguous runs of erase blocks with one size each do
 * we have for a given erase function?
 */
#define NUM_ERASEREGIONS 5

/*
 * How many different erase functions do we have per chip?
 * Atmel AT25FS010 has 6 different functions.
 */
#define NUM_ERASEFUNCTIONS 6

#define FEATURE_REGISTERMAP	(1 << 0)
#define FEATURE_BYTEWRITES	(1 << 1)
#define FEATURE_LONG_RESET	(0 << 4)
#define FEATURE_SHORT_RESET	(1 << 4)
#define FEATURE_EITHER_RESET	FEATURE_LONG_RESET
#define FEATURE_RESET_MASK	(FEATURE_LONG_RESET | FEATURE_SHORT_RESET)
#define FEATURE_ADDR_FULL	(0 << 2)
#define FEATURE_ADDR_MASK	(3 << 2)
#define FEATURE_ADDR_2AA	(1 << 2)
#define FEATURE_ADDR_AAA	(2 << 2)
#define FEATURE_ADDR_SHIFTED	(1 << 5)
#define FEATURE_WRSR_EWSR	(1 << 6)
#define FEATURE_WRSR_WREN	(1 << 7)
#define FEATURE_WRSR_EITHER	(FEATURE_WRSR_EWSR | FEATURE_WRSR_WREN)

/* Erasing flash produces zeroes rather than ones */
#define FEATURE_ERASE_TO_ZERO	(1 << 8)
#define FEATURE_UNBOUND_READ	(1 << 10)

struct voltage_range {
	uint16_t min, max;
};

struct flashchip {
	const char *vendor;
	const char *name;

	enum chipbustype bustype;

	/*
	 * With 32bit manufacture_id and model_id we can cover IDs up to
	 * (including) the 4th bank of JEDEC JEP106W Standard Manufacturer's
	 * Identification code.
	 */
	uint32_t manufacture_id;
	uint32_t model_id;

	/* Total chip size in kilobytes */
	unsigned int total_size;
	/* Chip page size in bytes */
	unsigned int page_size;
	int feature_bits;

	/*
	 * Indicate if flashrom has been tested with this flash chip and if
	 * everything worked correctly.
	 */
	uint32_t tested;

	int (*probe) (struct flashctx *flash);

	/* Delay after "enter/exit ID mode" commands in microseconds.
	 * NB: negative values have special meanings, see TIMING_* below.
	 */
	signed int probe_timing;

	/*
	 * Erase blocks and associated erase function. Any chip erase function
	 * is stored as chip-sized virtual block together with said function.
	 * The first one that fits will be chosen. There is currently no way to
	 * influence that behaviour. For testing just comment out the other
	 * elements or set the function pointer to NULL.
	 */
	struct block_eraser {
		struct eraseblock{
			unsigned int size; /* Eraseblock size in bytes */
			unsigned int count; /* Number of contiguous blocks with that size */
		} eraseblocks[NUM_ERASEREGIONS];
		/* a block_erase function should try to erase one block of size
		 * 'blocklen' at address 'blockaddr' and return 0 on success. */
		int (*block_erase) (struct flashctx *flash, unsigned int blockaddr, unsigned int blocklen);
	} block_erasers[NUM_ERASEFUNCTIONS];

	int (*printlock) (struct flashctx *flash);
	int (*unlock) (struct flashctx *flash);
	int (*write) (struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len);
	int (*read) (struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len);
	uint8_t (*read_status) (const struct flashctx *flash);
	int (*write_status) (const struct flashctx *flash, int status);
	struct voltage_range voltage;
	struct wp *wp;
};

/* struct flashctx must always contain struct flashchip at the beginning. */
struct flashctx {
	const char *vendor;
	const char *name;
	enum chipbustype bustype;
	uint32_t manufacture_id;
	uint32_t model_id;
	int total_size;
	int page_size;
	int feature_bits;
	uint32_t tested;
	int (*probe) (struct flashctx *flash);
	int probe_timing;
	struct block_eraser block_erasers[NUM_ERASEFUNCTIONS];
	int (*printlock) (struct flashctx *flash);
	int (*unlock) (struct flashctx *flash);
	int (*write) (struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len);
	int (*read) (struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len);
	uint8_t (*read_status) (const struct flashctx *flash);
	int (*write_status) (const struct flashctx *flash, int status);
	struct voltage_range voltage;
	struct wp *wp;
	/* struct flashchip ends here. */
	chipaddr virtual_memory;
	/* Some flash devices have an additional register space. */
	chipaddr virtual_registers;
};


/* This is the byte value we expect to see in erased regions of the flash */
int flash_erase_value(struct flashctx *flash);

/* This is a byte value that indicates that the region is not erased */
int flash_unerased_value(struct flashctx *flash);

#define TEST_UNTESTED	0

#define TEST_OK_PROBE	(1 << 0)
#define TEST_OK_READ	(1 << 1)
#define TEST_OK_ERASE	(1 << 2)
#define TEST_OK_WRITE	(1 << 3)
#define TEST_OK_UREAD	(1 << 4)
#define TEST_OK_PR	(TEST_OK_PROBE | TEST_OK_READ)
#define TEST_OK_PRE	(TEST_OK_PROBE | TEST_OK_READ | TEST_OK_ERASE)
#define TEST_OK_PREU	(TEST_OK_PROBE | TEST_OK_READ | TEST_OK_ERASE | TEST_OK_UREAD)
#define TEST_OK_PRU	(TEST_OK_PROBE | TEST_OK_READ | TEST_OK_UREAD)
#define TEST_OK_PRW	(TEST_OK_PROBE | TEST_OK_READ | TEST_OK_WRITE)
#define TEST_OK_PREW	(TEST_OK_PROBE | TEST_OK_READ | TEST_OK_ERASE | TEST_OK_WRITE)
#define TEST_OK_PREWU	(TEST_OK_PROBE | TEST_OK_READ | TEST_OK_ERASE | TEST_OK_WRITE | TEST_OK_UREAD)
#define TEST_OK_MASK	0x1f

#define TEST_BAD_PROBE	(1 << 5)
#define TEST_BAD_READ	(1 << 6)
#define TEST_BAD_ERASE	(1 << 7)
#define TEST_BAD_WRITE	(1 << 8)
#define TEST_BAD_UREAD	(1 << 9)
#define TEST_BAD_PREW	(TEST_BAD_PROBE | TEST_BAD_READ | TEST_BAD_ERASE | TEST_BAD_WRITE)
#define TEST_BAD_PREWU	(TEST_BAD_PROBE | TEST_BAD_READ | TEST_BAD_ERASE | TEST_BAD_WRITE | TEST_BAD_UREAD)
#define TEST_BAD_MASK	0x3e0

/* Timing used in probe routines. ZERO is -2 to differentiate between an unset
 * field and zero delay.
 *
 * SPI devices will always have zero delay and ignore this field.
 */
#define TIMING_FIXME	-1
/* this is intentionally same value as fixme */
#define TIMING_IGNORED	-1
#define TIMING_ZERO	-2

extern const struct flashchip flashchips[];
extern const struct flashchip flashchips_hwseq[];

void chip_writeb(const struct flashctx *flash, uint8_t val, chipaddr addr);
void chip_writew(const struct flashctx *flash, uint16_t val, chipaddr addr);
void chip_writel(const struct flashctx *flash, uint32_t val, chipaddr addr);
void chip_writen(const struct flashctx *flash, uint8_t *buf, chipaddr addr, size_t len);
uint8_t chip_readb(const struct flashctx *flash, const chipaddr addr);
uint16_t chip_readw(const struct flashctx *flash, const chipaddr addr);
uint32_t chip_readl(const struct flashctx *flash, const chipaddr addr);
void chip_readn(const struct flashctx *flash, uint8_t *buf, const chipaddr addr, size_t len);

/* print.c */
char *flashbuses_to_text(enum chipbustype bustype);
void print_supported(void);
void print_supported_wiki(void);

/* flashrom.c */
enum write_granularity {
	write_gran_1bit,
	write_gran_1byte,
	write_gran_256bytes,
};
extern enum chipbustype buses_supported;
extern int verbose;
extern const char flashrom_version[];
extern char *chip_to_probe;
void map_flash_registers(struct flashctx *flash);
int read_memmapped(struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len);
int erase_flash(struct flashctx *flash);
int probe_flash(int startchip, struct flashctx *fill_flash, int force);
int read_flash(struct flashctx *flash, uint8_t *buf,
			unsigned int start, unsigned int len);
int read_flash_to_file(struct flashctx *flash, const char *filename);
int min(int a, int b);
int max(int a, int b);
void tolower_string(char *str);
char *extract_param(char **haystack, const char *needle, const char *delim);
int verify_range(struct flashctx *flash, uint8_t *cmpbuf, unsigned int start, unsigned int len, const char *message);
char *strcat_realloc(char *dest, const char *src);
void print_version(void);
void print_banner(void);
void list_programmers_linebreak(int startcol, int cols, int paren);
int selfcheck(void);
int doit(struct flashctx *flash, int force, const char *filename, int read_it,
	 int write_it, int erase_it, int verify_it, int extract_it,
	 const char *diff_file);
int read_buf_from_file(unsigned char *buf, unsigned long size, const char *filename);
int write_buf_to_file(unsigned char *buf, unsigned long size, const char *filename);

#define OK 0
#define NT 1    /* Not tested */

/* what to do in case of an error */
enum error_action {
	error_fail,	/* fail immediately */
	error_ignore,	/* non-fatal error; continue */
};

/* Something happened that shouldn't happen, but we can go on. */
#define ERROR_NONFATAL 0x100

/* Something happened that shouldn't happen, we'll abort. */
#define ERROR_FATAL -0xee

/* Operation failed due to access restriction set in programmer or flash chip */
#define ACCESS_DENIED -7
extern enum error_action access_denied_action;

/* convenience function for checking return codes */
extern int ignore_error(int x);

/* cli_output.c */
/* Let gcc and clang check for correct printf-style format strings. */
int print(int type, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
#define MSG_ERROR	0
#define MSG_INFO	1
#define MSG_DEBUG	2
#define MSG_DEBUG2	3
#define MSG_BARF	4
#define msg_gerr(...)	print(MSG_ERROR, __VA_ARGS__)	/* general errors */
#define msg_perr(...)	print(MSG_ERROR, __VA_ARGS__)	/* programmer errors */
#define msg_cerr(...)	print(MSG_ERROR, __VA_ARGS__)	/* chip errors */
#define msg_ginfo(...)	print(MSG_INFO, __VA_ARGS__)	/* general info */
#define msg_pinfo(...)	print(MSG_INFO, __VA_ARGS__)	/* programmer info */
#define msg_cinfo(...)	print(MSG_INFO, __VA_ARGS__)	/* chip info */
#define msg_gdbg(...)	print(MSG_DEBUG, __VA_ARGS__)	/* general debug */
#define msg_pdbg(...)	print(MSG_DEBUG, __VA_ARGS__)	/* programmer debug */
#define msg_cdbg(...)	print(MSG_DEBUG, __VA_ARGS__)	/* chip debug */
#define msg_gdbg2(...)	print(MSG_DEBUG2, __VA_ARGS__)	/* general debug2 */
#define msg_pdbg2(...)	print(MSG_DEBUG2, __VA_ARGS__)	/* programmer debug2 */
#define msg_cdbg2(...)	print(MSG_DEBUG2, __VA_ARGS__)	/* chip debug2 */
#define msg_gspew(...)	print(MSG_BARF, __VA_ARGS__)	/* general debug barf  */
#define msg_pspew(...)	print(MSG_BARF, __VA_ARGS__)	/* programmer debug barf  */
#define msg_cspew(...)	print(MSG_BARF, __VA_ARGS__)	/* chip debug barf  */

/* cli_mfg.c */
extern int set_ignore_fmap;

/* layout.c */
int specified_partition();
int read_romlayout(char *name);
int find_romentry(char *name);
int handle_romentries(struct flashctx *flash, uint8_t *oldcontents, uint8_t *newcontents);
int add_fmap_entries(struct flashctx *flash);
int get_num_include_args(void);
int register_include_arg(char *name);
int process_include_args(void);
int num_include_files(void);
int included_regions_overlap(void);
int handle_romentries(struct flashctx *flash, uint8_t *oldcontents, uint8_t *newcontents);
int handle_partial_read(
    struct flashctx *flash,
    uint8_t *buf,
    int (*read) (struct flashctx *flash, uint8_t *buf,
                 unsigned int start, unsigned int len),
    int write_to_file);
    /* RETURN: the number of partitions that have beenpartial read.
    *         ==0 means no partition is specified.
    *         < 0 means writing file error. */
int handle_partial_verify(
    struct flashctx *flash,
    uint8_t *buf,
    int (*verify) (struct flashctx *flash, uint8_t *buf, unsigned int start,
                   unsigned int len, const char* message));
    /* RETURN: ==0 means all identical.
               !=0 means buf and flash are different. */

/* spi.c */
struct spi_command {
	unsigned int writecnt;
	unsigned int readcnt;
	const unsigned char *writearr;
	unsigned char *readarr;
};
int spi_send_command(const struct flashctx *flash, unsigned int writecnt, unsigned int readcnt,
		const unsigned char *writearr, unsigned char *readarr);
int spi_send_multicommand(const struct flashctx *flash, struct spi_command *cmds);
uint32_t spi_get_valid_read_addr(struct flashctx *flash);

#define NUM_VOLTAGE_RANGES	16
extern struct voltage_range voltage_ranges[];
/* returns number of unique voltage ranges, or <0 to indicate failure */
extern int flash_supported_voltage_ranges(enum chipbustype bus);

#endif				/* !__FLASH_H__ */
