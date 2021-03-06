/*
 * This file is part of the flashrom project.
 *
 * Copyright (C) 2009 Carl-Daniel Hailfinger
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
 *
 *
 * Header file for flash chip drivers. Included from flash.h.
 * As a general rule, every function listed here should take a pointer to
 * struct flashctx as first parameter.
 */

#ifndef __CHIPDRIVERS_H__
#define __CHIPDRIVERS_H__ 1

#include "flash.h"		/* for chipaddr and flashctx */
#include "writeprotect.h"	/* for generic_modifier_bits */

/* spi.c, should probably be in spi_chip.c */
int probe_spi_rdid(struct flashctx *flash);
int probe_spi_rdid4(struct flashctx *flash);
int probe_spi_rems(struct flashctx *flash);
int probe_spi_res1(struct flashctx *flash);
int probe_spi_res2(struct flashctx *flash);
int spi_write_enable(struct flashctx *flash);
int spi_write_disable(struct flashctx *flash);
int spi_block_erase_20(struct flashctx *flash, unsigned int addr, unsigned int blocklen);
int spi_block_erase_52(struct flashctx *flash, unsigned int addr, unsigned int blocklen);
int spi_block_erase_d7(struct flashctx *flash, unsigned int addr, unsigned int blocklen);
int spi_block_erase_d8(struct flashctx *flash, unsigned int addr, unsigned int blocklen);
int spi_block_erase_dc(struct flashctx *flash, unsigned int addr, unsigned int blocklen);
int spi_block_erase_60(struct flashctx *flash, unsigned int addr, unsigned int blocklen);
int spi_block_erase_c7(struct flashctx *flash, unsigned int addr, unsigned int blocklen);
int spi_chip_write_1(struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len);
int spi_chip_write_256(struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len);
int spi_chip_write_256_4b(struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len);
int spi_chip_read(struct flashctx *flash, uint8_t *buf, unsigned int start, int unsigned len);
int spi_chip_read_4b(struct flashctx *flash, uint8_t *buf, unsigned int start, int unsigned len);
uint8_t spi_read_status_register(const struct flashctx *flash);
uint8_t spi_read_status_register3(const struct flashctx *flash);
int spi_write_status_register(const struct flashctx *flash, int status);
void spi_prettyprint_status_register_bit(uint8_t status, int bit);
void spi_prettyprint_status_register_bp3210(uint8_t status, int bp);
void spi_prettyprint_status_register_welwip(uint8_t status);
int spi_prettyprint_status_register(struct flashctx *flash);
int spi_disable_blockprotect(struct flashctx *flash);
int spi_byte_program(struct flashctx *flash, unsigned int addr, uint8_t databyte);
int spi_nbyte_program(struct flashctx *flash, unsigned int addr, uint8_t *bytes, unsigned int len);
int spi_nbyte_read(struct flashctx *flash, unsigned int addr, uint8_t *bytes, unsigned int len);
int spi_read_chunked(struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len, unsigned int chunksize);
int spi_read_unbound(struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len, unsigned int chunksize);
int spi_write_chunked(struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len, unsigned int chunksize);
int spi_aai_write(struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len);
int spi_write_extended_address(struct flashctx *flash, unsigned char addr);
int spi_read_extended_address(struct flashctx *flash);
int spi_set_4b_mode(struct flashctx *flash, unsigned char mode);

/* opaque.c */
int probe_opaque(struct flashctx *flash);
int read_opaque(struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len);
int write_opaque(struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len);
int erase_opaque(struct flashctx *flash, unsigned int blockaddr, unsigned int blocklen);
uint8_t read_status_opaque(const struct flashctx *flash);
int write_status_opaque(const struct flashctx *flash, int status);

/* a25.c */
int spi_prettyprint_status_register_amic_a25l05p(struct flashctx *flash);
int spi_prettyprint_status_register_amic_a25l40p(struct flashctx *flash);
int spi_prettyprint_status_register_amic_a25l032(struct flashctx *flash);
int spi_prettyprint_status_register_amic_a25lq032(struct flashctx *flash);

/* at25.c */
int spi_prettyprint_status_register_at25df(struct flashctx *flash);
int spi_prettyprint_status_register_at25df_sec(struct flashctx *flash);
int spi_prettyprint_status_register_at25f(struct flashctx *flash);
int spi_prettyprint_status_register_at25fs010(struct flashctx *flash);
int spi_prettyprint_status_register_at25fs040(struct flashctx *flash);
int spi_prettyprint_status_register_atmel_at26df081a(struct flashctx *flash);
int spi_disable_blockprotect_at25df(struct flashctx *flash);
int spi_disable_blockprotect_at25df_sec(struct flashctx *flash);
int spi_disable_blockprotect_at25f(struct flashctx *flash);
int spi_disable_blockprotect_at25fs010(struct flashctx *flash);
int spi_disable_blockprotect_at25fs040(struct flashctx *flash);

/* 82802ab.c */
uint8_t wait_82802ab(struct flashctx *flash);
int probe_82802ab(struct flashctx *flash);
int erase_block_82802ab(struct flashctx *flash, unsigned int page, unsigned int pagesize);
int write_82802ab(struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len);
void print_status_82802ab(uint8_t status);
int unlock_82802ab(struct flashctx *flash);
int unlock_28f004s5(struct flashctx *flash);
int unlock_lh28f008bjt(struct flashctx *flash);

/* ichspi.c */
int ich_hwseq_probe(struct flashctx *flash);
int ich_hwseq_read(struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len);
int ich_hwseq_block_erase(struct flashctx *flash, unsigned int addr, unsigned int blocklen);
int ich_hwseq_write_256(struct flashctx *flash, uint8_t *buf, int start, int len);

/* jedec.c */
uint8_t oddparity(uint8_t val);
void toggle_ready_jedec(struct flashctx *flash, chipaddr dst);
void data_polling_jedec(struct flashctx *flash, chipaddr dst, uint8_t data);
int write_byte_program_jedec(struct flashctx *flash, chipaddr bios, uint8_t *src,
			     chipaddr dst);
int probe_jedec(struct flashctx *flash);
int write_jedec(struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len);
int write_jedec_1(struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len);
int erase_sector_jedec(struct flashctx *flash, unsigned int page, unsigned int pagesize);
int erase_block_jedec(struct flashctx *flash, unsigned int page, unsigned int blocksize);
int erase_chip_block_jedec(struct flashctx *flash, unsigned int page, unsigned int blocksize);

/* m29f400bt.c */
int probe_m29f400bt(struct flashctx *flash);
int block_erase_m29f400bt(struct flashctx *flash, unsigned int start, unsigned int len);
int block_erase_chip_m29f400bt(struct flashctx *flash, unsigned int start, unsigned int len);
int write_m29f400bt(struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len);
void protect_m29f400bt(struct flashctx *flash, chipaddr bios);

/* pm49fl00x.c */
int unlock_49fl00x(struct flashctx *flash);
int lock_49fl00x(struct flashctx *flash);

/* sst28sf040.c */
int erase_chip_28sf040(struct flashctx *flash, unsigned int addr, unsigned int blocklen);
int erase_sector_28sf040(struct flashctx *flash, unsigned int address, unsigned int sector_size);
int write_28sf040(struct flashctx *flash, uint8_t *buf, unsigned int start, unsigned int len);
int unprotect_28sf040(struct flashctx *flash);
int protect_28sf040(struct flashctx *flash);

/* sst49lfxxxc.c */
int erase_sector_49lfxxxc(struct flashctx *flash, unsigned int address, unsigned int sector_size);
int unlock_49lfxxxc(struct flashctx *flash);

/* sst_fwhub.c */
int printlock_sst_fwhub(struct flashctx *flash);
int unlock_sst_fwhub(struct flashctx *flash);

/* s25fl.c */
int probe_spi_big_spansion(struct flashctx *flash);
int s25fl_block_erase(struct flashctx *flash, unsigned int addr, unsigned int blocklen);

/* s25f.c */
int s25f_get_modifier_bits(const struct flashctx *flash, struct generic_modifier_bits *m);
int s25f_set_modifier_bits(const struct flashctx *flash, struct generic_modifier_bits *m);
int s25fs_block_erase_d8(struct flashctx *flash, unsigned int addr, unsigned int blocklen);

/* w39.c */
int printlock_w39l040(struct flashctx * flash);
int printlock_w39v040a(struct flashctx *flash);
int printlock_w39v040b(struct flashctx *flash);
int printlock_w39v040c(struct flashctx *flash);
int printlock_w39v040fa(struct flashctx *flash);
int printlock_w39v040fb(struct flashctx *flash);
int printlock_w39v040fc(struct flashctx *flash);
int printlock_w39v080a(struct flashctx *flash);
int printlock_w39v080fa(struct flashctx *flash);
int printlock_w39v080fa_dual(struct flashctx *flash);
int unlock_w39v040fb(struct flashctx *flash);
int unlock_w39v080fa(struct flashctx *flash);

/* w29ee011.c */
int probe_w29ee011(struct flashctx *flash);

/* stm50flw0x0x.c */
int erase_sector_stm50flw0x0x(struct flashctx *flash, unsigned int block, unsigned int blocksize);
int unlock_stm50flw0x0x(struct flashctx *flash);

/* dummyflasher.c */
int probe_variable_size(struct flashctx *flash);

#endif /* !__CHIPDRIVERS_H__ */
