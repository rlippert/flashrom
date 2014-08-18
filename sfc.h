/*
 * This file is part of the flashrom project.
 *
 * Copyright (C) 2014 Google Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of Google or the names of contributors or
 * licensors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * This software is provided "AS IS," without a warranty of any kind.
 * ALL EXPRESS OR IMPLIED CONDITIONS, REPRESENTATIONS AND WARRANTIES,
 * INCLUDING ANY IMPLIED WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE OR NON-INFRINGEMENT, ARE HEREBY EXCLUDED.
 * GOOGLE INC AND ITS LICENSORS SHALL NOT BE LIABLE
 * FOR ANY DAMAGES SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING
 * OR DISTRIBUTING THIS SOFTWARE OR ITS DERIVATIVES.  IN NO EVENT WILL
 * GOOGLE OR ITS LICENSORS BE LIABLE FOR ANY LOST REVENUE, PROFIT OR DATA,
 * OR FOR DIRECT, INDIRECT, SPECIAL, CONSEQUENTIAL, INCIDENTAL OR
 * PUNITIVE DAMAGES, HOWEVER CAUSED AND REGARDLESS OF THE THEORY OF
 * LIABILITY, ARISING OUT OF THE USE OF OR INABILITY TO USE THIS SOFTWARE,
 * EVEN IF GOOGLE HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 */

/*
 * SPI Flash Controller definitions.
 */

#ifndef __SFC_H__
#define __SFC_H__

#ifdef __cplusplus
extern "C" {
#endif

#define SFC_REG_CONF      0x10 /* CONF: Direct Access Configuration */
#define SFC_REG_CONF_FRZE  (1 << 3)
#define SFC_REG_CONF_ECCEN (1 << 2)
#define SFC_REG_CONF_DRCD  (1 << 1)
#define SFC_REG_CONF_FLRLD (1 << 0)

#define SFC_REG_STATUS    0x0C /* STATUS : Status Reg */
#define SFC_REG_STATUS_NX_ON_SHFT  28
#define SFC_REG_STATUS_RWP         (1 << 27)
#define SFC_REG_STATUS_FOURBYTEAD  (1 << 26)
#define SFC_REG_STATUS_ILLEGAL     (1 << 4)
#define SFC_REG_STATUS_ECCERRCNTN  (1 << 3)
#define SFC_REG_STATUS_ECCUEN      (1 << 2)
#define SFC_REG_STATUS_DONE        (1 << 0)

#define SFC_REG_CMD       0x40 /* CMD : Command */
#define SFC_REG_CMD_OPCODE_SHFT    9
#define SFC_REG_CMD_LENGTH_SHFT    0

#define SFC_REG_SPICLK    0x3C /* SPICLK: SPI clock rate config */
#define SFC_REG_SPICLK_OUTDLY_SHFT     24
#define SFC_REG_SPICLK_INSAMPDLY_SHFT  16
#define SFC_REG_SPICLK_CLKHI_SHFT      8
#define SFC_REG_SPICLK_CLKLO_SHFT      0
#define SFC_REG_ADR       0x44 /* ADR : Address */
#define SFC_REG_ERASMS    0x48 /* ERASMS : Small Erase Block Size */
#define SFC_REG_ERASLGS   0x4C /* ERALGS : Large Erase Block Size */
#define SFC_REG_CONF4     0x54 /* CONF4  : SPI Op Code for Small Erase */
#define SFC_REG_CONF5     0x58 /* CONF5  : Small Erase Size config reg */
#define SFC_REG_CONF8     0x64 /* CONF8  : Read Command */
#define SFC_REG_CONF8_CSINACTIVEREAD_SHFT  18
#define SFC_REG_CONF8_DUMMY_SHFT           8
#define SFC_REG_CONF8_READOP_SHFT          0
#define SFC_REG_ADRCBF    0x80 /* ADRCBF : First Intf NOR Addr Offset */
#define SFC_REG_ADRCMF    0x84 /* ADRCMF : First Intf NOR Allocation */
#define SFC_REG_ADRCBS    0x88 /* ADRCBS : Second Intf NOR Addr Offset */
#define SFC_REG_ADRCMS    0x8C /* ADRCMS : Second Intf NOR Allocation */
#define SFC_REG_OADRNB    0x90 /* OADRNB : Direct Access OBP Window Base Address */
#define SFC_REG_OADRNS    0x94 /* OADRNS : DIrect Access OPB Window Size */

#define SFC_REG_CHIPIDCONF    0x9C /* CHIPIDCONF : config ChipId CMD */
#define SFC_REG_CHIPIDCONF_OPCODE_SHFT 24
#define SFC_REG_CHIPIDCONF_READ        (1 << 23)
#define SFC_REG_CHIPIDCONF_WRITE       (1 << 22)
#define SFC_REG_CHIPIDCONF_USE_ADDR    (1 << 21)
#define SFC_REG_CHIPIDCONF_DUMMY_SHFT  16
#define SFC_REG_CHIPIDCONF_LEN_SHFT    0

#define SFC_OP_READRAW      0x03 /* Read Raw */
#define SFC_OP_WRITERAW     0x02 /* Write Raw */
#define SFC_OP_ERASM        0x32 /* Erase Small */
#define SFC_OP_ERALG        0x34 /* Erase Large */
#define SFC_OP_ENWRITPROT   0x53 /* Enable WRite Protect */
#define SFC_OP_CHIPID       0x1F /* Get Chip ID */
#define SFC_OP_STATUS       0x05 /* Get Status */
#define SFC_OP_TURNOFF      0x5E /* Turn Off */
#define SFC_OP_TURNON       0x50 /* Turn On */
#define SFC_OP_ABORT        0x6F /* Super-Abort */
#define SFC_OP_START4BA     0x37 /* Start 4BA */
#define SFC_OP_END4BA       0x69 /* End 4BA */

#define SFC_CMDBUF_SIZE     256  /* Command buffer is 256 bytes */

/* Return code values for all functions */
#define SFC_SUCCESS         0
#define SFC_ERR_INVALID    -1
#define SFC_ERR_IO         -2
#define SFC_ERR_TIMEOUT    -3

struct sfc {
    /*
     * Initialize the SFC.
     */
    int (*init)(struct sfc *sfc);

    /*
     * Read/write the SFC registers.
     */
    int (*cmdreg_read)(struct sfc *sfc, uint8_t reg, uint32_t *val);
    int (*cmdreg_write)(struct sfc *sfc, uint8_t reg, uint32_t val);

    int (*set_4ba)(struct sfc *sfc, int enabled);

    /*
     * Reads the flash using the SFC command buffer.
     * Args:
     *   offset: 24bit address
     *   len: number of bytes to transfer (0-255)
     *   data: data pointer
     * Returns: SFC status return code
     */
    int (*read_raw)(struct sfc *sfc, uint32_t offset, uint32_t len,
        uint8_t *data);

    /*
     * Writes to the flash using the SFC command buffer.
     * Args:
     *   offset: 24bit address
     *   len: number of bytes to transfer (0-255)
     *   data: data pointer
     * Returns: SFC status return code
     */
    int (*write_raw)(struct sfc *sfc, uint32_t offset, uint32_t len,
        const uint8_t *data);

    /*
     * Gets the erase size for small/large erase operations.
     * Args:
     *   small_size: SFC small erase size
     *   large_size: SFC large erase size
     * Returns: SFC status return code
     */
    int (*get_erase_size)(struct sfc *sfc, uint32_t *small_size,
        uint32_t *large_size);

    /*
     * Performs a SFC small erase operation.
     * Args:
     *   address: flash sector to erase
     * Returns: SFC status return code
     */
    int (*erase_small)(struct sfc *sfc, uint32_t address);

    /*
     * Executes a SPI command packet.
     * Args:
     *   write_cnt: number of bytes to write to device
     *   read_cnt: number of bytes to read from device
     *   write_data: write data buffer
     *   read_data: read data buffer
     * Returns: SFC status return code
     */
    int (*exec_spicommand)(struct sfc *sfc,
        uint32_t writecnt, uint32_t readcnt,
        const uint8_t *writearr, uint8_t *readarr);

    int (*read_lpc)(struct sfc *sfc, uint32_t addr, uint32_t *out);
    int (*write_lpc)(struct sfc *sfc, uint32_t addr, uint32_t val);
    int (*nanosleep)(struct sfc *sfc, uint64_t nsec);

    uint32_t cmd_offset;
    uint32_t cmdbuf_offset;
    uint32_t mmio_offset;
    int mode_4ba;
};

void sfc_init(struct sfc *sfc,
              uint32_t cmd_offset,
              uint32_t cmdbuf_offset,
              uint32_t mmio_offset);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // __SFC_H__
