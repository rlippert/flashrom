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
 * Implementation of SPI Flash Controller (SFC) functionality.
 *
 * The SFC is accessed from the LPC bus and provides a register set,
 * data/command buffer, and MMIO "direct access" space.
 *
 * The SFC hardware takes care of many of the SPI protocol details when
 * sending basic read/write commands or using the direct access space.
 *
 * Additionally limited support for "raw" SPI commands is provided via the
 * CHIPID command and configuration registers.
 *
 * This is intended as a very basic support layer and does not configure
 * all of the windows and opcodes and such that are necessary for proper
 * functioning of the device.
 */

#include <stdint.h>
#include <string.h>

#include "sfc.h"

/* Command register support */
static int sfc_cmdreg_read(struct sfc *sfc, uint8_t reg, uint32_t *val) {
    if (!sfc->read_lpc) {
        return SFC_ERR_INVALID;
    }
    return sfc->read_lpc(sfc, sfc->cmd_offset + reg, val);
}

static int sfc_cmdreg_write(struct sfc *sfc, uint8_t reg, uint32_t val) {
    if (!sfc->write_lpc) {
        return SFC_ERR_INVALID;
    }
    return sfc->write_lpc(sfc, sfc->cmd_offset + reg, val);
}

/* Command buffer support.
 * The buffer is used for read and write commands to store the data to be
 * transmitted or read from the device.
 * The command buffer is assumed to be in big-endian format with the data in
 * byte 0 being sent to or received from the device first.
 */
static int sfc_buf_write(struct sfc *sfc,
                         uint32_t len, const uint8_t *data) {
    int i;

    if (!sfc->write_lpc || len > SFC_CMDBUF_SIZE) {
        return SFC_ERR_INVALID;
    }

    for (i = 0; i < len; i += sizeof(uint32_t)) {
        uint32_t word = 0;
        if (i < len) word |= (data[i] << 24);
        if ((i + 1) < len) word |= (data[i + 1] << 16);
        if ((i + 2) < len) word |= (data[i + 2] << 8);
        if ((i + 3) < len) word |= (data[i + 3] << 0);

        int rc = sfc->write_lpc(sfc, sfc->cmdbuf_offset + i, word);
        if (rc != 0) {
            return rc;
        }
    }

    return SFC_SUCCESS;
}

static int sfc_buf_read(struct sfc *sfc,
                        uint32_t len, uint8_t *data) {
    int i;

    if (!sfc->read_lpc || len > SFC_CMDBUF_SIZE) {
        return SFC_ERR_INVALID;
    }

    for (i = 0; i < len; i += sizeof(uint32_t)) {
        uint32_t word = 0;
        int rc = sfc->read_lpc(sfc, sfc->cmdbuf_offset + i, &word);
        if (rc != 0) {
            return rc;
        }

        if (i < len) data[i] = (word >> 24) & 0xff;
        if ((i + 1) < len) data[i + 1] = (word >> 16) & 0xff;
        if ((i + 2) < len) data[i + 2] = (word >> 8) & 0xff;
        if ((i + 3) < len) data[i + 3] = (word >> 0) & 0xff;
    }

    return SFC_SUCCESS;
}

/* Polls until SFC indicates command is complete */
static int sfc_poll_complete(struct sfc *sfc) {
    uint32_t status_reg;
    /* TODO: test timeout values on real hardware */
    int timeout = 10000;

    do {
        int rc;

        rc = sfc_cmdreg_read(sfc, SFC_REG_STATUS, &status_reg);
        if (rc != 0) {
            return rc;
        }
        if (status_reg & SFC_REG_STATUS_DONE) {
            break;
        }

        if (sfc->nanosleep) {
            sfc->nanosleep(sfc, 10000);
        }
    } while (timeout-- > 0);

    if (timeout == 0) {
        return SFC_ERR_TIMEOUT;
    }
    if (!(status_reg & SFC_REG_STATUS_DONE)) {
        return SFC_ERR_IO;
    }
    return SFC_SUCCESS;
}

/*
 * Executes a single SFC command.
 * Args:
 *   opcode: SFC 7-bit opcode
 *   length: length of transfer (not applicable to all commands)
 * Returns: SFC status return code
 */
static int sfc_exec_command(struct sfc *sfc,
                            uint8_t opcode, uint32_t length) {
    int rc = SFC_SUCCESS;
    uint32_t cmd_reg = 0;

    if (opcode > 0x7f || length > 0x1ff) {
        return SFC_ERR_INVALID;
    }

    /* Write command register to start execution */
    cmd_reg |= (opcode << SFC_REG_CMD_OPCODE_SHFT);
    cmd_reg |= (length << SFC_REG_CMD_LENGTH_SHFT);
    rc = sfc_cmdreg_write(sfc, SFC_REG_CMD, cmd_reg);
    if (rc != SFC_SUCCESS) {
        return rc;
    }

    /* Wait for command to complete */
    rc = sfc_poll_complete(sfc);
    if (rc != SFC_SUCCESS) {
        return rc;
    }

    return rc;
}

/*
 * Executes a SFC CHIPID command which can also execute a limited set of
 * arbitrary SPI commands.
 * Args:
 *   opcode: SPI 8-bit opcode
 *   use_address: true if command should transfer address bytes first
 *   address: SPI 4-byte address
 *   read_op: true if the operation expects to read data from device
 *   datacnt: number of data bytes to read/write
 *   data: data buffer for reading/writing
 * Returns: SFC status return code
 */
static int sfc_chipid_cmd(struct sfc *sfc,
                          uint8_t opcode, int use_address, uint32_t address,
                          int read_op, uint32_t datacnt, uint8_t *data) {
    uint32_t chipidconf = 0;
    int rc = SFC_SUCCESS;

    chipidconf |= (opcode << SFC_REG_CHIPIDCONF_OPCODE_SHFT);
    /* Handle address bytes */
    if (use_address) {
        chipidconf |= SFC_REG_CHIPIDCONF_USE_ADDR;

        rc = sfc_cmdreg_write(sfc, SFC_REG_ADR, address);
        if (rc != SFC_SUCCESS) {
            return rc;
        }
    }
    /* Handle data bytes */
    if (datacnt > SFC_SUCCESS) {
        if (read_op) {
            chipidconf |= SFC_REG_CHIPIDCONF_READ;
        } else {
            chipidconf |= SFC_REG_CHIPIDCONF_WRITE;

            /* Prepare the data buffer with data to be written */
            rc = sfc_buf_write(sfc, datacnt, data);
            if (rc != 0) {
                return rc;
            }
        }

        chipidconf |= (datacnt << SFC_REG_CHIPIDCONF_LEN_SHFT);
    }

    /* Configure the chip id command register */
    rc = sfc_cmdreg_write(sfc, SFC_REG_CHIPIDCONF, chipidconf);
    if (rc != SFC_SUCCESS) {
        return rc;
    }

    /* Execute the command */
    rc = sfc_exec_command(sfc, SFC_OP_CHIPID, 0);
    if (rc != SFC_SUCCESS) {
        return rc;
    }

    /* Copy out data buffer for read ops */
    if (datacnt > 0 && read_op) {
        rc = sfc_buf_read(sfc, datacnt, data);
        if (rc != SFC_SUCCESS) {
            return rc;
        }
    }

    return SFC_SUCCESS;
}

static int sfc_init_regs(struct sfc *sfc) {
    int rc;
    uint32_t status_reg;

    /* Read current state of 4 byte addressing */
    rc = sfc_cmdreg_read(sfc, SFC_REG_STATUS, &status_reg);
    if (rc != SFC_SUCCESS) {
        return rc;
    }
    sfc->mode_4ba = (status_reg & SFC_REG_STATUS_FOURBYTEAD) ? 1 : 0;

    return rc;
}

static int sfc_set_4ba(struct sfc *sfc, int enabled) {
    int rc;

    rc = sfc_exec_command(sfc, enabled ? SFC_OP_START4BA : SFC_OP_END4BA, 0);
    if (rc != SFC_SUCCESS) {
        return rc;
    }

    sfc->mode_4ba = enabled;
    return rc;
}

static int sfc_readraw(struct sfc *sfc,
                       uint32_t offset, uint32_t len, uint8_t *data) {
    int rc = SFC_SUCCESS;

    if (len > SFC_CMDBUF_SIZE) {
        return SFC_ERR_INVALID;
    }

    rc = sfc_cmdreg_write(sfc, SFC_REG_ADR, offset);
    if (rc != SFC_SUCCESS) {
        return rc;
    }
    rc = sfc_exec_command(sfc, SFC_OP_READRAW, len);
    if (rc != SFC_SUCCESS) {
        return rc;
    }
    rc = sfc_buf_read(sfc, len, data);
    if (rc != SFC_SUCCESS) {
        return rc;
    }

    return rc;
}

static int sfc_writeraw(struct sfc *sfc,
                        uint32_t offset, uint32_t len, const uint8_t *data) {
    int rc = 0;

    if (len > SFC_CMDBUF_SIZE) {
        return -1;
    }

    rc = sfc_buf_write(sfc, len, data);
    if (rc != SFC_SUCCESS) {
        return rc;
    }
    rc = sfc_cmdreg_write(sfc, SFC_REG_ADR, offset);
    if (rc != SFC_SUCCESS) {
        return rc;
    }
    rc = sfc_exec_command(sfc, SFC_OP_WRITERAW, len);
    if (rc != SFC_SUCCESS) {
        return rc;
    }

    return rc;
}

static int sfc_get_erase_size(struct sfc *sfc,
                              uint32_t *small_size, uint32_t *large_size) {
    int rc = 0;

    if (small_size) {
        rc |= sfc_cmdreg_read(sfc, SFC_REG_ERASMS, small_size);
    }
    if (large_size) {
        rc |= sfc_cmdreg_read(sfc, SFC_REG_ERASLGS, large_size);
    }

    return rc;
}

static int sfc_erasesmall(struct sfc *sfc, uint32_t address) {
    int rc = SFC_SUCCESS;

    rc = sfc_cmdreg_write(sfc, SFC_REG_ADR, address);
    if (rc != SFC_SUCCESS) {
        return rc;
    }
    rc = sfc_exec_command(sfc, SFC_OP_ERASM, 0);
    if (rc != SFC_SUCCESS) {
        return rc;
    }

    return rc;
}

/*
 * Executes a SPI command packet.
 * Args:
 *   write_cnt: number of bytes to write to device
 *   read_cnt: number of bytes to read from device
 *   write_data: write data buffer
 *   read_data: read data buffer
 * Returns: SFC status return code
 */
static int sfc_exec_spicommand(struct sfc *sfc,
                               uint32_t write_cnt, uint32_t read_cnt,
                               const uint8_t *write_data, uint8_t *read_data) {
    uint8_t opcode = 0;
    uint32_t address = 0;
    int address_bytes = 0;
    uint32_t data_cnt = 0;
    uint8_t *data = NULL;
    int read_op = 0;

    if (write_cnt == 0) {
        /* Error: malformed SPI packet */
        return SFC_ERR_INVALID;
    }
    opcode = write_data[0];

    /* If we are writing data (more than opcode), check for address */
    if (!sfc->mode_4ba && write_cnt >= 4) {  /* 3 byte address */
        address_bytes = 3;
        address = (write_data[1] << 16) |
            (write_data[2] << 8) |
            write_data[3];
    } else if (sfc->mode_4ba && write_cnt >= 5) {  /* 4 byte address */
        address_bytes = 4;
        address = (write_data[1] << 24) |
            (write_data[2] << 16) |
            (write_data[3] << 8) |
            write_data[4];
    } else if (write_cnt >= 2) {  /* no address */
        address_bytes = 0;
    }

    if (write_cnt > (address_bytes + 1)) {  /* write data */
        if (read_cnt > 0) {
            /* Can't read and write at the same time */
            return SFC_ERR_INVALID;
        }
        read_op = 0;
        data_cnt = write_cnt - (address_bytes + 1);
        data = (uint8_t*)(&write_data[address_bytes + 1]);
    } else if (read_cnt > 0) {  /* read data */
        read_op = 1;
        data_cnt = read_cnt;
        data = read_data;
    }

    return sfc_chipid_cmd(sfc, opcode, address_bytes > 0 ? 1 : 0, address,
                          read_op, data_cnt, data);
}

void sfc_init(struct sfc *sfc,
              uint32_t cmd_offset,
              uint32_t cmdbuf_offset,
              uint32_t mmio_offset) {
    sfc->init = sfc_init_regs;
    sfc->cmdreg_read = sfc_cmdreg_read;
    sfc->cmdreg_write = sfc_cmdreg_write;
    sfc->set_4ba = sfc_set_4ba;
    sfc->read_raw = sfc_readraw;
    sfc->write_raw = sfc_writeraw;
    sfc->get_erase_size = sfc_get_erase_size;
    sfc->erase_small = sfc_erasesmall;
    sfc->exec_spicommand = sfc_exec_spicommand;
    /* The following must be filled in by caller */
    sfc->read_lpc = NULL;
    sfc->write_lpc = NULL;
    sfc->nanosleep = NULL;

    sfc->cmd_offset = cmd_offset;
    sfc->cmdbuf_offset = cmdbuf_offset;
    sfc->mmio_offset = mmio_offset;
}

/* ======================================================================= */
/* Flashrom specific code starts below, above code is shared with firmware */
/* ======================================================================= */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include "chipdrivers.h"
#include "flash.h"
#include "programmer.h"
#include "spi.h"
#include "writeprotect.h"

#define DEFAULT_SFC_CMD_OFFSET 0xc00
#define DEFAULT_SFC_CMDBUF_OFFSET 0xd00
#define DEFAULT_SFC_MMIO_OFFSET 0xc000000

static struct sfc flashrom_sfc;

int sfc_fw_fd = -1;

static int flashrom_sfc_read_lpc(struct sfc *sfc, uint32_t addr, uint32_t *out)
{
	int rc = pread(sfc_fw_fd, out, sizeof(uint32_t), addr);
	msg_pspew("LPC RD [%d] %08x=%08x\n", rc, addr, *out);
	return rc == sizeof(uint32_t) ? SFC_SUCCESS : SFC_ERR_IO;
}

static int flashrom_sfc_write_lpc(struct sfc *sfc, uint32_t addr, uint32_t val)
{
	int rc = pwrite(sfc_fw_fd, &val, sizeof(uint32_t), addr);
	msg_pspew("LPC WR [%d] %08x=%08x\n", rc, addr, val);
	return rc == sizeof(uint32_t) ? SFC_SUCCESS : SFC_ERR_IO;
}

static int flashrom_sfc_nanosleep(struct sfc *sfc, uint64_t nsec)
{
	programmer_delay(nsec / 1000);
	return 0;
}

static int flashrom_sfc_spi_send_command(const struct flashctx *flash,
					 unsigned int writecnt,
					 unsigned int readcnt,
					 const unsigned char *writearr,
					 unsigned char *readarr)
{
	int rc = flashrom_sfc.exec_spicommand(&flashrom_sfc, writecnt, readcnt,
	                                      writearr, readarr);
	msg_pspew("CMD [%d] writecnt=%u readcnt=%u\n", rc, writecnt, readcnt);
	return rc;
}

static int flashrom_sfc_read(struct flashctx *flash, uint8_t *buf,
                             unsigned int start, unsigned int len)
{
	/* We can only read 256 bytes at a time */
	while (len > 0)
	{
		/*
		 * Flash chip writes wrap at 256 byte "pages" internal to the
		 * device.  Adjust the write length so we don't wrap on
		 * unaligned offsets.
		 */
		int chunksize = min(len, SFC_CMDBUF_SIZE - (start & 0xff));
		int rc = flashrom_sfc.read_raw(&flashrom_sfc, start,
		                               chunksize, buf);
		msg_pspew("READ [%d] start=%d len=%d\n", rc, start, chunksize);
		if (rc) {
			return rc;
		}
		start += chunksize;
		buf += chunksize;
		len -= chunksize;
	}
	return 0;
}

static int flashrom_sfc_write(struct flashctx *flash, uint8_t *buf,
                              unsigned int start, unsigned int len)
{
	/* We can only write 256 bytes at a time */
	while (len > 0)
	{
		spi_write_enable(flash);
		int chunksize = min(len, SFC_CMDBUF_SIZE);
		int rc = flashrom_sfc.write_raw(&flashrom_sfc, start,
		                                chunksize, buf);
		msg_pspew("WRITE [%d] start=%d len=%d\n", rc, start, chunksize);
		if (rc) {
			return rc;
		}
		start += chunksize;
		buf += chunksize;
		len -= chunksize;
	}
	return 0;
}

static const struct spi_programmer spi_programmer_sfc = {
	.type		= SPI_CONTROLLER_LPC2SPI_SFC,
	.feature_bits	= SPI_FEAT_READ_4BA | SPI_FEAT_WRITE_4BA,
	.max_data_read	= SFC_CMDBUF_SIZE,
	.max_data_write	= SFC_CMDBUF_SIZE,
	.command	= flashrom_sfc_spi_send_command,
	.multicommand	= default_spi_send_multicommand,
	.read		= flashrom_sfc_read,
	.write_256	= flashrom_sfc_write,
};

/* log2() could be used if we link with -lm */
static unsigned int logbase2(unsigned int x)
{
	unsigned int log = 0;

	/* naive way */
	while (x) {
		x >>= 1;
		log++;
	}
	return log;
}

static int sfc_wp_list_ranges(const struct flashctx *flash)
{
	uint32_t start, size;
	int rc;
	rc = flashrom_sfc.cmdreg_read(&flashrom_sfc, SFC_REG_PROTA, &start);
	if (rc != SFC_SUCCESS)
		return rc;
	rc = flashrom_sfc.cmdreg_read(&flashrom_sfc, SFC_REG_PROTM, &size);
	if (rc != SFC_SUCCESS)
		return rc;

	/* The region size is stored as a power of 2 in the lower 5 bits */
	size &= 0x1f;
	size = size ? (4096 * (1 << (size - 1))) : 0;

	msg_pinfo("  start=0x%08x len=0x%08x\n", start, size);

	return 0;
}

static int sfc_wp_set_range(const struct flashctx *flash,
                            unsigned int start, unsigned int len)
{
	int rc;

	/* The length must be a power of 2 between 4K and 64MB, inclusive.
	 * The length must cleanly divide the start.
	 */
	if (len < 4096 || len > (64 * 1024 * 1024) ||
	    ((len & (len - 1)) != 0) ||  /* power of 2 */
	    (start > (64 * 1024 * 1024)) || (start % len != 0)) {
		msg_perr("FAILED: Unsupported write protection range "
		         "(0x%08x,0x%08x)\n\n", start, len);
		return 1;
	}

	rc = flashrom_sfc.cmdreg_write(&flashrom_sfc, SFC_REG_PROTA, start);
	if (rc != SFC_SUCCESS)
		return rc;
	rc = flashrom_sfc.cmdreg_write(&flashrom_sfc, SFC_REG_PROTM,
				       logbase2(len / 4096));
	if (rc != SFC_SUCCESS)
		return rc;

	return 0;
}

static int sfc_wp_enable(const struct flashctx *flash,
		         enum wp_mode wp_mode)
{
	int rc;
	uint32_t chipidconf = 0;

	/* Since we are about to lock the registers, make sure the CHIPID
	 * configuration register contains the SPI opcode that we need
	 * in order to execute a "Read ID" command and not something malicious
	 * like an erase or write command.
	 * This is a little paranoid since it should already be set up
	 * correctly when we probed the flash type before getting to this point
	 * but better safe than sorry...
	 */
	chipidconf = (JEDEC_RDID << SFC_REG_CHIPIDCONF_OPCODE_SHFT) |
		SFC_REG_CHIPIDCONF_READ |
		(JEDEC_RDID_INSIZE << SFC_REG_CHIPIDCONF_LEN_SHFT);
	rc = flashrom_sfc.cmdreg_write(&flashrom_sfc, SFC_REG_CHIPIDCONF,
				       chipidconf);
	if (rc != SFC_SUCCESS)
		return rc;

	switch (wp_mode) {
	case WP_MODE_POWER_CYCLE:
		rc = sfc_exec_command(&flashrom_sfc, SFC_OP_ENWRITPROT, 0);
		break;
	default:
		msg_perr("%s():%d Only 'power_cycle' write-protection mode "
			"is supported\n", __func__, __LINE__);
		rc = 1;
		break;
	}

	return rc;
}

static int sfc_wp_disable(const struct flashctx *flash)
{
	msg_perr("%s():%d Write protect disable is unavailable\n",
	         __func__, __LINE__);
	return 1;
}

static int sfc_wp_status(const struct flashctx *flash)
{
	uint32_t status;
	int rc;
	int enabled = 0;
	rc = flashrom_sfc.cmdreg_read(&flashrom_sfc, SFC_REG_STATUS, &status);
	if (rc != SFC_SUCCESS)
		return 1;

	/* Check the region write protect bit */
	enabled = !!(status & SFC_REG_STATUS_RWP);

	/* Using same format as SPI output */
	msg_pinfo("WP: status: 0x%02x\n", enabled ? 0x80 : 0x00);
	msg_pinfo("WP: status.srp0: %x\n", enabled);
	msg_pinfo("WP: write protect is %s.\n",
			enabled ? "enabled" : "disabled");
	msg_pinfo("WP: write protect range:");
	sfc_wp_list_ranges(flash);

	return 0;
}

static struct wp sfc_wp =
{
	.list_ranges    = sfc_wp_list_ranges,
	.set_range      = sfc_wp_set_range,
	.enable         = sfc_wp_enable,
	.disable        = sfc_wp_disable,
	.wp_status      = sfc_wp_status,
};

/* TODO: commandline parameter? */
static const char lpc_fw_file[] = "/sys/kernel/debug/powerpc/lpc/fw";

static int flashrom_sfc_shutdown(void *data)
{
	msg_pspew("%s\n", __func__);

	close(sfc_fw_fd);
	return 0;
}

int flashrom_sfc_init(void)
{
	msg_pspew("%s\n", __func__);

	sfc_init(&flashrom_sfc,
	         DEFAULT_SFC_CMD_OFFSET,
	         DEFAULT_SFC_CMDBUF_OFFSET,
	         DEFAULT_SFC_MMIO_OFFSET);
	flashrom_sfc.read_lpc = flashrom_sfc_read_lpc;
	flashrom_sfc.write_lpc = flashrom_sfc_write_lpc;
	flashrom_sfc.nanosleep = flashrom_sfc_nanosleep;

	sfc_fw_fd = open(lpc_fw_file, O_RDWR);
	if (sfc_fw_fd < 0) {
		msg_perr("Cannot open LPC FW file: %s (%s)\n",
		         lpc_fw_file, strerror(errno));
		return 1;
	}

	if (flashrom_sfc.init(&flashrom_sfc) != SFC_SUCCESS)
	{
		msg_perr("SFC: initialization failure\n");
		return 1;
	}

	if (register_shutdown(flashrom_sfc_shutdown, NULL))
		return 1;

	register_spi_programmer(&spi_programmer_sfc);
	return 0;
}

int flashrom_sfc_probe_chip(struct flashctx *flash)
{
	/* Override any chip-level locking with programmer-level locking */
	flash->wp = &sfc_wp;
	return 0;
}
