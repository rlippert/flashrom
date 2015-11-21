/*
 * This file is part of the flashrom project.
 *
 * Copyright (C) 2010 Carl-Daniel Hailfinger
 * Copyright (C) 2015 Simon Glass
 * Copyright (C) 2015 Stefan Tauner
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

//#include "platform.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <errno.h>

#if IS_WINDOWS
#include <lusb0_usb.h>
#else
#include <libusb.h>
#endif

#include "flash.h"
#include "flashchips.h"
#include "chipdrivers.h"
#include "programmer.h"
#include "spi.h"

#define FIRMWARE_VERSION(x,y,z) ((x << 16) | (y << 8) | z)
#define DEFAULT_TIMEOUT 3000
#define DEDIPROG_ASYNC_TRANSFERS 8 /* at most 8 asynchronous transfers */
#define REQTYPE_OTHER_OUT (LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_OTHER)	/* 0x43 */
#define REQTYPE_OTHER_IN (LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_OTHER)	/* 0xC3 */
#define REQTYPE_EP_OUT (LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_ENDPOINT)	/* 0x42 */
#define REQTYPE_EP_IN (LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_ENDPOINT)	/* 0xC2 */
struct libusb_context *usb_ctx;
static libusb_device_handle *dediprog_handle;
static int dediprog_in_endpoint;
static int dediprog_out_endpoint;
static int dediprog_firmwareversion = FIRMWARE_VERSION(0, 0, 0);

enum dediprog_devtype {
	DEV_UNKNOWN		= 0,
	DEV_SF100		= 100,
	DEV_SF600		= 600,
};

enum dediprog_devtype dediprog_devicetype;

enum dediprog_leds {
	LED_INVALID		= -1,
	LED_NONE		= 0,
	LED_PASS		= 1 << 0,
	LED_BUSY		= 1 << 1,
	LED_ERROR		= 1 << 2,
	LED_ALL			= 7,
};

/* IO bits for CMD_SET_IO_LED message */
enum dediprog_ios {
	IO1			= 1 << 0,
	IO2			= 1 << 1,
	IO3			= 1 << 2,
	IO4			= 1 << 3,
};

enum dediprog_cmds {
	CMD_TRANSCEIVE		= 0x01,
	CMD_POLL_STATUS_REG	= 0x02,
	CMD_SET_VPP		= 0x03,
	CMD_SET_TARGET		= 0x04,
	CMD_READ_EEPROM		= 0x05,
	CMD_WRITE_EEPROM	= 0x06,
	CMD_SET_IO_LED		= 0x07,
	CMD_READ_PROG_INFO	= 0x08,
	CMD_SET_VCC		= 0x09,
	CMD_SET_STANDALONE	= 0x0A,
	CMD_SET_VOLTAGE		= 0x0B,	/* Only in firmware older than 6.0.0 */
	CMD_GET_BUTTON		= 0x11,
	CMD_GET_UID		= 0x12,
	CMD_SET_CS		= 0x14,
	CMD_IO_MODE		= 0x15,
	CMD_FW_UPDATE		= 0x1A,
	CMD_FPGA_UPDATE		= 0x1B,
	CMD_READ_FPGA_VERSION	= 0x1C,
	CMD_SET_HOLD		= 0x1D,
	CMD_READ		= 0x20,
	CMD_WRITE		= 0x30,
	CMD_WRITE_AT45DB	= 0x31,
	CMD_NAND_WRITE		= 0x32,
	CMD_NAND_READ		= 0x33,
	CMD_SET_SPI_CLK		= 0x61,
	CMD_CHECK_SOCKET	= 0x62,
	CMD_DOWNLOAD_PRJ	= 0x63,
	CMD_READ_PRJ_NAME	= 0x64,
	// New protocol/firmware only
	CMD_CHECK_SDCARD	= 0x65,
	CMD_READ_PRJ		= 0x66,
};

enum dediprog_target {
	FLASH_TYPE_APPLICATION_FLASH_1	= 0,
	FLASH_TYPE_FLASH_CARD,
	FLASH_TYPE_APPLICATION_FLASH_2,
	FLASH_TYPE_SOCKET,
};

enum dediprog_readmode {
	READ_MODE_STD			= 1,
	READ_MODE_FAST			= 2,
	READ_MODE_ATMEL45		= 3,
	READ_MODE_4B_ADDR_FAST		= 4,
	READ_MODE_4B_ADDR_FAST_0x0C	= 5, /* New protocol only */
};

enum dediprog_writemode {
	WRITE_MODE_PAGE_PGM 			= 1,
	WRITE_MODE_PAGE_WRITE			= 2,
	WRITE_MODE_1B_AAI			= 3,
	WRITE_MODE_2B_AAI			= 4,
	WRITE_MODE_128B_PAGE			= 5,
	WRITE_MODE_PAGE_AT26DF041		= 6,
	WRITE_MODE_SILICON_BLUE_FPGA		= 7,
	WRITE_MODE_64B_PAGE_NUMONYX_PCM		= 8,	/* unit of length 512 bytes */
	WRITE_MODE_4B_ADDR_256B_PAGE_PGM	= 9,
	WRITE_MODE_32B_PAGE_PGM_MXIC_512K	= 10,	/* unit of length 512 bytes */
	WRITE_MODE_4B_ADDR_256B_PAGE_PGM_0x12	= 11,
	WRITE_MODE_4B_ADDR_256B_PAGE_PGM_FLAGS	= 12,
};

enum dediprog_standalone_mode {
	ENTER_STANDALONE_MODE = 0,
	LEAVE_STANDALONE_MODE = 1,
};

#ifndef LIBUSB_HAVE_ERROR_NAME
/* Quick and dirty replacement for missing libusb_error_name in older libusb 1.0. */
const char *libusb_error_name(int error_code)
{
	/* 18 chars for text, rest for number, sign, nullbyte. */
	static char my_libusb_error[18 + 6];

	sprintf(my_libusb_error, "libusb error code %i", error_code);
	return my_libusb_error;
}
#endif

/* Returns true if firmware (and thus hardware) supports the "new" protocol */
static int is_new_prot(void)
{
	switch (dediprog_devicetype) {
	case DEV_SF100:
		return dediprog_firmwareversion >= FIRMWARE_VERSION(5, 5, 0);
	case DEV_SF600:
		return dediprog_firmwareversion >= FIRMWARE_VERSION(6, 9, 0);
	default:
		return 0;
	}
}

struct dediprog_transfer_status {
	int error; /* OK if 0, ERROR else */
	unsigned int queued_idx;
	unsigned int finished_idx;
};

static void dediprog_bulk_read_cb(struct libusb_transfer *const transfer)
{
	struct dediprog_transfer_status *const status = (struct dediprog_transfer_status *)transfer->user_data;
	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		status->error = 1;
		msg_perr("SPI bulk read failed!\n");
	}
	++status->finished_idx;
}

static int dediprog_bulk_read_poll(const struct dediprog_transfer_status *const status, const int finish)
{
	if (status->finished_idx >= status->queued_idx)
		return 0;

	do {
		struct timeval timeout = { 10, 0 };
		const int ret = libusb_handle_events_timeout_completed(usb_ctx, &timeout, NULL);
		if (ret < 0) {
			msg_perr("Polling read events failed: %i %s!\n", ret, libusb_error_name(ret));
			return 1;
		}
	} while (finish && (status->finished_idx < status->queued_idx));
	return 0;
}

static int dediprog_read_ep(enum dediprog_cmds cmd, unsigned int value, unsigned int idx, uint8_t *bytes, size_t size)
{
	return libusb_control_transfer(dediprog_handle, REQTYPE_EP_IN, cmd, value, idx,
				      (unsigned char *)bytes, size, DEFAULT_TIMEOUT);
}

static int dediprog_write_ep(enum dediprog_cmds cmd, unsigned int value, unsigned int idx, const uint8_t *bytes, size_t size)
{
	return libusb_control_transfer(dediprog_handle, REQTYPE_EP_OUT, cmd, value, idx,
				      (unsigned char *)bytes, size, DEFAULT_TIMEOUT);
}

static int dediprog_read_other(enum dediprog_cmds cmd, unsigned int value, unsigned int idx, const uint8_t *bytes, size_t size)
{
	return libusb_control_transfer(dediprog_handle, REQTYPE_OTHER_IN, cmd, value, idx,
				      (unsigned char *)bytes, size, DEFAULT_TIMEOUT);
}

#if 0
/* Might be useful for other USB devices as well. static for now. */
/* device parameter allows user to specify one device of multiple installed */
static struct usb_device *get_device_by_vid_pid(uint16_t vid, uint16_t pid, unsigned int device)
{
	struct usb_bus *bus;
	struct usb_device *dev;

	for (bus = usb_get_busses(); bus; bus = bus->next)
		for (dev = bus->devices; dev; dev = dev->next)
			if ((dev->descriptor.idVendor == vid) &&
			    (dev->descriptor.idProduct == pid)) {
				if (device == 0)
					return dev;
				device--;
			}

	return NULL;
}
#endif

/* Might be useful for other USB devices as well. static for now. */
/* device parameter allows user to specify one device of multiple installed */
static struct libusb_device_handle *get_device_by_vid_pid_number(uint16_t vid, uint16_t pid, unsigned int num)
{
	struct libusb_device **list;
	ssize_t i = 0;
	int err = 0;
	struct libusb_device_handle *handle = NULL;
	struct libusb_device_descriptor desc = {};
	ssize_t count = libusb_get_device_list(usb_ctx, &list);

	if (count < 0) {
		msg_perr("Getting the USB device list failed (%s)!\n", libusb_error_name(count));
		return NULL;
	}

	for (i = 0; i < count; i++) {
		struct libusb_device *dev = list[i];
		err = libusb_get_device_descriptor(dev, &desc);
		if (err != 0) {
			msg_perr("Reading the USB device descriptor failed (%s)!\n", libusb_error_name(err));
			libusb_free_device_list(list, 1);
			return NULL;
		}
		if ((desc.idVendor == vid) && (desc.idProduct == pid)) {
			msg_pdbg("Found USB device %04hx:%04hx at address %hhx-%hhx.\n", desc.idVendor,
				 desc.idProduct, libusb_get_bus_number(dev), libusb_get_device_address(dev));
			if (num == 0) {
				err = libusb_open(dev, &handle);
				if (err != 0) {
					msg_perr("Opening the USB device failed (%s)!\n",
						 libusb_error_name(err));
					libusb_free_device_list(list, 1);
					return NULL;
				}
				break;
			}
			num--;
		}
	}
	libusb_free_device_list(list, 1);

	return handle;
}

/* This function sets the GPIOs connected to the LEDs as well as IO1-IO4. */
static int dediprog_set_leds(int leds)
{
	if (leds < LED_NONE || leds > LED_ALL)
		leds = LED_ALL;

	/* Older Dediprogs with 2.x.x and 3.x.x firmware only had two LEDs, assigned to different bits. So map
	 * them around if we have an old device. On those devices the LEDs map as follows:
	 *   bit 2 == 0: green light is on.
	 *   bit 0 == 0: red light is on.
	 *
	 * Additionally, the command structure has changed with the "new" protocol.
	 *
	 * FIXME: take IO pins into account
	 */
	int target_leds, ret;
	if (is_new_prot()) {
		target_leds = (leds ^ 7) << 8;
		ret = dediprog_write_ep(CMD_SET_IO_LED, target_leds, 0, NULL, 0);
	} else {
		if (dediprog_firmwareversion < FIRMWARE_VERSION(5, 0, 0)) {
			target_leds = ((leds & LED_ERROR) >> 2) | ((leds & LED_PASS) << 2);
		} else {
			target_leds = leds;
		}
		target_leds ^= 7;

		ret = dediprog_write_ep(CMD_SET_IO_LED, 0x9, target_leds, NULL, 0);
	}

	if (ret != 0x0) {
		msg_perr("Command Set LED 0x%x failed (%s)!\n", leds, libusb_error_name(ret));
		return 1;
	}

	return 0;
}

struct dediprog_spispeeds {
	const char *const name;
	const int speed;
};

static const struct dediprog_spispeeds spispeeds[] = {
	{ "24M",	0x0 },
	{ "12M",	0x2 },
	{ "8M",		0x1 },
	{ "3M",		0x3 },
	{ "2.18M",	0x4 },
	{ "1.5M",	0x5 },
	{ "750k",	0x6 },
	{ "375k",	0x7 },
	{ NULL,		0x0 },
};

static int dediprog_set_spi_speed(unsigned int spispeed_idx)
{
	if (dediprog_firmwareversion < FIRMWARE_VERSION(5, 0, 0)) {
		msg_pinfo("Skipping to set SPI speed because firmware is too old.\n");
		return 0;
	}

	const struct dediprog_spispeeds *spispeed = &spispeeds[spispeed_idx];
	msg_pdbg("SPI speed is %sHz\n", spispeed->name);

	int ret = dediprog_write_ep(CMD_SET_SPI_CLK, spispeed->speed, 0, NULL, 0);
	if (ret != 0x0) {
		msg_perr("Command Set SPI Speed 0x%x failed!\n", spispeed->speed);
		return 1;
	}
	return 0;
}

/* Bulk read interface, will read multiple 512 byte chunks aligned to 512 bytes.
 * @start	start address
 * @len		length
 * @return	0 on success, 1 on failure
 */
static int dediprog_spi_bulk_read(struct flashchip *flash, uint8_t *buf,
				  unsigned int start, unsigned int len)
{
	int ret, err = 1;
	unsigned int i;
	/* chunksize must be 512, other sizes will NOT work at all. */
	const unsigned int chunksize = 0x200;
	const unsigned int count = len / chunksize;
	unsigned int cmd_len;
	struct dediprog_transfer_status status = { 0, 0, 0 };
	struct libusb_transfer *transfers[DEDIPROG_ASYNC_TRANSFERS] = { NULL, };
	struct libusb_transfer *transfer;

	if ((start % chunksize) || (len % chunksize)) {
		msg_perr("%s: Unaligned start=%i, len=%i! Please report a bug "
			 "at flashrom@flashrom.org\n", __func__, start, len);
		return 1;
	}

	/* No idea if the hardware can handle empty reads, so chicken out. */
	if (!len)
		return 0;
	/* Command Read SPI Bulk. */
	if (is_new_prot()) {
		const uint8_t read_cmd[] = {
			count & 0xff,
			(count >> 8) & 0xff,
			0,
			READ_MODE_FAST,
			0,
			0,
			start & 0xff,
			(start >> 8) & 0xff,
			(start >> 16) & 0xff,
			(start >> 24) & 0xff,
			};

		cmd_len = sizeof(read_cmd);
		ret = dediprog_write_ep(CMD_READ, 0, 0, read_cmd, cmd_len);
	} else {
		const uint8_t read_cmd[] = {count & 0xff,
					(count >> 8) & 0xff,
					chunksize & 0xff,
					(chunksize >> 8) & 0xff};

		cmd_len = sizeof(read_cmd);
		ret = dediprog_write_ep(CMD_READ, start % 0x10000, start / 0x10000, read_cmd, cmd_len);
	}
	if (ret != cmd_len) {
		msg_perr("Command Read SPI Bulk failed, %i %s!\n", ret, libusb_error_name(ret));
		return 1;
	}

	/*
	 * Ring buffer of bulk transfers.
	 * Poll until at least one transfer is ready,
	 * schedule next transfers until buffer is full.
	 */

	/* Allocate bulk transfers. */
	for (i = 0; i < min(DEDIPROG_ASYNC_TRANSFERS, count); ++i) {
		transfers[i] = libusb_alloc_transfer(0);
		if (!transfers[i]) {
			msg_perr("Allocating libusb transfer %i failed: %s!\n", i, libusb_error_name(ret));
			goto _err_free;
 		}
 	}

	/* Now transfer requested chunks using libusb's asynchronous interface. */
	while (!status.error && (status.queued_idx < count)) {
		while ((status.queued_idx - status.finished_idx) < DEDIPROG_ASYNC_TRANSFERS) {
			transfer = transfers[status.queued_idx % DEDIPROG_ASYNC_TRANSFERS];
			libusb_fill_bulk_transfer(transfer, dediprog_handle, 0x80 | dediprog_in_endpoint,
					(unsigned char *)buf + status.queued_idx * chunksize, chunksize,
					dediprog_bulk_read_cb, &status, DEFAULT_TIMEOUT);
			transfer->flags |= LIBUSB_TRANSFER_SHORT_NOT_OK;
			ret = libusb_submit_transfer(transfer);
			if (ret < 0) {
				msg_perr("Submitting SPI bulk read %i failed: %i %s!\n",
					 status.queued_idx, ret, libusb_error_name(ret));
				goto _err_free;
			}
			++status.queued_idx;
		}
		if (dediprog_bulk_read_poll(&status, 0))
			goto _err_free;
	}
	/* Wait for transfers to finish. */
	if (dediprog_bulk_read_poll(&status, 1))
		goto _err_free;
	/* Check if everything has been transmitted. */
	if ((status.finished_idx < count) || status.error)
		goto _err_free;

	err = 0;

_err_free:
	dediprog_bulk_read_poll(&status, 1);
	for (i = 0; i < DEDIPROG_ASYNC_TRANSFERS; ++i)
		if (transfers[i]) libusb_free_transfer(transfers[i]);
	return err;
}

static int dediprog_spi_read(struct flashchip *flash, uint8_t *buf,
			     unsigned int start, unsigned int len)
{
	int ret;
	/* chunksize must be 512, other sizes will NOT work at all. */
	const unsigned int chunksize = 0x200;
	unsigned int residue = start % chunksize ? chunksize - start % chunksize : 0;
	unsigned int bulklen;

	dediprog_set_leds(LED_BUSY);

	if (residue) {
		msg_pdbg("Slow read for partial block from 0x%x, length 0x%x\n",
			 start, residue);
		ret = spi_read_chunked(flash, buf, start, residue, 16);
		if (ret)
			goto err;
	}

	/* Round down. */
	bulklen = (len - residue) / chunksize * chunksize;
	ret = dediprog_spi_bulk_read(flash, buf + residue, start + residue,
				     bulklen);
	if (ret)
		goto err;

	len -= residue + bulklen;
	if (len) {
		msg_pdbg("Slow read for partial block from 0x%x, length 0x%x\n",
			 start, len);
		ret = spi_read_chunked(flash, buf + residue + bulklen,
				       start + residue + bulklen, len, 16);
		if (ret)
			goto err;
	}

	dediprog_set_leds(LED_PASS);
	return 0;
err:
	dediprog_set_leds(LED_ERROR);
	return ret;
}

/* Bulk write interface, will write multiple chunksize byte chunks aligned to chunksize bytes.
 * @chunksize       length of data chunks, only 256 supported by now
 * @start           start address
 * @len             length
 * @dedi_spi_cmd    dediprog specific write command for spi bus
 * @return          0 on success, 1 on failure
 */
static int dediprog_spi_bulk_write(struct flashchip *flash, const uint8_t *buf, unsigned int chunksize,
				   unsigned int start, unsigned int len, uint8_t dedi_spi_cmd)
{
	int ret, transferred;
	unsigned int i;
	/* USB transfer size must be 512, other sizes will NOT work at all.
	 * chunksize is the real data size per USB bulk transfer. The remaining
	 * space in a USB bulk transfer must be filled with 0xff padding.
	 */
	const unsigned int count = len / chunksize;
	const unsigned char count_and_cmd_old[] = {count & 0xff, (count >> 8) & 0xff, 0x00, dedi_spi_cmd};
	const unsigned char count_and_cmd_new[] = {
		count & 0xff,
		(count >> 8) & 0xff,
		0,			/* used for 24-bit address support? */
		dedi_spi_cmd,
		JEDEC_BYTE_PROGRAM,	/* FIXME: needs to be determined based on byte 3? */
		0,
		start & 0xff,
		(start >> 8) & 0xff,
		(start >> 16) & 0xff,
		(start >> 24) & 0xff,
	};
	unsigned char usbbuf[512];

	/*
	 * We should change this check to
	 *   chunksize > 512
	 * once we know how to handle different chunk sizes.
	 */
	if (chunksize != 256) {
		msg_perr("%s: Chunk sizes other than 256 bytes are unsupported, chunksize=%u!\n"
			 "Please report a bug at flashrom@flashrom.org\n", __func__, chunksize);
		return 1;
	}

	if ((start % chunksize) || (len % chunksize)) {
		msg_perr("%s: Unaligned start=%i, len=%i! Please report a bug "
			 "at flashrom@flashrom.org\n", __func__, start, len);
		return 1;
	}

	/* No idea if the hardware can handle empty writes, so chicken out. */
	if (!len)
		return 0;
	if (!is_new_prot()) {
		/* Command Write SPI Bulk. No idea which write command is used on the
		 * SPI side.
		 */
		ret = dediprog_write_ep(CMD_WRITE, start % 0x10000, start / 0x10000,
				      count_and_cmd_old, sizeof(count_and_cmd_old));
		if (ret != sizeof(count_and_cmd_old)) {
			msg_perr("Command Write SPI Bulk failed, %i %s!\n", ret,
				 libusb_error_name(ret));
			return 1;
		}
	} else {
		/* Command Write SPI Bulk. No idea which write command is used on the
		 * SPI side.
		 */
		ret = dediprog_write_ep(CMD_WRITE, 0, 0,
				      count_and_cmd_new, sizeof(count_and_cmd_new));
		if (ret != sizeof(count_and_cmd_new)) {
			msg_perr("Command Write SPI Bulk failed, %i %s!\n", ret,
				 libusb_error_name(ret));
			return 1;
		}
	}

	for (i = 0; i < count; i++) {
		memset(usbbuf, 0xff, sizeof(usbbuf));
		memcpy(usbbuf, buf + i * chunksize, chunksize);
		ret = libusb_bulk_transfer(dediprog_handle, dediprog_out_endpoint,
					   usbbuf, 512, &transferred, DEFAULT_TIMEOUT);
		if ((ret < 0) || (transferred != 512)) {
			msg_perr("SPI bulk write failed, expected %i, got %i %s!\n",
				 512, ret, libusb_error_name(ret));
			return 1;
		}
	}

	return 0;
}

static int dediprog_spi_write(struct flashchip *flash, const uint8_t *buf,
			      unsigned int start, unsigned int len, uint8_t dedi_spi_cmd)
{
	int ret;
	const unsigned int chunksize = flash->page_size;
	unsigned int residue = start % chunksize ? chunksize - start % chunksize : 0;
	unsigned int bulklen;

	dediprog_set_leds(LED_BUSY);

	if (chunksize != 256) {
		msg_pdbg("Page sizes other than 256 bytes are unsupported as "
			 "we don't know how dediprog\nhandles them.\n");
		/* Write everything like it was residue. */
		residue = len;
	}

	if (residue) {
		msg_pdbg("Slow write for partial block from 0x%x, length 0x%x\n",
			 start, residue);
		/* No idea about the real limit. Maybe 12, maybe more. */
		ret = spi_write_chunked(flash, (uint8_t *)buf, start, residue, 12);
		if (ret) {
			dediprog_set_leds(LED_ERROR);
			return ret;
		}
	}

	/* Round down. */
	bulklen = (len - residue) / chunksize * chunksize;
	ret = dediprog_spi_bulk_write(flash, buf + residue, chunksize, start + residue, bulklen, dedi_spi_cmd);
	if (ret) {
		dediprog_set_leds(LED_ERROR);
		return ret;
	}

	len -= residue + bulklen;
	if (len) {
		msg_pdbg("Slow write for partial block from 0x%x, length 0x%x\n",
			 start, len);
		ret = spi_write_chunked(flash, (uint8_t *)(buf + residue + bulklen),
				        start + residue + bulklen, len, 12);
		if (ret) {
			dediprog_set_leds(LED_ERROR);
			return ret;
		}
	}

	dediprog_set_leds(LED_PASS);
	return 0;
}

//static int dediprog_spi_write_256(struct flashchip *flash, const uint8_t *buf, unsigned int start, unsigned int len)
static int dediprog_spi_write_256(struct flashchip *flash, uint8_t *buf, unsigned int start, unsigned int len)
{
	return dediprog_spi_write(flash, buf, start, len, WRITE_MODE_PAGE_PGM);
}

#if 0
static int dediprog_spi_write_aai(struct flashchip *flash, const uint8_t *buf, unsigned int start, unsigned int len)
{
	return dediprog_spi_write(flash, buf, start, len, WRITE_MODE_2B_AAI);
}
#endif

//static int dediprog_spi_send_command(struct flashchip *flash,
static int dediprog_spi_send_command(unsigned int writecnt,
				     unsigned int readcnt,
				     const unsigned char *writearr,
				     unsigned char *readarr)
{
	int ret;

	msg_pspew("%s, writecnt=%i, readcnt=%i\n", __func__, writecnt, readcnt);
	if (writecnt > UINT16_MAX) {
		msg_perr("Invalid writecnt=%i, aborting.\n", writecnt);
		return 1;
	}
	if (readcnt > UINT16_MAX) {
		msg_perr("Invalid readcnt=%i, aborting.\n", readcnt);
		return 1;
	}
	
	/* New protocol has the read flag as value while the old protocol had it in the index field. */
	if (is_new_prot()) {
		ret = dediprog_write_ep(CMD_TRANSCEIVE, readcnt ? 1 : 0, 0, writearr, writecnt);
	} else {
		ret = dediprog_write_ep(CMD_TRANSCEIVE, 0, readcnt ? 1 : 0, writearr, writecnt);
	}
	if (ret != writecnt) {
		msg_perr("Send SPI failed, expected %i, got %i %s!\n",
			 writecnt, ret, libusb_error_name(ret));
		return 1;
	}
	if (readcnt == 0)
		return 0;

	ret = dediprog_read_ep(CMD_TRANSCEIVE, 0, 0, readarr, readcnt);
	if (ret != readcnt) {
		msg_perr("Receive SPI failed, expected %i, got %i %s!\n",
			 readcnt, ret, libusb_error_name(ret));
		return 1;
	}
	return 0;
}

static int dediprog_check_devicestring(void)
{
	int ret;
	int fw[3];
	int sfnum;
	char buf[0x11];

	/* Command Receive Device String. */
	ret = dediprog_read_ep(CMD_READ_PROG_INFO, 0, 0, (uint8_t *)buf, 0x10);
	if (ret != 0x10) {
		msg_perr("Incomplete/failed Command Receive Device String!\n");
		return 1;
	}
	buf[0x10] = '\0';
	msg_pdbg("Found a %s\n", buf);
	if (memcmp(buf, "SF100", 0x5) == 0)
		dediprog_devicetype = DEV_SF100;
	else if (memcmp(buf, "SF600", 0x5) == 0)
		dediprog_devicetype = DEV_SF600;
	else {
		msg_perr("Device not a SF100 or SF600!\n");
		return 1;
	}
	if (sscanf(buf, "SF%d V:%d.%d.%d ", &sfnum, &fw[0], &fw[1], &fw[2])
	    != 4 || sfnum != dediprog_devicetype) {
		msg_perr("Unexpected firmware version string '%s'\n", buf);
		return 1;
	}
	/* Only these major versions were tested. */
	if (fw[0] < 2 || fw[0] > 7) {
		msg_perr("Unexpected firmware version %d.%d.%d!\n", fw[0], fw[1], fw[2]);
		return 1;
	}
	dediprog_firmwareversion = FIRMWARE_VERSION(fw[0], fw[1], fw[2]);

	return 0;
}

static int dediprog_supply_voltages[] = {
	0, 1800, 2500, 3500,
};

static int dediprog_set_spi_flash_voltage_manual(int millivolt)
{
	int ret;
	uint16_t voltage_selector;

	switch (millivolt) {
	case 0:
		/* Admittedly this one is an assumption. */
		voltage_selector = 0x0;
		break;
	case 1800:
		voltage_selector = 0x12;
		break;
	case 2500:
		voltage_selector = 0x11;
		break;
	case 3500:
		voltage_selector = 0x10;
		break;
	default:
		msg_perr("Unknown voltage %i mV! Aborting.\n", millivolt);
		return 1;
	}
	msg_pdbg("Setting SPI voltage to %u.%03u V\n", millivolt / 1000,
		 millivolt % 1000);

	if (voltage_selector == 0) {
		/* Wait some time as the original driver does. */
		programmer_delay(200 * 1000);
	}
	ret = dediprog_write_ep(CMD_SET_VCC, voltage_selector, 0, NULL, 0);
	if (ret != 0x0) {
		msg_perr("Command Set SPI Voltage 0x%x failed!\n",
			 voltage_selector);
		return 1;
	}
	if (voltage_selector != 0) {
		/* Wait some time as the original driver does. */
		programmer_delay(200 * 1000);
	}
	return 0;
}

static int dediprog_set_spi_flash_voltage_auto(void)
{
	int i;
	int spi_flash_ranges;

	spi_flash_ranges = flash_supported_voltage_ranges(BUS_SPI);
	if (spi_flash_ranges < 0)
		return -1;

	for (i = 0; i < ARRAY_SIZE(dediprog_supply_voltages); i++) {
		int j;
		int v = dediprog_supply_voltages[i];	/* shorthand */

		for (j = 0; j < spi_flash_ranges; j++) {
			/* Dediprog can supply a voltage in this range. */
			if ((v >= voltage_ranges[j].min) && (v <= voltage_ranges[j].max)) {
				struct flashchip dummy;

				msg_cdbg("%s: trying %d\n", __func__, v);
				if (dediprog_set_spi_flash_voltage_manual(v)) {
					msg_cerr("%s: Failed to set SPI voltage\n", __func__);
					return 1;
				}

				clear_spi_id_cache();
				if (probe_flash(0, &dummy, 0) < 0) {
					/* No dice, try next voltage supported by Dediprog. */
					break;
				}

				if ((dummy.manufacture_id == GENERIC_MANUF_ID) ||
					(dummy.model_id == GENERIC_DEVICE_ID)) {
					break;
				}

				return 0;
			}
		}
	}

	return 1;
}

/* FIXME: ugly function signature */
static int dediprog_set_spi_voltage(int millivolt, int probe)
{
	if (probe)
		return dediprog_set_spi_flash_voltage_auto();
	else
		return dediprog_set_spi_flash_voltage_manual(millivolt);
}

/*
 * This command presumably sets the voltage for the SF100 itself (not the
 * SPI flash). Only use this command with firmware older than V6.0.0. Newer
 * (including all SF600s) do not support it.
 */
static int dediprog_set_voltage(void)
{
	int ret;
	unsigned char buf[0x1];

	memset(buf, 0, sizeof(buf));
	ret = dediprog_read_other(CMD_SET_VOLTAGE, 0x0, 0x0, buf, 0x1);
	if (ret < 0) {
		msg_perr("Command A failed (%s)!\n", libusb_error_name(ret));
		return 1;
	}
	if ((ret != 0x1) || (buf[0] != 0x6f)) {
		msg_perr("Unexpected response to init!\n");
		return 1;
	}
	return 0;
}

static int dediprog_leave_standalone_mode(void)
{
	int ret;

	if (dediprog_devicetype != DEV_SF600)
		return 0;

	msg_pdbg("Leaving standalone mode\n");
	ret = dediprog_write_ep(CMD_SET_STANDALONE, LEAVE_STANDALONE_MODE, 0, NULL, 0);
	if (ret) {
		msg_perr("Failed to leave standalone mode (%s)!\n", libusb_error_name(ret));
		return 1;
	}

	return 0;
}

#if 0
/* Something.
 * Present in eng_detect_blink.log with firmware 3.1.8
 * Always preceded by Command Receive Device String
 */
static int dediprog_command_b(void)
{
	int ret;
	char buf[0x3];

	ret = usb_control_msg(dediprog_handle, REQTYPE_OTHER_IN, 0x7, 0x0, 0xef00,
			      buf, 0x3, DEFAULT_TIMEOUT);
	if (ret < 0) {
		msg_perr("Command B failed (%s)!\n", usb_strerror());
		return 1;
	}
	if ((ret != 0x3) || (buf[0] != 0xff) || (buf[1] != 0xff) ||
	    (buf[2] != 0xff)) {
		msg_perr("Unexpected response to Command B!\n");
		return 1;
	}

	return 0;
}
#endif

static int set_target_flash(enum dediprog_target target)
{
	int ret = dediprog_write_ep(CMD_SET_TARGET, target, 0, NULL, 0);
	if (ret != 0) {
		msg_perr("set_target_flash failed (%s)!\n", libusb_error_name(ret));
		return 1;
	}
	return 0;
}

#if 0
/* Returns true if the button is currently pressed. */
static bool dediprog_get_button(void)
{
	char buf[1];
	int ret = usb_control_msg(dediprog_handle, REQTYPE_EP_IN, CMD_GET_BUTTON, 0, 0,
			      buf, 0x1, DEFAULT_TIMEOUT);
	if (ret != 0) {
		msg_perr("Could not get button state (%s)!\n", usb_strerror());
		return 1;
	}
	return buf[0] != 1;
}
#endif

static int parse_voltage(char *voltage)
{
	char *tmp = NULL;
	int i;
	int millivolt = 0, fraction = 0;

	if (!voltage || !strlen(voltage)) {
		msg_perr("Empty voltage= specified.\n");
		return -1;
	}
	millivolt = (int)strtol(voltage, &tmp, 0);
	voltage = tmp;
	/* Handle "," and "." as decimal point. Everything after it is assumed
	 * to be in decimal notation.
	 */
	if ((*voltage == '.') || (*voltage == ',')) {
		voltage++;
		for (i = 0; i < 3; i++) {
			fraction *= 10;
			/* Don't advance if the current character is invalid,
			 * but continue multiplying.
			 */
			if ((*voltage < '0') || (*voltage > '9'))
				continue;
			fraction += *voltage - '0';
			voltage++;
		}
		/* Throw away remaining digits. */
		voltage += strspn(voltage, "0123456789");
	}
	/* The remaining string must be empty or "mV" or "V". */
	tolower_string(voltage);

	/* No unit or "V". */
	if ((*voltage == '\0') || !strncmp(voltage, "v", 1)) {
		millivolt *= 1000;
		millivolt += fraction;
	} else if (!strncmp(voltage, "mv", 2) ||
		   !strncmp(voltage, "milliv", 6)) {
		/* No adjustment. fraction is discarded. */
	} else {
		/* Garbage at the end of the string. */
		msg_perr("Garbage voltage= specified.\n");
		return -1;
	}
	return millivolt;
}

#if 0
static const struct spi_master spi_master_dediprog = {
	.type		= SPI_CONTROLLER_DEDIPROG,
	.max_data_read	= MAX_DATA_UNSPECIFIED,
	.max_data_write	= MAX_DATA_UNSPECIFIED,
	.command	= dediprog_spi_send_command,
	.multicommand	= default_spi_send_multicommand,
	.read		= dediprog_spi_read,
	.write_256	= dediprog_spi_write_256,
	.write_aai	= dediprog_spi_write_aai,
};
#endif
static const struct spi_programmer spi_programmer_dediprog = {
	.type		= SPI_CONTROLLER_DEDIPROG,
	.max_data_read	= MAX_DATA_UNSPECIFIED,
	.max_data_write	= MAX_DATA_UNSPECIFIED,
	.command	= dediprog_spi_send_command,
	.multicommand	= default_spi_send_multicommand,
	.read		= dediprog_spi_read,
	.write_256	= dediprog_spi_write_256,
};

static int dediprog_shutdown(void *data)
{
	msg_pspew("%s\n", __func__);

	dediprog_firmwareversion = FIRMWARE_VERSION(0, 0, 0);
	dediprog_devicetype = DEV_UNKNOWN;

	/* URB 28. Command Set SPI Voltage to 0. */
	if (dediprog_set_spi_voltage(0x0, 0))
		return 1;

	if (libusb_release_interface(dediprog_handle, 0)) {
		msg_perr("Could not release USB interface!\n");
		return 1;
	}
	libusb_close(dediprog_handle);
	libusb_exit(usb_ctx);

	return 0;
}

/* URB numbers refer to the first log ever captured. */
int dediprog_init(void)
{
	char *voltage, *device, *spispeed, *target_str;
	int spispeed_idx = 1;
	int millivolt = 0;
	long usedevice = 0;
	long target = 1;
	int i, ret;

	msg_pspew("%s\n", __func__);

	spispeed = extract_programmer_param("spispeed");
	if (spispeed) {
		for (i = 0; spispeeds[i].name; ++i) {
			if (!strcasecmp(spispeeds[i].name, spispeed)) {
				spispeed_idx = i;
				break;
			}
		}
		if (!spispeeds[i].name) {
			msg_perr("Error: Invalid spispeed value: '%s'.\n", spispeed);
			free(spispeed);
			return 1;
		}
		free(spispeed);
	}

	voltage = extract_programmer_param("voltage");
	if (voltage) {
		millivolt = parse_voltage(voltage);
		free(voltage);
		if (millivolt < 0)
			return 1;
		msg_pinfo("Setting voltage to %i mV\n", millivolt);
	}

	device = extract_programmer_param("device");
	if (device) {
		char *dev_suffix;
		errno = 0;
		usedevice = strtol(device, &dev_suffix, 10);
		if (errno != 0 || device == dev_suffix) {
			msg_perr("Error: Could not convert 'device'.\n");
			free(device);
			return 1;
		}
		if (usedevice < 0 || usedevice > UINT_MAX) {
			msg_perr("Error: Value for 'device' is out of range.\n");
			free(device);
			return 1;
		}
		if (strlen(dev_suffix) > 0) {
			msg_perr("Error: Garbage following 'device' value.\n");
			free(device);
			return 1;
		}
		msg_pinfo("Using device %li.\n", usedevice);
	}
	free(device);

	target_str = extract_programmer_param("target");
	if (target_str) {
		char *target_suffix;
		errno = 0;
		target = strtol(target_str, &target_suffix, 10);
		if (errno != 0 || target_str == target_suffix) {
			msg_perr("Error: Could not convert 'target'.\n");
			free(target_str);
			return 1;
		}
		if (target < 1 || target > 2) {
			msg_perr("Error: Value for 'target' is out of range.\n");
			free(target_str);
			return 1;
		}
		if (strlen(target_suffix) > 0) {
			msg_perr("Error: Garbage following 'target' value.\n");
			free(target_str);
			return 1;
		}
		msg_pinfo("Using target %li.\n", target);
	}
	free(target_str);

	/* Here comes the USB stuff. */
	libusb_init(&usb_ctx);
	if (!usb_ctx) {
		msg_perr("Could not initialize libusb!\n");
		return 1;
	}
	dediprog_handle = get_device_by_vid_pid_number(0x0483, 0xdada, (unsigned int) usedevice);
	if (!dediprog_handle) {
		msg_perr("Could not find a Dediprog programmer on USB!\n");
		libusb_exit(usb_ctx);
		return 1;
	}
	ret = libusb_set_configuration(dediprog_handle, 1);
	if (ret != 0) {
		msg_perr("Could not set USB device configuration: %i %s\n", ret, libusb_error_name(ret));
		libusb_close(dediprog_handle);
		libusb_exit(usb_ctx);
		return 1;
	}
	ret = libusb_claim_interface(dediprog_handle, 0);
	if (ret < 0) {
		msg_perr("Could not claim USB device interface %i: %i %s\n", 0, ret, libusb_error_name(ret));
		libusb_close(dediprog_handle);
		libusb_exit(usb_ctx);
		return 1;
	}

	if (register_shutdown(dediprog_shutdown, NULL))
		return 1;

	/* Try reading the devicestring. If that fails and the device is old
	 * (FW < 6.0.0) then we need to try the "set voltage" command and then
	 * attempt to read the devicestring again. */
	if (dediprog_check_devicestring()) {
		if (dediprog_set_voltage())
			return 1;
		if (dediprog_check_devicestring())
			return 1;
	}

	/* SF100 firmware exposes only one endpoint for in/out, SF600 firmware
           exposes separate endpoints for in and out. */
	dediprog_in_endpoint = 2;
	if (dediprog_devicetype == DEV_SF100)
		dediprog_out_endpoint = 2;
	else
		dediprog_out_endpoint = 1;


	/* Set all possible LEDs as soon as possible to indicate activity.
	 * Because knowing the firmware version is required to set the LEDs correctly we need to this after
	 * dediprog_setup() has queried the device and set dediprog_firmwareversion. */
	dediprog_set_leds(LED_ALL);

	/* FIXME: need to do this so buses_supported gets SPI */
	register_spi_programmer(&spi_programmer_dediprog);

	/* Select target/socket, frequency and VCC. */
	if (set_target_flash(FLASH_TYPE_APPLICATION_FLASH_1) ||
	    dediprog_set_spi_speed(spispeed_idx) ||
	    dediprog_set_spi_voltage(millivolt, voltage ? 0 : 1)) {
		dediprog_set_leds(LED_ERROR);
		return 1;
	}

	if (dediprog_leave_standalone_mode())
		return 1;


	dediprog_set_leds(LED_NONE);

	return 0;
}

