/*
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>

#include <lib/usb/usb_st_dfu.h>
#include <tools_share/firmware_image_package.h>

#include <stm32cubeprogrammer.h>

/* Undefined download address */
#define UNDEFINED_DOWN_ADDR	0xFFFFFFFF

#define USB_STATE_READY		0
#define USB_STATE_WRITTEN	1

#define USB_DFU_MAX_XFER_SIZE	USBD_DFU_XFER_SIZE

typedef struct {
	unsigned int image_id;
	uint8_t phase;
	uintptr_t base;
	size_t len;
	uintptr_t address;
	/* parameter */
	uintptr_t ssbl_base;
	size_t ssbl_len;
#if STM32MP_SSP
	uintptr_t cert_base;
	size_t cert_len;
#endif
	/* working buffer */
	uint8_t buffer[255];
} dfu_state_t;

static dfu_state_t dfu_state;

#define DFU_ERROR(...) \
	{ \
		ERROR(__VA_ARGS__); \
		if (dfu->phase != PHASE_RESET) { \
			snprintf((char *)&dfu->buffer[9], \
				 sizeof(dfu->buffer) - 9, __VA_ARGS__); \
			dfu->phase = PHASE_RESET; \
			dfu->address = UNDEFINED_DOWN_ADDR; \
			dfu->len = 0; \
		} \
	}

static inline bool is_valid_header(fip_toc_header_t *header)
{
	if ((header->name == TOC_HEADER_NAME) && (header->serial_number != 0U)) {
		return true;
	}

	return false;
}

static int dfu_callback_upload(uint8_t alt, uintptr_t *buffer, uint32_t *len,
			       void *user_data)
{
	int result = 0;
	uint32_t length = 0;
	dfu_state_t *dfu = (dfu_state_t *)user_data;

	switch (usb_dfu_get_phase(alt)) {
	case PHASE_CMD:
		/* Get Pḧase */
#if STM32MP_SSP
		if (dfu->phase == PHASE_SSP) {
			dfu->buffer[0] = PHASE_FLASHLAYOUT;
		} else {
			dfu->buffer[0] = dfu->phase;
		}
#else
		dfu->buffer[0] = dfu->phase;
#endif
		dfu->buffer[1] = (uint8_t)(dfu->address);
		dfu->buffer[2] = (uint8_t)(dfu->address >> 8);
		dfu->buffer[3] = (uint8_t)(dfu->address >> 16);
		dfu->buffer[4] = (uint8_t)(dfu->address >> 24);
		dfu->buffer[5] = 0x00;
		dfu->buffer[6] = 0x00;
		dfu->buffer[7] = 0x00;
		dfu->buffer[8] = 0x00;
		length = 9;
		if (dfu->phase == PHASE_FLASHLAYOUT &&
		    dfu->address == UNDEFINED_DOWN_ADDR) {
			INFO("Send detach request\n");
			dfu->buffer[9] = 0x01;
			length = 10;
		}
		if (dfu->phase == PHASE_RESET) {
			length = 8 + strnlen((char *)&dfu->buffer[9],
					     sizeof(dfu->buffer) - 9);
		}
		break;

#if STM32MP_SSP
	case PHASE_SSP:
		/* Fix phase to flashlayout phase */
		dfu->buffer[0] = PHASE_FLASHLAYOUT;
		dfu->buffer[1] = (uint8_t)(dfu_state.cert_base);
		dfu->buffer[2] = (uint8_t)(dfu_state.cert_base >> 8);
		dfu->buffer[3] = (uint8_t)(dfu_state.cert_base >> 16);
		dfu->buffer[4] = (uint8_t)(dfu_state.cert_base >> 24);
		dfu->buffer[5] = 0x00;
		dfu->buffer[6] = 0x00;
		dfu->buffer[7] = 0x00;
		dfu->buffer[8] = 0x00;
		length = 9U;

		if ((length + dfu_state.cert_len) <= sizeof(dfu->buffer)) {
			memcpy(&dfu->buffer[9], (uint8_t *)dfu_state.cert_base,
			       dfu_state.cert_len);
			length += dfu_state.cert_len;
		}

		break;
#endif
	default:
		DFU_ERROR("phase ID :%i, alternate %i for phase %i\n",
			  dfu->phase, alt, usb_dfu_get_phase(alt));
		result = -EIO;
		break;
	}

	if (result == 0) {
		*len = length;
		*buffer = (uintptr_t)dfu->buffer;
	}

	return result;
}

static int dfu_callback_download(uint8_t alt, uintptr_t *buffer, uint32_t *len,
				 void *user_data)
{
	dfu_state_t *dfu = (dfu_state_t *)user_data;

	if ((dfu->phase != usb_dfu_get_phase(alt)) ||
	    (dfu->address == UNDEFINED_DOWN_ADDR)) {
		DFU_ERROR("phase ID :%i, alternate %i, address %x\n",
			  dfu->phase, alt, (uint32_t)dfu->address);
		return -EIO;
	}

	VERBOSE("Download %d %lx %x\n", alt, dfu->address, *len);
	*buffer = dfu->address;
	dfu->address += *len;

	if (dfu->address - dfu->base > dfu->len) {
		return  -EIO;
	}

	return 0;
}

static int dfu_callback_manifestation(uint8_t alt, void *user_data)
{
#if STM32MP_USE_STM32IMAGE
	int result;
#endif
	boot_api_image_header_t *header __unused;
	dfu_state_t *dfu = (dfu_state_t *)user_data;

	if (dfu->phase != usb_dfu_get_phase(alt)) {
		ERROR("Manifestation phase ID :%i, alternate %i, address %lx\n",
		      dfu->phase, alt, dfu->address);
		return -EIO;
	}

	INFO("phase ID :%i, Manifestation %d at %lx\n",
	     dfu->phase, alt, dfu->address);
	switch (dfu->phase) {
#if STM32MP_SSP
	case PHASE_SSP:
		/* Configure End with request detach */
		dfu->phase = PHASE_FLASHLAYOUT;
		dfu->address = UNDEFINED_DOWN_ADDR;
		dfu->len = 0;
		break;
#else
	case PHASE_FLASHLAYOUT:
		header = (boot_api_image_header_t *)(dfu->base);

		/* TODO check data flush */
		flush_dcache_range((unsigned long)header,
				   header->image_length +
				   sizeof(boot_api_image_header_t));

#if STM32MP_USE_STM32IMAGE
		/* Verify header and checksum payload */
		INFO("Flashlayout Header check at %lx\n",
		     (uintptr_t)header);
		result = stm32mp_check_header(header,
					      (unsigned long)header +
					      sizeof(boot_api_image_header_t));
		if (result != 0) {
			DFU_ERROR("Header check failed for phase %d\n", alt);
			return -EIO;
		}
#endif
		/* Configure U-Boot loading */
		dfu->phase = PHASE_SSBL;
		dfu->address = dfu->ssbl_base;
		dfu->base = dfu->ssbl_base;
		dfu->len = dfu->ssbl_len;
		break;

	case PHASE_SSBL:
#if !STM32MP_USE_STM32IMAGE
		if (dfu->image_id == FIP_IMAGE_ID) {
			if (!is_valid_header((fip_toc_header_t *)dfu->base)) {
				DFU_ERROR("FIP Header check failed for phase %d\n", alt);
				return -EIO;
			}

			VERBOSE("FIP header looks OK.\n");
		}
#else
		if (dfu->image_id == STM32_IMAGE_ID) {
			header = (boot_api_image_header_t *)dfu->base;
			/* Verify header and checksum payload */
			result = stm32mp_check_header(header,
						      dfu->base +
						      sizeof(boot_api_image_header_t));
			if (result != 0) {
				DFU_ERROR("STM32 Header check failed for phase %d\n", alt);
				return -EIO;
			}

			VERBOSE("STM32 header looks OK.\n");
		}
#endif
		/* Configure End with request detach */
		dfu->phase = PHASE_FLASHLAYOUT;
		dfu->address = UNDEFINED_DOWN_ADDR;
		dfu->len = 0;
		break;
#endif /* STM32MP_SSP */
	default:
		DFU_ERROR("Unknown phase\n");
	}

	return 0;
}

/* Open a connection to the USB device */
static const usb_dfu_media_t usb_dfu_fops = {
	.upload = dfu_callback_upload,
	.download = dfu_callback_download,
	.manifestation = dfu_callback_manifestation,
};

#if STM32MP_SSP
int stm32cubeprog_usb_ssp(usb_handle_t *usb_core_handle,
			  uintptr_t cert_base,
			  size_t cert_len,
			  uintptr_t ssp_base,
			  size_t ssp_len)
{
	int ret;

	usb_core_handle->user_data = (void *)&dfu_state;

	INFO("DFU USB START...\n");
	ret = usb_core_start(usb_core_handle);
	if (ret != USBD_OK) {
		return -EIO;
	}

	if (cert_base == UNDEFINED_DOWN_ADDR) {
		dfu_state_t *dfu = (dfu_state_t *)usb_core_handle->user_data;

		/* Send Provisioning message to programmer for reboot */
		DFU_ERROR("Provisioning\n");
	} else {
		dfu_state.phase = PHASE_SSP;
		dfu_state.image_id = MAX_IMAGE_IDS;
		dfu_state.address = ssp_base;
		dfu_state.base = ssp_base;
		dfu_state.len = ssp_len;
		dfu_state.cert_base = cert_base;
		dfu_state.cert_len = cert_len;
	}

	ret = usb_dfu_loop(usb_core_handle, &usb_dfu_fops);
	if (ret != USBD_OK) {
		return -EIO;
	}

	INFO("DFU USB STOP...\n");
	ret = usb_core_stop(usb_core_handle);
	if (ret != USBD_OK) {
		return -EIO;
	}

	return 0;
}
#endif

int stm32cubeprog_usb_load(unsigned int image_id,
			   usb_handle_t *usb_core_handle,
			   uintptr_t flashlayout_base,
			   size_t flashlayout_len,
			   uintptr_t ssbl_base,
			   size_t ssbl_len)
{
	int ret;

	usb_core_handle->user_data = (void *)&dfu_state;

	INFO("DFU USB START...\n");
	ret = usb_core_start(usb_core_handle);
	if (ret != USBD_OK) {
		return -EIO;
	}

	dfu_state.image_id = image_id;
	dfu_state.ssbl_base = ssbl_base;
	dfu_state.ssbl_len = ssbl_len;

	if (flashlayout_len) {
		dfu_state.phase = PHASE_FLASHLAYOUT;
		dfu_state.address = flashlayout_base;
		dfu_state.base = flashlayout_base;
		dfu_state.len = flashlayout_len;
	} else {
		dfu_state.phase = PHASE_SSBL;
		dfu_state.address = ssbl_base;
		dfu_state.base = ssbl_base;
		dfu_state.len = ssbl_len;
	}

	ret = usb_dfu_loop(usb_core_handle, &usb_dfu_fops);
	if (ret != USBD_OK) {
		return -EIO;
	}

	INFO("DFU USB STOP...\n");
	ret = usb_core_stop(usb_core_handle);
	if (ret != USBD_OK) {
		return -EIO;
	}

	return 0;
}
