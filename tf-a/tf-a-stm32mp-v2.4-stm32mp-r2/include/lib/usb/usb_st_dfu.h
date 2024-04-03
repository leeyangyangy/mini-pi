/*
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef USB_ST_DFU_H
#define USB_ST_DFU_H

#include <stdint.h>

#include <lib/usb/usb_core.h>

#define DFU_DESCRIPTOR_TYPE		0x21

/* Max DFU Packet Size = 1024 bytes */
#define USBD_DFU_XFER_SIZE		1024

#define TRANSFER_SIZE_BYTES(size) \
	((uint8_t)((size) & 0xFF)), /* XFERSIZEB0 */\
	((uint8_t)((size) >> 8))    /* XFERSIZEB1 */

/* Descriptor of DFU interface 0 Alternate setting n */
#define USBD_DFU_IF_DESC(n)	0x09, /* Interface Descriptor size */\
				USB_DESC_TYPE_INTERFACE, /* descriptor type */\
				0x00, /* Number of Interface */\
				(n), /* Alternate setting */\
				0x00, /* bNumEndpoints*/\
				0xFE, /* Application Specific Class Code */\
				0x01, /* Device Firmware Upgrade Code */\
				0x02, /* DFU mode protocol */ \
				USBD_IDX_USER0_STR + (n) /* iInterface:
							  * Index of string
							  * descriptor
							  */

/* DFU1.1 Standard */
#define USB_DFU_VERSION			0x0110
#define USB_DFU_ITF_SIZ			9
#define USB_DFU_DESC_SIZ(itf)		(USB_DFU_ITF_SIZ * ((itf) + 2))

/* bmAttribute :
 * bitCanDnload = 1(bit 0)
 * bitCanUpload = 1(bit 1)
 * bitManifestationTolerant = 1 (bit 2)
 * bitWillDetach = 1(bit 3)
 * Reserved (bit4-6)
 * bitAcceleratedST = 0(bit 7)
 */
#define DFU_BM_ATTRIBUTE		0x0F

#define DFU_MEDIA_STATE_READY		0x00
#define DFU_MEDIA_STATE_WRITTEN		0x01
#define DFU_MEDIA_STATE_ERROR		0x02

#define DFU_STATUS_SIZE			6U

typedef void (*p_function)(void);

/* Callback for media access */
typedef struct {
	int (*upload)(uint8_t alt, uintptr_t *buffer, uint32_t *len,
		      void *user_data);
	int (*download)(uint8_t alt, uintptr_t *buffer, uint32_t *len,
			void *user_data);
	int (*manifestation)(uint8_t alt, void *user_data);
} usb_dfu_media_t;

/* Internal DFU handle */
typedef struct {
	uint8_t status[DFU_STATUS_SIZE];
	uint8_t dev_state;
	uint8_t dev_status;
	uint32_t alt_setting;
	const usb_dfu_media_t *callback;
} usb_dfu_handle_t;

void usb_dfu_register(usb_handle_t *pdev, usb_dfu_handle_t *phandle);

int usb_dfu_loop(usb_handle_t *pdev, const usb_dfu_media_t *pmedia);

/* Function provided by plat */
usb_handle_t *usb_dfu_plat_init(void);

#endif /* USB_ST_DFU_H */
