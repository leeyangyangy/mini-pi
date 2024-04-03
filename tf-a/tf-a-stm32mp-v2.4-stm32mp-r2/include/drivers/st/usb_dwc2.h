/*
 * Copyright (c) 2015-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef USB_DWC2_H
#define USB_DWC2_H

#include <lib/usb/usb_core.h>

#define USB_MAX_ENDPOINT_NB			0x10

void usb_dwc2_init_driver(usb_handle_t *usb_core_handle,
			  pcd_handle_t *pcd_handle,
			  void *base_register);

#endif /* USB_DWC2_H */

