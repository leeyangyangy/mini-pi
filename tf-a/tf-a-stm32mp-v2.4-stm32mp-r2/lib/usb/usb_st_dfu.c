/*
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <string.h>

#include <platform_def.h>

#include <common/debug.h>
#include <lib/usb/usb_st_dfu.h>

/* DFU Requests  DFU states */
#define APP_STATE_IDLE			0
#define APP_STATE_DETACH		1
#define DFU_STATE_IDLE			2
#define DFU_STATE_DNLOAD_SYNC		3
#define DFU_STATE_DNLOAD_BUSY		4
#define DFU_STATE_DNLOAD_IDLE		5
#define DFU_STATE_MANIFEST_SYNC		6
#define DFU_STATE_MANIFEST		7
#define DFU_STATE_MANIFEST_WAIT_RESET	8
#define DFU_STATE_UPLOAD_IDLE		9
#define DFU_STATE_ERROR			10

/* DFU errors */
#define DFU_ERROR_NONE			0x00
#define DFU_ERROR_TARGET		0x01
#define DFU_ERROR_FILE			0x02
#define DFU_ERROR_WRITE			0x03
#define DFU_ERROR_ERASE			0x04
#define DFU_ERROR_CHECK_ERASED		0x05
#define DFU_ERROR_PROG			0x06
#define DFU_ERROR_VERIFY		0x07
#define DFU_ERROR_ADDRESS		0x08
#define DFU_ERROR_NOTDONE		0x09
#define DFU_ERROR_FIRMWARE		0x0A
#define DFU_ERROR_VENDOR		0x0B
#define DFU_ERROR_USB			0x0C
#define DFU_ERROR_POR			0x0D
#define DFU_ERROR_UNKNOWN		0x0E
#define DFU_ERROR_STALLEDPKT		0x0F

typedef enum {
	DFU_DETACH = 0,
	DFU_DNLOAD,
	DFU_UPLOAD,
	DFU_GETSTATUS,
	DFU_CLRSTATUS,
	DFU_GETSTATE,
	DFU_ABORT
} dfu_request_t;

static bool usb_dfu_detach_req;

/*
 * @brief  usb_dfu_init
 *         Initialize the DFU interface
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t usb_dfu_init(usb_handle_t *pdev, uint8_t cfgidx)
{
	/* Nothing to do in this stage */
	return USBD_OK;
}

/**
 * @brief  usb_dfu_de_init
 *         De-Initialize the DFU layer
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t usb_dfu_de_init(usb_handle_t *pdev, uint8_t cfgidx)
{
	/* Nothing to do in this stage */
	return USBD_OK;
}

/*
 * @brief  usb_dfu_data_in
 *         handle data IN Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t  usb_dfu_data_in(usb_handle_t *pdev, uint8_t epnum)
{
	(void)pdev;
	(void)epnum;

	return USBD_OK;
}

/*
 * @brief  usb_dfu_ep0_rx_ready
 *         handle EP0 Rx Ready event
 * @param  pdev: device instance
 * @retval status
 */
static uint8_t usb_dfu_ep0_rx_ready(usb_handle_t *pdev)
{
	(void)pdev;

	return USBD_OK;
}

/*
 * @brief  usb_dfu_ep0_tx_ready
 *         handle EP0 TRx Ready event
 * @param  pdev: device instance
 * @retval status
 */
static uint8_t usb_dfu_ep0_tx_ready(usb_handle_t *pdev)
{
	return USBD_OK;
}

/*
 * @brief  usb_dfu_sof
 *         handle SOF event
 * @param  pdev: device instance
 * @retval status
 */
static uint8_t usb_dfu_sof(usb_handle_t *pdev)
{
	(void)pdev;

	return USBD_OK;
}

/*
 * @brief  usb_dfu_iso_in_incomplete
 *         handle data ISO IN Incomplete event
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t usb_dfu_iso_in_incomplete(usb_handle_t *pdev, uint8_t epnum)
{
	(void)pdev;
	(void)epnum;

	return USBD_OK;
}

/*
 * @brief  usb_dfu_iso_out_incomplete
 *         handle data ISO OUT Incomplete event
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t usb_dfu_iso_out_incomplete(usb_handle_t *pdev, uint8_t epnum)
{
	(void)pdev;
	(void)epnum;

	return USBD_OK;
}

/*
 * @brief  usb_dfu_data_out
 *         handle data OUT Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t usb_dfu_data_out(usb_handle_t *pdev, uint8_t epnum)
{
	(void)pdev;
	(void)epnum;

	return USBD_OK;
}

/*
 * @brief  usb_dfu_detach
 *         Handles the DFU DETACH request.
 * @param  pdev: device instance
 * @param  req: pointer to the request structure.
 * @retval None.
 */
static void usb_dfu_detach(usb_handle_t *pdev, usb_setup_req_t *req)
{
	usb_dfu_handle_t *hdfu = (usb_dfu_handle_t *)pdev->class_data;

	INFO("Receive DFU Detach\n");

	if ((hdfu->dev_state == DFU_STATE_IDLE) ||
	    (hdfu->dev_state == DFU_STATE_DNLOAD_SYNC) ||
	    (hdfu->dev_state == DFU_STATE_DNLOAD_IDLE) ||
	    (hdfu->dev_state == DFU_STATE_MANIFEST_SYNC) ||
	    (hdfu->dev_state == DFU_STATE_UPLOAD_IDLE)) {
		/* Update the state machine */
		hdfu->dev_state = DFU_STATE_IDLE;
		hdfu->dev_status = DFU_ERROR_NONE;
	}

	usb_dfu_detach_req = true;
}

/*
 * @brief  usb_dfu_download
 *         Handles the DFU DNLOAD request.
 * @param  pdev: device instance
 * @param  req: pointer to the request structure
 * @retval None
 */
static void usb_dfu_download(usb_handle_t *pdev, usb_setup_req_t *req)
{
	usb_dfu_handle_t *hdfu = (usb_dfu_handle_t *)pdev->class_data;
	uintptr_t data_ptr;
	uint32_t length;
	int ret;

	/* Data setup request */
	if (req->length > 0) {
		/* Unsupported state */
		if ((hdfu->dev_state != DFU_STATE_IDLE) &&
		    (hdfu->dev_state != DFU_STATE_DNLOAD_IDLE)) {
			/* Call the error management function (command will be nacked) */
			usb_core_ctl_error(pdev);
			return;
		}

		/* Get the data address */
		length = req->length;
		ret = hdfu->callback->download(hdfu->alt_setting, &data_ptr,
					       &length, pdev->user_data);
		if (ret == 0U) {
			/* Update the state machine */
			hdfu->dev_state = DFU_STATE_DNLOAD_SYNC;
			/* Start the transfer */
			usb_core_receive_ep0(pdev, (uint8_t *)data_ptr, length);
		} else {
			usb_core_ctl_error(pdev);
		}
	} else {
		/* End of DNLOAD operation*/
		if (hdfu->dev_state != DFU_STATE_DNLOAD_IDLE) {
			/* Call the error management function (command will be nacked) */
			usb_core_ctl_error(pdev);
			return;
		}
		/* End of DNLOAD operation*/
		hdfu->dev_state = DFU_STATE_MANIFEST_SYNC;
		ret = hdfu->callback->manifestation(hdfu->alt_setting, pdev->user_data);
		if (ret == 0U) {
			hdfu->dev_state = DFU_STATE_MANIFEST_SYNC;
		} else {
			usb_core_ctl_error(pdev);
		}
	}
}

/*
 * @brief  usb_dfu_upload
 *         Handles the DFU UPLOAD request.
 * @param  pdev: instance
 * @param  req: pointer to the request structure
 * @retval status
 */
static void usb_dfu_upload(usb_handle_t *pdev, usb_setup_req_t *req)
{
	usb_dfu_handle_t *hdfu = (usb_dfu_handle_t *)pdev->class_data;
	uintptr_t data_ptr;
	uint32_t length;
	int ret;

	/* Data setup request */
	if (req->length == 0) {
		/* No Data setup request */
		hdfu->dev_state = DFU_STATE_IDLE;
		return;
	}

	/* Unsupported state */
	if ((hdfu->dev_state != DFU_STATE_IDLE) && (hdfu->dev_state != DFU_STATE_UPLOAD_IDLE)) {
		ERROR("UPLOAD : Unsupported State\n");
		/* Call the error management function (command will be nacked) */
		usb_core_ctl_error(pdev);
		return;
	}

	/* Update the data address */
	length = req->length;
	ret = hdfu->callback->upload(hdfu->alt_setting, &data_ptr, &length, pdev->user_data);
	if (ret == 0U) {
		/* Short frame */
		hdfu->dev_state = (req->length > length) ? DFU_STATE_IDLE : DFU_STATE_UPLOAD_IDLE;

		/* Start the transfer */
		usb_core_transmit_ep0(pdev, (uint8_t *)data_ptr, length);
	} else {
		ERROR("UPLOAD : bad block %i on alt %i\n", req->value, req->index);
		hdfu->dev_state = DFU_STATE_ERROR;
		hdfu->dev_status = DFU_ERROR_STALLEDPKT;

		/* Call the error management function (command will be nacked) */
		usb_core_ctl_error(pdev);
	}
}

/*
 * @brief  usb_dfu_get_status
 *         Handles the DFU GETSTATUS request.
 * @param  pdev: instance
 * @retval status
 */
static void usb_dfu_get_status(usb_handle_t *pdev)
{
	usb_dfu_handle_t *hdfu = (usb_dfu_handle_t *)pdev->class_data;

	hdfu->status[0] = hdfu->dev_status;	/* bStatus */
	hdfu->status[1] = 0;			/* bwPollTimeout[3]; */
	hdfu->status[2] = 0;
	hdfu->status[3] = 0;
	hdfu->status[4] = hdfu->dev_state;	/* bState */
	hdfu->status[5] = 0;			/* iString */

	/* next step */
	switch (hdfu->dev_state) {
	case DFU_STATE_DNLOAD_SYNC:
		hdfu->dev_state = DFU_STATE_DNLOAD_IDLE;
		break;
	case DFU_STATE_MANIFEST_SYNC:
		/* We're MainfestationTolerant */
		hdfu->status[4] = DFU_STATE_MANIFEST;
		hdfu->status[1] = 1U; /* bwPollTimeout = 1ms */
		hdfu->dev_state = DFU_STATE_IDLE;
		break;

	default:
		break;
	}

	/* Start the transfer */
	usb_core_transmit_ep0(pdev, (uint8_t *)&hdfu->status[0], sizeof(hdfu->status));
}

/*
 * @brief  usb_dfu_clear_status
 *         Handles the DFU CLRSTATUS request.
 * @param  pdev: device instance
 * @retval status
 */
static void usb_dfu_clear_status(usb_handle_t *pdev)
{
	usb_dfu_handle_t *hdfu = (usb_dfu_handle_t *)pdev->class_data;

	if (hdfu->dev_state == DFU_STATE_ERROR) {
		hdfu->dev_state = DFU_STATE_IDLE;
		hdfu->dev_status = DFU_ERROR_NONE;
	} else {
		/* State Error */
		hdfu->dev_state = DFU_STATE_ERROR;
		hdfu->dev_status = DFU_ERROR_UNKNOWN;
	}
}

/*
 * @brief  usb_dfu_get_state
 *         Handles the DFU GETSTATE request.
 * @param  pdev: device instance
 * @retval None
 */
static void usb_dfu_get_state(usb_handle_t *pdev)
{
	usb_dfu_handle_t *hdfu = (usb_dfu_handle_t *)pdev->class_data;

	/* Return the current state of the DFU interface */
	usb_core_transmit_ep0(pdev, &hdfu->dev_state, 1);
}

/*
 * @brief  usb_dfu_abort
 *         Handles the DFU ABORT request.
 * @param  pdev: device instance
 * @retval None
 */
static void usb_dfu_abort(usb_handle_t *pdev)
{
	usb_dfu_handle_t *hdfu = (usb_dfu_handle_t *)pdev->class_data;

	if ((hdfu->dev_state == DFU_STATE_IDLE) ||
	    (hdfu->dev_state == DFU_STATE_DNLOAD_SYNC) ||
	    (hdfu->dev_state == DFU_STATE_DNLOAD_IDLE) ||
	    (hdfu->dev_state == DFU_STATE_MANIFEST_SYNC) ||
	    (hdfu->dev_state == DFU_STATE_UPLOAD_IDLE)) {
		hdfu->dev_state = DFU_STATE_IDLE;
		hdfu->dev_status = DFU_ERROR_NONE;
	}
}

/*
 * @brief  usb_dfu_setup
 *         Handle the DFU specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */
static uint8_t usb_dfu_setup(usb_handle_t *pdev, usb_setup_req_t *req)
{
	uint8_t *pbuf = NULL;
	uint16_t len = 0U;
	uint8_t ret = USBD_OK;
	usb_dfu_handle_t *hdfu = (usb_dfu_handle_t *)pdev->class_data;

	switch (req->bm_request & USB_REQ_TYPE_MASK) {
	case USB_REQ_TYPE_CLASS:
		switch (req->b_request) {
		case DFU_DNLOAD:
			usb_dfu_download(pdev, req);
			break;

		case DFU_UPLOAD:
			usb_dfu_upload(pdev, req);
			break;

		case DFU_GETSTATUS:
			usb_dfu_get_status(pdev);
			break;

		case DFU_CLRSTATUS:
			usb_dfu_clear_status(pdev);
			break;

		case DFU_GETSTATE:
			usb_dfu_get_state(pdev);
			break;

		case DFU_ABORT:
			usb_dfu_abort(pdev);
			break;

		case DFU_DETACH:
			usb_dfu_detach(pdev, req);
			break;

		default:
			ERROR("unkwon request %x on alternate %i\n",
			      req->b_request, hdfu->alt_setting);
			usb_core_ctl_error(pdev);
			ret = USBD_FAIL;
			break;
		}
		break;
	case USB_REQ_TYPE_STANDARD:
		switch (req->b_request) {
		case USB_REQ_GET_DESCRIPTOR:
			if (HIBYTE(req->value) == DFU_DESCRIPTOR_TYPE) {
				pbuf = pdev->desc->get_config_desc(&len);
				/* DFU descriptor at the end of the USB */
				pbuf += len - 9U;
				len = 9U;
				len = MIN(len, req->length);
			}

			/* Start the transfer */
			usb_core_transmit_ep0(pdev, pbuf, len);

			break;

		case USB_REQ_GET_INTERFACE:
			/* Start the transfer */
			usb_core_transmit_ep0(pdev, (uint8_t *)&hdfu->alt_setting, 1U);
			break;

		case USB_REQ_SET_INTERFACE:
			hdfu->alt_setting = LOBYTE(req->value);
			break;

		default:
			usb_core_ctl_error(pdev);
			ret = USBD_FAIL;
			break;
		}
	default:
		break;
	}

	return ret;
}

static const usb_class_t USBD_DFU_initvalue = {
	.init = usb_dfu_init,
	.de_init = usb_dfu_de_init,
	.setup = usb_dfu_setup,
	.ep0_tx_sent = usb_dfu_ep0_tx_ready,
	.ep0_rx_ready = usb_dfu_ep0_rx_ready,
	.data_in = usb_dfu_data_in,
	.data_out = usb_dfu_data_out,
	.sof = usb_dfu_sof,
	.iso_in_incomplete = usb_dfu_iso_in_incomplete,
	.iso_out_incomplete = usb_dfu_iso_out_incomplete,
};

void usb_dfu_register(usb_handle_t *pdev, usb_dfu_handle_t *phandle)
{
	pdev->class = (usb_class_t *)&USBD_DFU_initvalue;
	pdev->class_data = phandle;

	phandle->dev_state = DFU_STATE_IDLE;
	phandle->dev_status = DFU_ERROR_NONE;
}

int usb_dfu_loop(usb_handle_t *pdev, const usb_dfu_media_t *pmedia)
{
	uint32_t it_count;
	usb_status_t ret;
	usb_dfu_handle_t *hdfu = (usb_dfu_handle_t *)pdev->class_data;

	hdfu->callback = pmedia;
	usb_dfu_detach_req = false;
	/* Continue to handle USB core IT to assure complete data transmission */
	it_count = 100U;

	/* DFU infinite loop until DETACH_REQ */
	while (it_count != 0U) {
		ret = usb_core_handle_it(pdev);
		if (ret != USBD_OK) {
			return -EIO;
		}

		/* detach request received */
		if (usb_dfu_detach_req) {
			it_count--;
		}
	}

	return 0;
}
