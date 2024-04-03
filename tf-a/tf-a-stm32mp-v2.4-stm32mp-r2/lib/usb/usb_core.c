/*
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <stdint.h>

#include <common/debug.h>
#include <lib/usb/usb_core.h>

/* define for field bEndpointAddress */
#define EP_DIR_MASK		BIT(7)
#define EP_DIR_IN		BIT(7)
#define EP_NUM_MASK		GENMASK(3, 0)

#define EP0_IN			(0U | EP_DIR_IN)
#define EP0_OUT			0U

/* USB address between 1 through 127 = 0x7F mask */
#define ADDRESS_MASK		GENMASK(6, 0)

/*
 * @brief  Set a STALL condition over an endpoint
 * @param  pdev: USB handle
 * @param  ep_addr: endpoint address
 * @retval HAL status
 */
static usb_status_t usb_core_set_stall(usb_handle_t *pdev, uint8_t ep_addr)
{
	usbd_ep_t *ep;
	pcd_handle_t *hpcd = (pcd_handle_t *)pdev->data;
	uint8_t num;

	num = ep_addr & EP_NUM_MASK;
	if ((EP_DIR_MASK & ep_addr) == EP_DIR_IN) {
		ep = &hpcd->in_ep[num];
		ep->is_in = true;
	} else {
		ep = &hpcd->out_ep[num];
		ep->is_in = false;
	}
	ep->num = num;

	pdev->driver->ep_set_stall(hpcd->instance, ep);
	if (num == 0U) {
		pdev->driver->ep0_out_start(hpcd->instance);
	}

	return USBD_OK;
}

/*
 * usb_core_get_desc
 *         Handle Get Descriptor requests
 * pdev : device instance
 * req : usb request
 * return : status
 */
static void usb_core_get_desc(usb_handle_t *pdev, usb_setup_req_t *req)
{
	uint16_t len;
	uint8_t *pbuf;
	uint8_t desc_type = HIBYTE(req->value);
	uint8_t desc_idx = LOBYTE(req->value);

	switch (desc_type) {
	case USB_DESC_TYPE_DEVICE:
		pbuf = pdev->desc->get_device_desc(&len);
		break;

	case USB_DESC_TYPE_CONFIGURATION:
		pbuf = (uint8_t *)pdev->desc->get_config_desc(&len);
		pbuf[1] = USB_DESC_TYPE_CONFIGURATION;
		break;

	case USB_DESC_TYPE_STRING:
		switch (desc_idx) {
		case USBD_IDX_LANGID_STR:
			pbuf = pdev->desc->get_lang_id_desc(&len);
			break;

		case USBD_IDX_MFC_STR:
			pbuf = pdev->desc->get_manufacturer_desc(&len);
			break;

		case USBD_IDX_PRODUCT_STR:
			pbuf = pdev->desc->get_product_desc(&len);
			break;

		case USBD_IDX_SERIAL_STR:
			pbuf = pdev->desc->get_serial_desc(&len);
			break;

		case USBD_IDX_CONFIG_STR:
			pbuf = pdev->desc->get_configuration_desc(&len);
			break;

		case USBD_IDX_INTERFACE_STR:
			pbuf = pdev->desc->get_interface_desc(&len);
			break;

		/* for all USER string */
		case USBD_IDX_USER0_STR:
		default:
			pbuf = pdev->desc->get_usr_desc(desc_idx - USBD_IDX_USER0_STR, &len);
			break;
		}
		break;

	case USB_DESC_TYPE_DEVICE_QUALIFIER:
		pbuf = (uint8_t *)pdev->desc->get_device_qualifier_desc(&len);
		break;

	case USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION:
		pbuf = (uint8_t *)pdev->desc->get_config_desc(&len);
		pbuf[1] = USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION;
		break;

	default:
		ERROR("Unknown request %i\n", desc_type);
		usb_core_ctl_error(pdev);
		return;
	}

	if ((len != 0U) && (req->length != 0U)) {
		len = MIN(len, req->length);

		/* Start the transfer */
		usb_core_transmit_ep0(pdev, pbuf, len);
	}
}

/*
 * usb_core_set_config
 *         Handle Set device configuration request
 * pdev : device instance
 * req : usb request
 * return : status
 */
static void usb_core_set_config(usb_handle_t *pdev, usb_setup_req_t *req)
{
	static uint8_t cfgidx;

	cfgidx = LOBYTE(req->value);

	if (cfgidx > USBD_MAX_NUM_CONFIGURATION) {
		usb_core_ctl_error(pdev);
		return;
	}

	switch (pdev->dev_state) {
	case USBD_STATE_ADDRESSED:
		if (cfgidx != 0U) {
			pdev->dev_config = cfgidx;
			pdev->dev_state = USBD_STATE_CONFIGURED;
			if (!pdev->class) {
				usb_core_ctl_error(pdev);
				return;
			}
			/* Set configuration  and Start the Class */
			if (pdev->class->init(pdev, cfgidx) != 0U) {
				usb_core_ctl_error(pdev);
				return;
			}
		}
		break;

	case USBD_STATE_CONFIGURED:
		if (cfgidx == 0U) {
			pdev->dev_state = USBD_STATE_ADDRESSED;
			pdev->dev_config = cfgidx;
			pdev->class->de_init(pdev, cfgidx);
		} else if (cfgidx != pdev->dev_config) {
			if (pdev->class != NULL) {
				usb_core_ctl_error(pdev);
				return;
			}
			/* Clear old configuration */
			pdev->class->de_init(pdev, pdev->dev_config);
			/* Set new configuration */
			pdev->dev_config = cfgidx;
			/* Set configuration and start the Class*/
			if (pdev->class->init(pdev, cfgidx) != 0U) {
				usb_core_ctl_error(pdev);
				return;
			}
		}
		break;

	default:
		usb_core_ctl_error(pdev);
		return;
	}

	/* Send status */
	usb_core_transmit_ep0(pdev, NULL, 0U);
}

/*
 * usb_core_get_status
 *         Handle Get Status request
 * pdev : device instance
 * req : usb request
 * return : status
 */
static void usb_core_get_status(usb_handle_t *pdev, usb_setup_req_t *req)
{
	if ((pdev->dev_state != USBD_STATE_ADDRESSED) &&
	    (pdev->dev_state != USBD_STATE_CONFIGURED)) {
		usb_core_ctl_error(pdev);
		return;
	}

	pdev->dev_config_status = USB_CONFIG_SELF_POWERED;

	if (pdev->dev_remote_wakeup != 0U) {
		pdev->dev_config_status |= USB_CONFIG_REMOTE_WAKEUP;
	}

	/* Start the transfer */
	usb_core_transmit_ep0(pdev, (uint8_t *)&pdev->dev_config_status, 2U);
}

/*
 * usb_core_set_address
 *         Set device address
 * pdev : device instance
 * req : usb request
 * return : status
 */
static void usb_core_set_address(usb_handle_t *pdev, usb_setup_req_t *req)
{
	uint8_t dev_addr;

	if ((req->index != 0U) || (req->length != 0U)) {
		usb_core_ctl_error(pdev);
		return;
	}

	dev_addr = req->value & ADDRESS_MASK;
	if (pdev->dev_state != USBD_STATE_DEFAULT) {
		usb_core_ctl_error(pdev);
		return;
	}

	pdev->dev_address = dev_addr;
	pdev->driver->set_address(((pcd_handle_t *)(pdev->data))->instance, dev_addr);

	/* Send status */
	usb_core_transmit_ep0(pdev, NULL, 0U);

	if (dev_addr != 0U) {
		pdev->dev_state  = USBD_STATE_ADDRESSED;
	} else {
		pdev->dev_state  = USBD_STATE_DEFAULT;
	}
}

/*
 * usb_core_dev_req
 *         Handle standard usb device requests
 * pdev : device instance
 * req : usb request
 * return : status
 */
static usb_status_t usb_core_dev_req(usb_handle_t *pdev, usb_setup_req_t *req)
{
	VERBOSE("receive request %i\n", req->b_request);
	switch (req->b_request) {
	case USB_REQ_GET_DESCRIPTOR:
		usb_core_get_desc(pdev, req);
		break;

	case USB_REQ_SET_CONFIGURATION:
		usb_core_set_config(pdev, req);
		break;

	case USB_REQ_GET_STATUS:
		usb_core_get_status(pdev, req);
		break;

	case USB_REQ_SET_ADDRESS:
		usb_core_set_address(pdev, req);
		break;

	case USB_REQ_GET_CONFIGURATION:
	case USB_REQ_SET_FEATURE:
	case USB_REQ_CLEAR_FEATURE:
	default:
		ERROR("NOT SUPPORTED %i\n", req->b_request);
		usb_core_ctl_error(pdev);
		break;
	}

	return USBD_OK;
}

/*
 * usb_core_itf_req
 *         Handle standard usb interface requests
 * pdev : device instance
 * req : usb request
 * return : status
 */
static usb_status_t usb_core_itf_req(usb_handle_t *pdev, usb_setup_req_t *req)
{
	if (pdev->dev_state != USBD_STATE_CONFIGURED) {
		usb_core_ctl_error(pdev);
		return USBD_OK;
	}

	if (LOBYTE(req->index) <= USBD_MAX_NUM_INTERFACES) {
		pdev->class->setup(pdev, req);

		if (req->length == 0U) {
			usb_core_transmit_ep0(pdev, NULL, 0U);
		}
	} else {
		usb_core_ctl_error(pdev);
	}

	return USBD_OK;
}

/*
 * @brief  USBD_ParseSetupRequest
 *         Copy buffer into setup structure
 * @param  pdev: device instance
 * @param  req: usb request
 * @retval None
 */
static void usb_core_parse_req(usb_setup_req_t *req, uint8_t *pdata)
{
	req->bm_request = pdata[0];
	req->b_request = pdata[1];
	req->value = pdata[2] + (pdata[3] << 8);
	req->index = pdata[4] + (pdata[5] << 8);
	req->length = pdata[6] + (pdata[7] << 8);
}

/*
 * usb_core_setup_stage
 *         Handle the setup stage
 * pdev: device instance
 * return : status
 */
static usb_status_t usb_core_setup_stage(usb_handle_t *pdev, uint8_t *psetup)
{
	usb_core_parse_req(&pdev->request, psetup);

	pdev->ep0_state = USBD_EP0_SETUP;
	pdev->ep0_data_len = pdev->request.length;

	switch (pdev->request.bm_request & USB_REQ_RECIPIENT_MASK) {
	case USB_REQ_RECIPIENT_DEVICE:
		usb_core_dev_req(pdev, &pdev->request);
		break;

	case USB_REQ_RECIPIENT_INTERFACE:
		usb_core_itf_req(pdev, &pdev->request);
		break;

	case USB_REQ_RECIPIENT_ENDPOINT:
	default:
		ERROR("receive unsupported request %i",
		      pdev->request.bm_request & USB_REQ_RECIPIENT_MASK);
		usb_core_set_stall(pdev, pdev->request.bm_request & USB_REQ_DIRECTION);
		return USBD_FAIL;
	}

	return USBD_OK;
}

/*
 * usb_core_data_out
 *         Handle data OUT stage
 * pdev: device instance
 * epnum: endpoint index
 * return : status
 */
static usb_status_t usb_core_data_out(usb_handle_t *pdev, uint8_t epnum,
				      uint8_t *pdata)
{
	usb_endpoint_t *pep;

	if (epnum == 0U) {
		pep = &pdev->ep_out[0];
		if (pdev->ep0_state == USBD_EP0_DATA_OUT) {
			if (pep->rem_length > pep->maxpacket) {
				pep->rem_length -= pep->maxpacket;

				usb_core_receive(pdev, 0U, pdata,
						 MIN(pep->rem_length,
						     pep->maxpacket));
			} else {
				if (pdev->class->ep0_rx_ready &&
				    (pdev->dev_state == USBD_STATE_CONFIGURED))
					pdev->class->ep0_rx_ready(pdev);

				usb_core_transmit_ep0(pdev, NULL, 0U);
			}
		}
	} else if (pdev->class->data_out != NULL &&
		   (pdev->dev_state == USBD_STATE_CONFIGURED)) {
		pdev->class->data_out(pdev, epnum);
	}

	return USBD_OK;
}

/*
 * usb_core_data_in
 *         Handle data in stage
 * pdev: device instance
 * epnum: endpoint index
 * return : status
 */
static usb_status_t usb_core_data_in(usb_handle_t *pdev, uint8_t epnum,
				     uint8_t *pdata)
{
	if (epnum == 0U) {
		usb_endpoint_t *pep = &pdev->ep_in[0];

		if (pdev->ep0_state == USBD_EP0_DATA_IN) {
			if (pep->rem_length > pep->maxpacket) {
				pep->rem_length -= pep->maxpacket;

				usb_core_transmit(pdev, 0U, pdata,
						  pep->rem_length);

				/* Prepare endpoint for premature
				 * end of transfer
				 */
				usb_core_receive(pdev, 0U, NULL, 0U);
			} else {
				/* Last packet is MPS multiple,
				 * so send ZLP packet
				 */
				if ((pep->total_length % pep->maxpacket == 0U) &&
				    (pep->total_length >= pep->maxpacket) &&
				    (pep->total_length < pdev->ep0_data_len)) {
					usb_core_transmit(pdev, 0U, NULL, 0U);

					pdev->ep0_data_len = 0U;

					/* Prepare endpoint for premature
					 * end of transfer
					 */
					usb_core_receive(pdev, 0U, NULL, 0U);
				} else {
					if (pdev->class->ep0_tx_sent != NULL &&
					    (pdev->dev_state ==
					     USBD_STATE_CONFIGURED))
						pdev->class->ep0_tx_sent(pdev);

					/* Start the transfer */
					usb_core_receive_ep0(pdev, NULL, 0U);
				}
			}
		}
	} else if (pdev->class->data_in != NULL &&
		  (pdev->dev_state == USBD_STATE_CONFIGURED)) {
		pdev->class->data_in(pdev, epnum);
	}

	return USBD_OK;
}

/*
 * usb_core_suspend
 *         Handle suspend event
 * pdev : device instance
 * return : status
 */

static usb_status_t usb_core_suspend(usb_handle_t  *pdev)
{
	INFO("USB Suspend mode\n");
	pdev->dev_old_state =  pdev->dev_state;
	pdev->dev_state  = USBD_STATE_SUSPENDED;

	return USBD_OK;
}

/*
 * usb_core_resume
 *         Handle resume event
 * pdev : device instance
 * return : status
 */

static usb_status_t usb_core_resume(usb_handle_t *pdev)
{
	INFO("USB Resume\n");
	pdev->dev_state = pdev->dev_old_state;

	return USBD_OK;
}

/*
 * usb_core_sof
 *         Handle SOF event
 * pdev : device instance
 * return : status
 */

static usb_status_t usb_core_sof(usb_handle_t *pdev)
{
	if (pdev->dev_state == USBD_STATE_CONFIGURED) {
		if (pdev->class->sof != NULL)
			pdev->class->sof(pdev);
	}

	return USBD_OK;
}

/*
 * usb_core_disconnect
 *         Handle device disconnection event
 * pdev : device instance
 * return : status
 */
static usb_status_t usb_core_disconnect(usb_handle_t *pdev)
{
	/* Free class resources */
	pdev->dev_state = USBD_STATE_DEFAULT;
	pdev->class->de_init(pdev, pdev->dev_config);

	return USBD_OK;
}

usb_status_t usb_core_handle_it(usb_handle_t *pdev)
{
	uint32_t param = 0U;
	uint32_t len = 0U;
	usbd_ep_t *ep;

	switch (pdev->driver->it_handler(pdev->data->instance, &param)) {
	case USB_DATA_OUT:
		usb_core_data_out(pdev, param,
				  pdev->data->out_ep[param].xfer_buff);
		break;

	case USB_DATA_IN:
		usb_core_data_in(pdev, param,
				 pdev->data->in_ep[param].xfer_buff);
		break;

	case USB_SETUP:
		usb_core_setup_stage(pdev, (uint8_t *)pdev->data->setup);
		break;

	case USB_ENUM_DONE:
		break;

	case USB_READ_DATA_PACKET:
		ep = &pdev->data->out_ep[param &  USBD_OUT_EPNUM_MASK];
		len = (param &  USBD_OUT_COUNT_MASK) >> USBD_OUT_COUNT_SHIFT;
		pdev->driver->read_packet(pdev->data->instance,
					  ep->xfer_buff, len);
		ep->xfer_buff += len;
		ep->xfer_count += len;
		break;

	case USB_READ_SETUP_PACKET:
		ep = &pdev->data->out_ep[param &  USBD_OUT_EPNUM_MASK];
		len = (param &  USBD_OUT_COUNT_MASK) >> 0x10;
		pdev->driver->read_packet(pdev->data->instance,
					  (uint8_t *)pdev->data->setup, 8);
		ep->xfer_count += len;
		break;

	case USB_RESET:
		pdev->dev_state = USBD_STATE_DEFAULT;
		break;

	case USB_RESUME:
		if (pdev->data->lpm_state == LPM_L1) {
			pdev->data->lpm_state = LPM_L0;
		} else {
			usb_core_resume(pdev);
		}
		break;

	case USB_SUSPEND:
		usb_core_suspend(pdev);
		break;

	case USB_LPM:
		if (pdev->data->lpm_state == LPM_L0) {
			pdev->data->lpm_state = LPM_L1;
		} else {
			usb_core_suspend(pdev);
		}
		break;

	case USB_SOF:
		usb_core_sof(pdev);
		break;

	case USB_DISCONNECT:
		usb_core_disconnect(pdev);
		break;

	case USB_WRITE_EMPTY:
		pdev->driver->write_empty_tx_fifo(pdev->data->instance, param,
				     pdev->data->in_ep[param].xfer_len,
				     (uint32_t *)&pdev->data->in_ep[param].xfer_count,
				     pdev->data->in_ep[param].maxpacket,
				     &pdev->data->in_ep[param].xfer_buff);
		break;

	case USB_NOTHING:
	default:
		break;
	}

	return USBD_OK;
}

/**
 * @brief  Receive an amount of data
 * @param  pdev: USB handle
 * @param  ep_addr: endpoint address
 * @param  pBuf: pointer to the reception buffer
 * @param  len: amount of data to be received
 * @retval status
 */
usb_status_t usb_core_receive(usb_handle_t *pdev, uint8_t ep_addr,
			      uint8_t *buf, uint32_t len)
{
	usbd_ep_t *ep;
	pcd_handle_t *hpcd = (pcd_handle_t *)pdev->data;
	uint8_t num;

	num = ep_addr & EP_NUM_MASK;
	ep = &hpcd->out_ep[num];

	/* Setup and start the Xfer */
	ep->xfer_buff = buf;
	ep->xfer_len = len;
	ep->xfer_count = 0U;
	ep->is_in = false;
	ep->num = num;

	if (num == 0U) {
		pdev->driver->ep0_start_xfer(hpcd->instance, ep);
	} else {
		pdev->driver->ep_start_xfer(hpcd->instance, ep);
	}

	return USBD_OK;
}

/*
 * @brief  Send an amount of data
 * @param  pdev: USB handle
 * @param  ep_addr: endpoint address
 * @param  pBuf: pointer to the transmission buffer
 * @param  len: amount of data to be sent
 * @retval status
 */
usb_status_t usb_core_transmit(usb_handle_t *pdev, uint8_t ep_addr,
			       uint8_t *buf, uint32_t len)
{
	usbd_ep_t *ep;
	pcd_handle_t *hpcd = (pcd_handle_t *)pdev->data;
	uint8_t num;

	num = ep_addr & EP_NUM_MASK;
	ep = &hpcd->in_ep[num];

	/* Setup and start the Xfer */
	ep->xfer_buff = buf;
	ep->xfer_len = len;
	ep->xfer_count = 0U;
	ep->is_in = true;
	ep->num = num;

	if (num == 0U) {
		pdev->driver->ep0_start_xfer(hpcd->instance, ep);
	} else {
		pdev->driver->ep_start_xfer(hpcd->instance, ep);
	}

	return USBD_OK;
}

/**
 * @brief  Receive an amount of data on ep0
 * @param  pdev: USB handle
 * @param  buf: pointer to the reception buffer
 * @param  len: amount of data to be received
 * @retval status
 */
usb_status_t usb_core_receive_ep0(usb_handle_t *pdev, uint8_t *buf,
				  uint32_t len)
{
	/* Prepare the reception of the buffer over EP0 */
	if (len != 0U) {
		pdev->ep0_state = USBD_EP0_DATA_OUT;
	} else {
		pdev->ep0_state = USBD_EP0_STATUS_OUT;
	}

	pdev->ep_out[0].total_length = len;
	pdev->ep_out[0].rem_length = len;

	/* Start the transfer */
	return usb_core_receive(pdev, 0U, buf, len);
}

/*
 * @brief  Send an amount of data on ep0
 * @param  pdev: USB handle
 * @param  buf: pointer to the transmission buffer
 * @param  len: amount of data to be sent
 * @retval status
 */
usb_status_t usb_core_transmit_ep0(usb_handle_t *pdev, uint8_t *buf,
				   uint32_t len)
{
	/* Set EP0 State */
	if (len != 0U) {
		pdev->ep0_state = USBD_EP0_DATA_IN;
	} else {
		pdev->ep0_state = USBD_EP0_STATUS_IN;
	}

	pdev->ep_in[0].total_length = len;
	pdev->ep_in[0].rem_length = len;

	/* Start the transfer */
	return usb_core_transmit(pdev, 0U, buf, len);
}

/*
 * @brief  usb_core_ctl_error
 *         Handle USB low level error
 * @param  pdev: device instance
 * @param  req: usb request
 * @retval None
 */

void usb_core_ctl_error(usb_handle_t *pdev)
{
	ERROR("%s : Send an ERROR\n", __func__);
	usb_core_set_stall(pdev, EP0_IN);
	usb_core_set_stall(pdev, EP0_OUT);
}

/*
 * usb_core_start
 *         Start the USB device core.
 * pdev: Device Handle
 * return : USBD Status
 */
usb_status_t usb_core_start(usb_handle_t *pdev)
{
	/* Start the low level driver */
	pdev->driver->start_device(pdev->data->instance);

	return USBD_OK;
}

/*
 * usb_core_stop
 *         Stop the USB device core.
 * pdev: Device Handle
 * return : USBD Status
 */
usb_status_t usb_core_stop(usb_handle_t *pdev)
{
	/* Free class resources */
	pdev->class->de_init(pdev, pdev->dev_config);

	/* Stop the low level driver */
	pdev->driver->stop_device(pdev->data->instance);

	return USBD_OK;
}

/*
 * register_usb_driver
 *         Stop the USB device core.
 * pdev: Device Handle
 * @param  hpcd: PCD handle
 * @param  driver: USB driver
 * @param  driver_handle: USB driver handle
 * return : USBD Status
 */
usb_status_t register_usb_driver(usb_handle_t *pdev, pcd_handle_t *pcd_handle,
				 const usb_driver_t *driver,
				 void *driver_handle)
{
	uint8_t i;

	assert(pdev != NULL);
	assert(pcd_handle != NULL);
	assert(driver != NULL);
	assert(driver_handle != NULL);

	/* Free class resources */
	pdev->driver = driver;
	pdev->data = pcd_handle;
	pdev->data->instance = driver_handle;
	pdev->dev_state = USBD_STATE_DEFAULT;
	pdev->ep0_state = USBD_EP0_IDLE;

	/* Copy endpoint information */
	for (i = 0U; i < 15U; i++) {
		pdev->ep_in[i].maxpacket = pdev->data->in_ep[i].maxpacket;
		pdev->ep_out[i].maxpacket = pdev->data->out_ep[i].maxpacket;
	}

	return USBD_OK;
}

/*
 * register_platform
 *         Register the USB device core.
 * pdev: Device Handle
 * plat_call_back: callback
 * return : USBD Status
 */
usb_status_t register_platform(usb_handle_t *pdev,
			       const usb_desc_t *plat_call_back)
{
	assert(pdev != NULL);
	assert(plat_call_back != NULL);

	/* Save platform info in class resources */
	pdev->desc = plat_call_back;

	return USBD_OK;
}
