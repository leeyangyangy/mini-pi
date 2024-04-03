/*
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef USB_CORE_H
#define USB_CORE_H

#include <stdint.h>

#include <lib/utils_def.h>

#define USBD_MAX_NUM_INTERFACES			1
#define USBD_MAX_NUM_CONFIGURATION		1

#define USB_LEN_DEV_QUALIFIER_DESC		0x0A
#define USB_LEN_DEV_DESC			0x12
#define USB_LEN_CFG_DESC			0x09
#define USB_LEN_IF_DESC				0x09
#define USB_LEN_EP_DESC				0x07
#define USB_LEN_OTG_DESC			0x03
#define USB_LEN_LANGID_STR_DESC			0x04
#define USB_LEN_OTHER_SPEED_DESC_SIZ		0x09

#define USBD_IDX_LANGID_STR			0x00
#define USBD_IDX_MFC_STR			0x01
#define USBD_IDX_PRODUCT_STR			0x02
#define USBD_IDX_SERIAL_STR			0x03
#define USBD_IDX_CONFIG_STR			0x04
#define USBD_IDX_INTERFACE_STR			0x05
#define USBD_IDX_USER0_STR			0x06

#define USB_REQ_TYPE_STANDARD			0x00
#define USB_REQ_TYPE_CLASS			0x20
#define USB_REQ_TYPE_VENDOR			0x40
#define USB_REQ_TYPE_MASK			0x60

#define USB_REQ_RECIPIENT_DEVICE		0x00
#define USB_REQ_RECIPIENT_INTERFACE		0x01
#define USB_REQ_RECIPIENT_ENDPOINT		0x02
#define USB_REQ_RECIPIENT_MASK			0x1F

#define USB_REQ_DIRECTION			0x80

#define USB_REQ_GET_STATUS			0x00
#define USB_REQ_CLEAR_FEATURE			0x01
#define USB_REQ_SET_FEATURE			0x03
#define USB_REQ_SET_ADDRESS			0x05
#define USB_REQ_GET_DESCRIPTOR			0x06
#define USB_REQ_SET_DESCRIPTOR			0x07
#define USB_REQ_GET_CONFIGURATION		0x08
#define USB_REQ_SET_CONFIGURATION		0x09
#define USB_REQ_GET_INTERFACE			0x0A
#define USB_REQ_SET_INTERFACE			0x0B
#define USB_REQ_SYNCH_FRAME			0x0C

#define USB_DESC_TYPE_DEVICE			0x01
#define USB_DESC_TYPE_CONFIGURATION		0x02
#define USB_DESC_TYPE_STRING			0x03
#define USB_DESC_TYPE_INTERFACE			0x04
#define USB_DESC_TYPE_ENDPOINT			0x05
#define USB_DESC_TYPE_DEVICE_QUALIFIER		0x06
#define USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION	0x07
#define USB_DESC_TYPE_BOS			0x0F

#define USB_CONFIG_REMOTE_WAKEUP		2
#define USB_CONFIG_SELF_POWERED			1

#define USB_FEATURE_EP_HALT			0
#define USB_FEATURE_REMOTE_WAKEUP		1
#define USB_FEATURE_TEST_MODE			2

#define USB_DEVICE_CAPABITY_TYPE		0x10

#define USB_MAX_EP0_SIZE			64

/*  Device Status */
#define USBD_STATE_DEFAULT			1
#define USBD_STATE_ADDRESSED			2
#define USBD_STATE_CONFIGURED			3
#define USBD_STATE_SUSPENDED			4

/*  EP0 State */
#define USBD_EP0_IDLE				0
#define USBD_EP0_SETUP				1
#define USBD_EP0_DATA_IN			2
#define USBD_EP0_DATA_OUT			3
#define USBD_EP0_STATUS_IN			4
#define USBD_EP0_STATUS_OUT			5
#define USBD_EP0_STALL				6

#define USBD_EP_TYPE_CTRL			0
#define USBD_EP_TYPE_ISOC			1
#define USBD_EP_TYPE_BULK			2
#define USBD_EP_TYPE_INTR			3

#define USB_OTG_SPEED_HIGH			0
#define USB_OTG_SPEED_HIGH_IN_FULL		1
#define USB_OTG_SPEED_LOW			2
#define USB_OTG_SPEED_FULL			3

#define  USBD_OUT_EPNUM_MASK			GENMASK(15, 0)
#define  USBD_OUT_COUNT_MASK			GENMASK(31, 16)
#define  USBD_OUT_COUNT_SHIFT			16U

#define LOBYTE(x)	((uint8_t)((x) & 0x00FF))
#define HIBYTE(x)	((uint8_t)(((x) & 0xFF00) >> 8))

typedef struct {
	uint8_t bm_request;
	uint8_t b_request;
	uint16_t value;
	uint16_t index;
	uint16_t length;
} usb_setup_req_t;

struct usb_handle;

typedef struct {
	uint8_t (*init)(struct usb_handle *pdev, uint8_t cfgidx);
	uint8_t (*de_init)(struct usb_handle *pdev, uint8_t cfgidx);
	/* Control Endpoints*/
	uint8_t (*setup)(struct usb_handle *pdev, usb_setup_req_t  *req);
	uint8_t (*ep0_tx_sent)(struct usb_handle *pdev);
	uint8_t (*ep0_rx_ready)(struct usb_handle *pdev);
	/* Class Specific Endpoints*/
	uint8_t (*data_in)(struct usb_handle *pdev, uint8_t epnum);
	uint8_t (*data_out)(struct usb_handle *pdev, uint8_t epnum);
	uint8_t (*sof)(struct usb_handle *pdev);
	uint8_t (*iso_in_incomplete)(struct usb_handle *pdev, uint8_t epnum);
	uint8_t (*iso_out_incomplete)(struct usb_handle *pdev, uint8_t epnum);
} usb_class_t;

/* Following USB Device status */
typedef enum {
	USBD_OK = 0,
	USBD_BUSY,
	USBD_FAIL,
	USBD_TIMEOUT
} usb_status_t;

/* Action to do after IT handling */
typedef enum {
	USB_NOTHING = 0,
	USB_DATA_OUT,
	USB_DATA_IN,
	USB_SETUP,
	USB_ENUM_DONE,
	USB_READ_DATA_PACKET,
	USB_READ_SETUP_PACKET,
	USB_RESET,
	USB_RESUME,
	USB_SUSPEND,
	USB_LPM,
	USB_SOF,
	USB_DISCONNECT,
	USB_WRITE_EMPTY
} usb_action_t;

/* USB Device descriptors structure */
typedef struct {
	uint8_t *(*get_device_desc)(uint16_t *length);
	uint8_t *(*get_lang_id_desc)(uint16_t *length);
	uint8_t *(*get_manufacturer_desc)(uint16_t *length);
	uint8_t *(*get_product_desc)(uint16_t *length);
	uint8_t *(*get_serial_desc)(uint16_t *length);
	uint8_t *(*get_configuration_desc)(uint16_t *length);
	uint8_t *(*get_interface_desc)(uint16_t *length);
	uint8_t *(*get_usr_desc)(uint8_t index, uint16_t *length);
	uint8_t *(*get_config_desc)(uint16_t *length);
	uint8_t *(*get_device_qualifier_desc)(uint16_t *length);
} usb_desc_t;

/* USB Device handle structure */
typedef struct {
	uint32_t status;
	uint32_t total_length;
	uint32_t rem_length;
	uint32_t maxpacket;
} usb_endpoint_t;

typedef struct {
	uint8_t num; /* Endpoint number
		      * This parameter must be a number between Min_Data = 1
		      * and Max_Data = 15
		      */
	bool is_in; /* Endpoint direction */
	uint8_t type; /* Endpoint type */
	uint32_t maxpacket; /* Endpoint Max packet size
			     * This parameter must be a number between
			     * Min_Data = 0 and Max_Data = 64KB
			     */
	uint8_t *xfer_buff; /* Pointer to transfer buffer */
	uint32_t xfer_len; /* Current transfer length */
	uint32_t xfer_count; /* Partial transfer length in case of multi
			      * packet transfer
			      */
} usbd_ep_t;

typedef enum {
	LPM_L0 = 0x00, /* on */
	LPM_L1 = 0x01, /* LPM L1 sleep */
	LPM_L2 = 0x02, /* suspend */
	LPM_L3 = 0x03, /* off */
} pcd_lpm_state_t;

/* USB Device descriptors structure */
typedef struct {
	usb_status_t (*ep0_out_start)(void *handle);
	usb_status_t (*ep_start_xfer)(void *handle, usbd_ep_t *ep);
	usb_status_t (*ep0_start_xfer)(void *handle, usbd_ep_t *ep);
	usb_status_t (*write_packet)(void *handle, uint8_t *src,
				     uint8_t ch_ep_num, uint16_t len);
	void *(*read_packet)(void *handle, uint8_t *dest, uint16_t len);
	usb_status_t (*ep_set_stall)(void *handle, usbd_ep_t *ep);
	usb_status_t (*start_device)(void *handle);
	usb_status_t (*stop_device)(void *handle);
	usb_status_t (*set_address)(void *handle, uint8_t address);
	usb_status_t (*write_empty_tx_fifo)(void *handle,
					    uint32_t epnum, uint32_t xfer_len,
					    uint32_t *xfer_count,
					    uint32_t maxpacket,
					    uint8_t **xfer_buff);
	usb_action_t (*it_handler)(void *handle, uint32_t *param);
} usb_driver_t;

/* USB Peripheral Controller Drivers */
typedef struct {
	void *instance; /* Register base address */
	usbd_ep_t in_ep[15]; /* IN endpoint parameters */
	usbd_ep_t out_ep[15]; /* OUT endpoint parameters */
	uint32_t setup[12]; /* Setup packet buffer */
	pcd_lpm_state_t lpm_state; /* LPM State */
} pcd_handle_t;

/* USB Device handle structure */
typedef struct usb_handle {
	uint8_t id;
	uint32_t dev_config;
	uint32_t dev_config_status;
	usb_endpoint_t ep_in[15];
	usb_endpoint_t ep_out[15];
	uint32_t ep0_state;
	uint32_t ep0_data_len;
	uint8_t dev_state;
	uint8_t dev_old_state;
	uint8_t dev_address;
	uint32_t dev_remote_wakeup;
	usb_setup_req_t request;
	const usb_desc_t *desc;
	usb_class_t *class;
	void *class_data;
	void *user_data;
	pcd_handle_t *data;
	const usb_driver_t *driver;
} usb_handle_t;

usb_status_t usb_core_handle_it(usb_handle_t *pdev);
usb_status_t usb_core_receive(usb_handle_t *pdev, uint8_t ep_addr,
			      uint8_t *p_buf, uint32_t len);
usb_status_t usb_core_transmit(usb_handle_t *pdev, uint8_t ep_addr,
			       uint8_t *p_buf, uint32_t len);
usb_status_t usb_core_receive_ep0(usb_handle_t *pdev, uint8_t *p_buf,
				  uint32_t len);
usb_status_t usb_core_transmit_ep0(usb_handle_t *pdev, uint8_t *p_buf,
				   uint32_t len);
void usb_core_ctl_error(usb_handle_t *pdev);
usb_status_t usb_core_start(usb_handle_t *pdev);
usb_status_t usb_core_stop(usb_handle_t *pdev);
usb_status_t register_usb_driver(usb_handle_t *pdev, pcd_handle_t *pcd_handle,
				 const usb_driver_t *driver,
				 void *driver_handle);
usb_status_t register_platform(usb_handle_t *pdev,
			       const usb_desc_t *plat_call_back);

#endif /* USB_CORE_H */
