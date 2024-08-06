/**
  ******************************************************************************
  * @file    usb_desc.c
  * @author  MCD Application Team
  * @version V4.1.0
  * @date    26-May-2017
  * @brief   Descriptors for Custom Composite device MSC+CDC
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"

/* USB Standard Device Descriptor */
const uint8_t Composite_DeviceDescriptor[Composite_SIZ_DEVICE_DESC] =
{
		0x12,                       /*bLength */
		USB_DEVICE_DESCRIPTOR_TYPE, /*bDescriptorType*/
		0x00,
		0x02,   /* bcdUSB = 2.00 */
		0x00,                       /*bDeviceClass (Miscellaneous)*/
		0x00,                       /*bDeviceSubClass*/
		0x00,                       /*bDeviceProtocol*/
		0x40,   /* bMaxPacketSize */
		0x83,
		0x04,   /* idVendor = 0x0483 */
		0x41,
		0x57,   /* idProduct = 0x7541 */
		0x00,
		0x02,   /* bcdDevice = 2.00 */
		1,              /* Index of string descriptor describing manufacturer */
		2,              /* Index of string descriptor describing product */
		3,              /* Index of string descriptor describing the device's serial number */
		0x01    /* bNumConfigurations */
}; /* Composite_DeviceDescriptor */


/* USB MSC+CDC Configuration Descriptor */
const uint8_t Composite_ConfigDescriptor[Composite_SIZ_CONFIG_DESC] =
{
	0x09, /* bLength: Configuration Descriptor size */
	USB_CONFIGURATION_DESCRIPTOR_TYPE,	/* bDescriptorType: Configuration */
	0x20,					/* wTotalLength: Bytes returned */
	0x00,
	0x01,					/* bNumInterfaces: 1 interface */
	0x01,					/* bConfigurationValue: Configuration value */
	0x00,					/* iConfiguration: Index of string descriptor describing the configuration*/
	0xC0,					/* bmAttributes: bus powered and Supports Remote Wakeup */
	0x32,					/* MaxPower 100 mA: this current is used for detecting Vbus */

	/************** Descriptor of Mass Storage interface ****************/
	0x09,					/* bLength: Interface Descriptor size */
	USB_INTERFACE_DESCRIPTOR_TYPE,		/* bDescriptorType: Interface descriptor type */
	MSC_INTERFACE_IDX,			/* bInterfaceNumber: Number of Interface */
	0x00,					/* bAlternateSetting: Alternate setting */
	0x02,					/* bNumEndpoints */
	0x08,					/* bInterfaceClass: MASS STORAGE Class */
	0x06,					/* bInterfaceSubClass : SCSI transparent*/
	0x50,					/* nInterfaceProtocol */
	0x00,					/* iInterface: */
	/******************** Mass Storage Endpoints ********************/
	0x07,					/*Endpoint descriptor length = 7*/
	0x05,					/*Endpoint descriptor type */
	MSC_IN_EP,				/*Endpoint address (IN1) */
	0x02,					/*Bulk endpoint type */
	0x40,					/*Maximum packet size (64 bytes) */
	0x00,
	0x00,					/*Polling interval in milliseconds */

	0x07,					/*Endpoint descriptor length = 7 */
	0x05,					/*Endpoint descriptor type */
	MSC_OUT_EP,				/*Endpoint address (OUT1) */
	0x02,					/*Bulk endpoint type */
	0x40,					/*Maximum packet size (64 bytes) */
	0x00,
	0x00,					/*Polling interval in milliseconds*/
}; /* Composite_ConfigDescriptor */

/* USB String Descriptors (optional) */
const uint8_t Composite_StringLangID[Composite_SIZ_STRING_LANGID] =
{
		Composite_SIZ_STRING_LANGID,
		USB_STRING_DESCRIPTOR_TYPE,
		0x09,
		0x04
}; /* LangID = 0x0409: U.S. English */

const uint8_t Composite_StringVendor[Composite_SIZ_STRING_VENDOR] =
{
	20, /* Size of Vendor string */
	USB_STRING_DESCRIPTOR_TYPE,  /* bDescriptorType*/
	'N', 0, 'o', 0, ' ', 0, 'V', 0, 'e', 0, 'n', 0, 'd', 0, 'o', 0, 'r', 0
};

const uint8_t Composite_StringProduct[Composite_SIZ_STRING_PRODUCT] =
{
	34,        /* bLength */
	USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
	'U', 0, 'S', 0, 'B', 0, ' ', 0, 'F', 0, 'l', 0,
	'o', 0, 'p', 0, 'p', 0, 'y', 0, ' ', 0, 'D', 0, 'r', 0, 'i', 0, 'v', 0,	'e', 0,
};

uint8_t Composite_StringSerial[Composite_SIZ_STRING_SERIAL] =
{
	Composite_SIZ_STRING_SERIAL,           /* bLength */
	USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
	'0', 0, '0', 0, '0', 0,'0', 0,'1', 0
};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

