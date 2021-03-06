/*
* Copyright (c) 2019, Th�o Meyer <meyertheopro@gmail.com>
* Copyright (c) 2016, Alex Taradov <alex@taradov.com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
* 3. The name of the author may not be used to endorse or promote products
*    derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/


/*- Includes ----------------------------------------------------------------*/
#include "usb.h"
#include "usb_descriptors.h"

/*- Variables ---------------------------------------------------------------*/
usb_device_descriptor_t usb_device_descriptor __attribute__ ((aligned (4))) = /* MUST BE IN RAM for USB peripheral */
{
	.bLength            = sizeof(usb_device_descriptor_t),
	.bDescriptorType    = USB_DEVICE_DESCRIPTOR,

	.bcdUSB                 = 0x0200,
	.bDeviceClass           = 0x00,
	.bDeviceSubClass        = 0x00,
	.bDeviceProtocol        = 0x00,
	
	.bMaxPacketSize0        = 64,
	.idVendor               = 0x1209,
	.idProduct              = 0x0001,
	.bcdDevice              = 0x0101,

	#if USE_STRING_DESCRIPTOR
	.iManufacturer          = USB_STR_ZERO,
	.iProduct               = USB_STR_PRODUCT,
	.iSerialNumber          = USB_STR_ZERO,
	#else
	.iManufacturer          = USB_STR_ZERO,
	.iProduct               = USB_STR_ZERO,
	.iSerialNumber          = USB_STR_ZERO,
	#endif

	.bNumConfigurations     = 1
};

usb_configuration_hierarchy_t usb_configuration_hierarchy __attribute__ ((aligned (4))) = /* MUST BE IN RAM for USB peripheral */
{
	.configuration =
	{
		.bLength             = sizeof(usb_configuration_descriptor_t),
		.bDescriptorType     = USB_CONFIGURATION_DESCRIPTOR,
		.wTotalLength        = sizeof(usb_configuration_hierarchy_t),
		.bNumInterfaces      = 1,
		.bConfigurationValue = 1,
		.iConfiguration      = USB_STR_ZERO,
		.bmAttributes        = 0x80,
		.bMaxPower           = 250, // 500 mA
	},

	.interface =
	{
		.bLength             = sizeof(usb_interface_descriptor_t),
		.bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
		.bInterfaceNumber    = 0,
		.bAlternateSetting   = 0,
		.bNumEndpoints       = 0,
		.bInterfaceClass     = 0xFE, /* Application Specific Class Code*/
		.bInterfaceSubClass  = 0x01, /* Device Firmware Upgrade Code */
		.bInterfaceProtocol  = 0x02, /* DFU mode protocol */
		.iInterface          = USB_STR_ZERO,
	},

	.dfu =
	{
		.bLength             = sizeof(usb_dfu_descriptor_t),
		.bDescriptorType     = 0x21, /* DFU FUNCTIONAL descriptor type. */
		.bmAttributes        = 0x03, /* bitCanDnload | bitCanUpload */
		.wDetachTimeout      = 0,
		.wTransferSize       = 64,
		.bcdDFU              = 0x100,
	},
};

const usb_string_descriptor_zero_t usb_string_descriptor_zero __attribute__ ((aligned (4))) =
{
	.bLength               = sizeof(usb_string_descriptor_zero_t),
	.bDescriptorType       = USB_STRING_DESCRIPTOR,
	.wLANGID               = 0x0409, // English (United States)
};

//string descriptor format : {arraySize, USB_STRING_DESCRIPTOR, char[0], 0, char[1], 0,..., char[n], 0}
uint8_t manufacturer[22] __attribute__ ((aligned (4))) ={22, USB_STRING_DESCRIPTOR, 'A', 0, 'l', 0, 'o', 0, 'y', 0, 's', 0, 'e', 0, 'T', 0, 'e', 0, 'c', 0, 'h', 0};//"AloyseTech";
uint8_t product[8] __attribute__ ((aligned (4))) =		{8, USB_STRING_DESCRIPTOR, 'D', 0, 'F', 0, 'U', 0};//"DFU";
uint8_t string_descriptor[] __attribute__ ((aligned (4))) =		{
	4, USB_STRING_DESCRIPTOR, 0x04, 0x09, // wLANGID : English (United States)
	8, USB_STRING_DESCRIPTOR, 'D', 0, 'F', 0, 'U', 0, //iProduct : "DFU";
};

uint8_t usb_string_descriptor_buffer[64] __attribute__ ((aligned (4)));