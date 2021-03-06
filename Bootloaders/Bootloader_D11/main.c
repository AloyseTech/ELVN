/*
* 1kByte USB DFU bootloader for Atmel SAMD11 microcontrollers
*
* Copyright (c) 2019, Theo Meyer <meyertheopro@gmail.com>
* Copyright (c) 2018, Peter Lawrence
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

#include "stddef.h"
#include "sam.h"
#include "usb.h"
#include "nvm_data.h"
#include "usb_descriptors.h"


/*- Definitions -------------------------------------------------------------*/
#define APPLICATION_LEN_OFFSET 0x10 //match the first "reserved" handler address in the irq vector

//#define LED_ENABLE
#define LED_PIN	27 // PA27

/* If this feature is enabled, the bootloader automatically continue to the application once flashed */
//#define BOOT_TO_APP_AFTER_DFU

/* Enable UART feature */
//#define FEATURE_UART

/* Enable USB DFU feature */
#define FEATURE_DFU

/* Bootloader entry methods */
/* Magic marker at a specific place in RAM (see BL_ENTRY_ON_MAGIC_ADDR) */
#define BL_ENTRY_ON_MAGIC_NUM		0xf02669ef
/* Reset "double tap" */
#define BL_ENTRY_ON_DOUBLE_RST
/* pin activated entry (on port A) */
//#define BL_ENTRY_ON_LOW_PIN			15
/* Bad app CRC */
#define BL_ENTRY_ON_BAD_CRC

#define BL_ENTRY_ON_MAGIC_ADDR		(uint32_t *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4)


#define UART_SERCOM              SERCOM0
#define UART_SERCOM_PMUX         PORT_PMUX_PMUXE_D_Val
#define UART_SERCOM_GCLK_ID      SERCOM0_GCLK_ID_CORE
#define UART_SERCOM_GCLK_GEN     1
#define UART_SERCOM_APBCMASK     PM_APBCMASK_SERCOM0
#define UART_SERCOM_TXPO         1
#define UART_SERCOM_RXPO         0
#define UART_TX_PIN              6 // PA04
#define UART_RX_PIN              4 // PA06
#define UART_SERCOM_GCLK	     8000000ul
#define UART_BAUDRATE		     115200ul
#define UART_BAUD_VAL	         (65536.0*(1.0-((float)(16.0*(float)UART_BAUDRATE)/(float)UART_SERCOM_GCLK)))

#define USB_CMD(dir, rcpt, type) ((USB_##dir##_TRANSFER << 7) | (USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))
#define SIMPLE_USB_CMD(rcpt, type) ((USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))

#define FLASH_ROW_SIZE (4 * FLASH_PAGE_SIZE)

/*- Types -------------------------------------------------------------------*/
typedef struct
{
	UsbDeviceDescBank  out;
	UsbDeviceDescBank  in;
} udc_mem_t;

/*- Variables ---------------------------------------------------------------*/
static uint32_t usb_config = 0;
static uint32_t dfu_status_choices[4] =
{
	0x00000000, 0x00000002, /* normal */
	0x00000000, 0x00000005, /* dl */
};

static udc_mem_t udc_mem[USB_EPT_NUM];
static uint32_t udc_ctrl_in_buf[16];
static uint32_t udc_ctrl_out_buf[16];


/*- Implementations ---------------------------------------------------------*/
//-----------------------------------------------------------------------------
static void nvm_write_page(uint32_t dest_addr, const uint8_t *page_buffer)
{
	if (0 == ((dest_addr >> 6) & 0x3))
	{
		//erase the row if necessary
		NVMCTRL->ADDR.reg = dest_addr >> 1;
		NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD(NVMCTRL_CTRLA_CMD_ER);
		while (!NVMCTRL->INTFLAG.bit.READY);
	}

	uint16_t *nvm_addr = (uint16_t *)(dest_addr);
	uint16_t *ram_addr = (uint16_t *)page_buffer;
	for (unsigned i = 0; i < 32; i++)
	*nvm_addr++ = *ram_addr++;
	while (!NVMCTRL->INTFLAG.bit.READY);
}

//-----------------------------------------------------------------------------
static void udc_control_send(const uint32_t *data, uint32_t size)
{
	/* USB peripheral *only* reads valid data from 32-bit aligned RAM locations */
	udc_mem[0].in.ADDR.reg = (uint32_t)data;

	udc_mem[0].in.PCKSIZE.reg = USB_DEVICE_PCKSIZE_BYTE_COUNT(size) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(0) | USB_DEVICE_PCKSIZE_SIZE(3 /*64 Byte*/);

	USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1;
	USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.BK1RDY = 1;

	while (0 == USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1);
}

//-----------------------------------------------------------------------------
static void udc_control_stall()
{
	USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;
}

//-----------------------------------------------------------------------------
static void udc_control_send_zlp(void)
{
	udc_control_send(NULL, 0); /* peripheral can't read from NULL address, but size is zero and this value takes less space to compile */
}

//-----------------------------------------------------------------------------
static uint8_t USB_Service(void)
{
	static uint32_t dfu_addr;

	if (USB->DEVICE.INTFLAG.bit.EORST) /* End Of Reset */
	{
		#ifdef LED_ENABLE
		PORT->Group[0].OUTTGL.reg = 1 << LED_PIN;
		#endif
		
		USB->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_EORST;
		USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN;

		for (int ep = 0; ep < USB_EPT_NUM; ep++)
		USB->DEVICE.DeviceEndpoint[ep].EPCFG.reg = 0;

		USB->DEVICE.DeviceEndpoint[0].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE0(1 /*CONTROL*/) | USB_DEVICE_EPCFG_EPTYPE1(1 /*CONTROL*/);
		USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.BK0RDY = 1;
		USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.bit.BK1RDY = 1;

		udc_mem[0].in.ADDR.reg = (uint32_t)udc_ctrl_in_buf;
		udc_mem[0].in.PCKSIZE.reg = USB_DEVICE_PCKSIZE_BYTE_COUNT(0) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(0) | USB_DEVICE_PCKSIZE_SIZE(3 /*64 Byte*/);

		udc_mem[0].out.ADDR.reg = (uint32_t)udc_ctrl_out_buf;
		udc_mem[0].out.PCKSIZE.reg = USB_DEVICE_PCKSIZE_BYTE_COUNT(64) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(0) | USB_DEVICE_PCKSIZE_SIZE(3 /*64 Byte*/);

		USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.bit.BK0RDY = 1;
	}

	if (USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT0) /* Transmit Complete 0 */
	{
		#ifdef LED_ENABLE
		PORT->Group[0].OUTTGL.reg = 1 << LED_PIN;
		#endif
		
		if (dfu_addr)
		{
			nvm_write_page(dfu_addr, (uint8_t*)udc_ctrl_out_buf);
			/*
			if (0 == ((dfu_addr >> 6) & 0x3))
			{
			NVMCTRL->ADDR.reg = dfu_addr >> 1;
			NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD(NVMCTRL_CTRLA_CMD_ER);
			while (!NVMCTRL->INTFLAG.bit.READY);
			}

			uint16_t *nvm_addr = (uint16_t *)(dfu_addr);
			uint16_t *ram_addr = (uint16_t *)udc_ctrl_out_buf;
			for (unsigned i = 0; i < 32; i++)
			*nvm_addr++ = *ram_addr++;
			while (!NVMCTRL->INTFLAG.bit.READY);
			*/

			udc_control_send_zlp();
			
			#ifdef BOOT_TO_APP_AFTER_DFU
			/* if this is the last page to be written, reset to application */
			if(dfu_addr == (*(volatile uint32_t *)(APPLICATION_START + APPLICATION_LEN_OFFSET)) % 64)
			//NVIC_SystemReset(); // we could also make main returns
			return 0;
			#endif
			dfu_addr = 0;
		}

		USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT0;
	}

	if (USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.RXSTP) /* Received Setup */
	{
		#ifdef LED_ENABLE
		PORT->Group[0].OUTTGL.reg = 1 << LED_PIN;
		#endif
		USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_RXSTP;
		USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.bit.BK0RDY = 1;

		usb_request_t *request = (usb_request_t *)udc_ctrl_out_buf;
		uint8_t type = request->wValue >> 8;
		//uint8_t index = request->wValue & 0xff;
		uint16_t length = request->wLength;
		static uint32_t *dfu_status = dfu_status_choices + 0;

		/* for these other USB requests, we must examine all fields in bmRequestType */
		if (USB_CMD(OUT, INTERFACE, STANDARD) == request->bmRequestType)
		{
			udc_control_send_zlp();
			return 1;
		}

		/* for these "simple" USB requests, we can ignore the direction and use only bRequest */
		switch (request->bmRequestType & 0x7F)
		{
			case SIMPLE_USB_CMD(DEVICE, STANDARD):
			case SIMPLE_USB_CMD(INTERFACE, STANDARD):
			switch (request->bRequest)
			{
				case USB_GET_DESCRIPTOR:
				if (USB_DEVICE_DESCRIPTOR == type)
				{
					udc_control_send((uint32_t *)&usb_device_descriptor, length);
				}
				else if (USB_CONFIGURATION_DESCRIPTOR == type)
				{
					udc_control_send((uint32_t *)&usb_configuration_hierarchy, length);
				}

				#if USE_STRING_DESCRIPTOR
				else if(USB_STRING_DESCRIPTOR == type)
				{
					uint8_t index = request->wValue & 0xff;

					if (0 == index)
					{
						//length = LIMIT(length, usb_string_descriptor_zero.bLength);
						//udc_control_send((uint32_t *)&usb_string_descriptor_zero, length);
						//uint8_t descriptor_zero[] __attribute__ ((aligned (4))) = {0x04, USB_STRING_DESCRIPTOR, 0x04, 0x09};
						udc_control_send((uint32_t *)string_descriptor, string_descriptor[0]);
					}
					else if (index < USB_STR_COUNT)
					{
						/*
						//1156
						const char *str = usb_strings[index];
						int len;

						for (len = 0; *str; len++, str++)
						{
						usb_string_descriptor_buffer[2 + len*2] = *str;
						usb_string_descriptor_buffer[3 + len*2] = 0;
						}

						usb_string_descriptor_buffer[0] = len*2 + 2;
						usb_string_descriptor_buffer[1] = USB_STRING_DESCRIPTOR;

						length = LIMIT(length, usb_string_descriptor_buffer[0]);
						*/
						/*
						if(index == USB_STR_MANUFACTURER)
						{
						udc_control_send((uint32_t*)manufacturer, manufacturer[0]);
						}
						else //index == USB_STR_PRODUCT
						{
						udc_control_send((uint32_t*)product, product[0]);
						}
						*/
						//udc_control_send((uint32_t*)product, product[0]);
						udc_control_send((uint32_t *)(string_descriptor+4), string_descriptor[4]);

					}
					else
					{
						udc_control_stall();
					}
				}
				#endif
				
				else
				{
					//USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;
					udc_control_stall();
				}
				break;
				case USB_GET_CONFIGURATION:
				udc_control_send(&usb_config, 1);
				break;
				case USB_GET_STATUS:
				udc_control_send(dfu_status_choices + 0, 2); /* a 32-bit aligned zero in RAM is all we need */
				break;
				case USB_SET_FEATURE:
				case USB_CLEAR_FEATURE:
				//USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;
				udc_control_stall();
				break;
				case USB_SET_ADDRESS:
				udc_control_send_zlp();
				USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | USB_DEVICE_DADD_DADD(request->wValue);
				break;
				case USB_SET_CONFIGURATION:
				usb_config = request->wValue;
				udc_control_send_zlp();
				break;
			}
			break;
			case SIMPLE_USB_CMD(INTERFACE, CLASS):
			switch (request->bRequest)
			{
				case 0x03: // DFU_GETSTATUS
				udc_control_send(&dfu_status[0], 6);
				break;
				case 0x05: // DFU_GETSTATE
				udc_control_send(&dfu_status[1], 1);
				break;
				case 0x01: // DFU_DNLOAD
				dfu_status = dfu_status_choices + 0;
				if (request->wLength)
				{
					dfu_status = dfu_status_choices + 2;
					dfu_addr = APPLICATION_START + request->wValue * 64;
				}
				/* fall through to below */
				default: // DFU_UPLOAD & others
				/* 0x00 == DFU_DETACH, 0x04 == DFU_CLRSTATUS, 0x06 == DFU_ABORT, and 0x01 == DFU_DNLOAD and 0x02 == DFU_UPLOAD */
				if (!dfu_addr)
				udc_control_send_zlp();
				break;
			}
			break;
		}
	}
	return 1;
}


#ifdef FEATURE_UART
static void UART_Init()
{
	#if UART_TX_PIN < 16 && UART_RX_PIN < 16
	PORT->Group[0].WRCONFIG.reg = PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_WRPINCFG
	| PORT_WRCONFIG_PMUX(UART_SERCOM_PMUX) | PORT_WRCONFIG_PINMASK((1 << UART_TX_PIN) | (1 << UART_RX_PIN));
	#elif UART_TX_PIN > 16 && UART_RX_PIN > 16
	PORT->Group[0].WRCONFIG.reg = PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_WRPINCFG
	| PORT_WRCONFIG_PMUX(UART_SERCOM_PMUX) | PORT_WRCONFIG_PINMASK((1 << (UART_TX_PIN - 16)) | (1 << (UART_RX_PIN - 16)));
	#else
	#error "UART TX and RX pin should be on the same 16bit pinmask for code size reduction"
	#endif
	
	PM->APBCMASK.reg |= UART_SERCOM_APBCMASK;
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(UART_SERCOM_GCLK_ID) |
	GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(UART_SERCOM_GCLK_GEN);
	
	UART_SERCOM->USART.CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
	while (UART_SERCOM->USART.CTRLA.bit.SWRST);


	UART_SERCOM->USART.CTRLA.reg = SERCOM_USART_CTRLA_RXPO(UART_SERCOM_RXPO) | SERCOM_USART_CTRLA_TXPO(UART_SERCOM_TXPO)
	| SERCOM_USART_CTRLA_MODE(1) | SERCOM_USART_CTRLA_DORD;
	
	UART_SERCOM->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_CHSIZE(0);
	while(UART_SERCOM->USART.SYNCBUSY.bit.CTRLB);
	
	UART_SERCOM->USART.BAUD.reg = UART_BAUD_VAL;
	
	UART_SERCOM->USART.CTRLA.bit.ENABLE = 1;
	#ifdef LED_ENABLE
	volatile uint32_t delay = 0;
	#endif
	while(UART_SERCOM->USART.SYNCBUSY.bit.ENABLE)
	{
		// TODO: System is stuck here
		// Is the clock generator correctly initialized? At the correct speed? The source of the Generator is running?
		#ifdef LED_ENABLE
		if(delay-- == 0)
		{
			PORT->Group[0].OUTTGL.reg = 1 << LED_PIN;
			delay = 0x10000;
		}
		#endif
	}
	
}

static void UART_Write(uint8_t data)
{
	while(!UART_SERCOM->USART.INTFLAG.bit.DRE);
	UART_SERCOM->USART.DATA.reg = (uint16_t)data;
}

static uint8_t UART_Read(void)
{
	while(!UART_SERCOM->USART.INTFLAG.bit.RXC);
	return((uint8_t)(UART_SERCOM->USART.DATA.reg & 0x00FF));
}


static uint8_t UART_Service()
{
	static uint8_t page_buffer[FLASH_PAGE_SIZE];
	static uint32_t i;
	static uint32_t dest_addr;
	static uint32_t * flash_ptr;
	
	if(!UART_SERCOM->USART.INTFLAG.bit.RXC) return 1;
	uint8_t data_8 = UART_Read();
	if (data_8 == '#')
	{
		UART_Write('s');
		UART_Write((uint8_t)((FLASH_SIZE - APPLICATION_START) / FLASH_ROW_SIZE));
	}
	else if (data_8 == 'e')
	{
		#if 0
		for(i = APPLICATION_START; i < FLASH_SIZE; i = i + 256)
		{
			//nvm_erase_row(i);
		}
		#endif
		dest_addr = APPLICATION_START;
		flash_ptr = (uint32_t *)APPLICATION_START;
		UART_Write('s');
	}
	else if (data_8 == 'p')
	{
		UART_Write('s');
		for (i = 0; i < FLASH_PAGE_SIZE; i++)
		{
			page_buffer[i] = UART_Read();
		}
		//nvm_write_buffer(dest_addr, page_buffer, FLASH_PAGE_SIZE);
		nvm_write_page(dest_addr, page_buffer);
		/*
		if (0 == ((dest_addr >> 6) & 0x3))
		{
		//erase the row if necessary
		NVMCTRL->ADDR.reg = dest_addr >> 1;
		NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD(NVMCTRL_CTRLA_CMD_ER);
		while (!NVMCTRL->INTFLAG.bit.READY);
		}

		uint16_t *nvm_addr = (uint16_t *)(dest_addr);
		uint16_t *ram_addr = (uint16_t *)page_buffer;
		for (unsigned i = 0; i < 32; i++)
		*nvm_addr++ = *ram_addr++;
		while (!NVMCTRL->INTFLAG.bit.READY);
		*/
		
		dest_addr += FLASH_PAGE_SIZE;
		UART_Write('s');
	}
	else if (data_8 == 'v')
	{
		UART_Write('s');
		for (i = 0; i < (FLASH_PAGE_SIZE/4); i++)
		{
			uint32_t app_start_address = *flash_ptr;
			UART_Write((uint8_t)(app_start_address));
			UART_Write((uint8_t)(app_start_address >> 8));
			UART_Write((uint8_t)(app_start_address >> 16));
			UART_Write((uint8_t)(app_start_address >> 24));
			flash_ptr++;
		}
	}
	else if (data_8 == 'c')
	{
		UART_Write('s');
		PAC1->WPCLR.reg = 2; /* clear DSU */

		DSU->ADDR.reg = APPLICATION_START; /* start CRC check at beginning of user app */
		DSU->LENGTH.reg = *(volatile uint32_t *)(APPLICATION_START + APPLICATION_LEN_OFFSET); /* use length encoded into unused vector address in user app */

		/* ask DSU to compute CRC */
		DSU->DATA.reg = 0xFFFFFFFF;
		DSU->CTRL.bit.CRC = 1;
		while (!DSU->STATUSA.bit.DONE);

		if (DSU->DATA.reg) //reverse CRC must be 0
		UART_Write('p'); // pass
		else
		UART_Write('f'); // fail
	}
	else if (data_8 == 'r')
	{
		return 0;
	}
	return 1;
}
#endif


int main(void)
{
	#ifdef LED_ENABLE
	/*
	#if LED_PIN <= 15
	PORT->Group[0].WRCONFIG.reg = PORT_WRCONFIG_HWSEL | PORT_WRCONFIG_WRPINCFG
	| PORT_WRCONFIG_PULLEN | PORT_WRCONFIG_INEN | PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_PINMASK(1 << (LED_PIN));
	#else
	PORT->Group[0].WRCONFIG.reg = PORT_WRCONFIG_HWSEL | PORT_WRCONFIG_WRPINCFG
	| PORT_WRCONFIG_ | PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_PINMASK(1 << (LED_PIN - 16));
	#endif
	*/
	PORT->Group[0].DIRSET.reg = 1 << LED_PIN;
	PORT->Group[0].PINCFG[LED_PIN].reg |= PORT_PINCFG_INEN;
	#endif
	
	#ifdef BL_ENTRY_ON_LOW_PIN
	/* configure PA15 (bootloader entry pin used by SAM-BA) as input pull-up */
	//PORT->Group[0].PINCFG[BL_ENTRY_ON_LOW_PIN].reg = PORT_PINCFG_PULLEN | PORT_PINCFG_INEN;
	//PORT->Group[0].OUTSET.reg = (1UL << BL_ENTRY_ON_LOW_PIN);
	#if BL_ENTRY_ON_LOW_PIN <= 15
	PORT->Group[0].WRCONFIG.reg = PORT_WRCONFIG_WRPINCFG
	| PORT_WRCONFIG_PULLEN | PORT_WRCONFIG_INEN | PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_PINMASK( 1 << BL_ENTRY_ON_LOW_PIN);
	#else
	PORT->Group[0].WRCONFIG.reg = PORT_WRCONFIG_HWSEL | PORT_WRCONFIG_WRPINCFG
	| PORT_WRCONFIG_PULLEN | PORT_WRCONFIG_INEN | PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_PINMASK(1 << (BL_ENTRY_ON_LOW_PIN - 16));
	#endif

	if (0 == (PORT->Group[0].IN.reg & (1UL << BL_ENTRY_ON_LOW_PIN)))
	goto run_bootloader; /* pin grounded, so run bootloader */
	#endif

	#ifdef BL_ENTRY_ON_MAGIC_NUM
	//if (PM->RCAUSE.reg & PM_RCAUSE_POR)
	//*BL_ENTRY_ON_MAGIC_ADDR = 0;
	
	if (*BL_ENTRY_ON_MAGIC_ADDR == BL_ENTRY_ON_MAGIC_NUM)
	{
		/* either a 'double tap' has happened or the app requested to run bootloader */
		*BL_ENTRY_ON_MAGIC_ADDR = 0;
		if (0 == (PM->RCAUSE.reg & PM_RCAUSE_POR))
		goto run_bootloader;
	}
	#endif


	#ifdef BL_ENTRY_ON_DOUBLE_RST
	
	#ifndef BL_ENTRY_ON_MAGIC_NUM
	#error Double tap reset to entry bootloader requires BL_ENTRY_ON_MAGIC_NUM
	#endif

	/* postpone boot for a short period of time; if a second reset happens during this window, the "magic" value will remain */
	*BL_ENTRY_ON_MAGIC_ADDR = BL_ENTRY_ON_MAGIC_NUM;
	volatile unsigned long wait = 0x10000; while (wait--);
	/* however, if execution reaches this point, the window of opportunity has closed and the "magic" disappears  */
	#endif

	*BL_ENTRY_ON_MAGIC_ADDR = 0;
	
	#ifdef BL_ENTRY_ON_BAD_CRC
	PAC1->WPCLR.reg = 2; /* clear DSU */

	DSU->ADDR.reg = APPLICATION_START; /* start CRC check at beginning of user app */
	DSU->LENGTH.reg = *(volatile uint32_t *)(APPLICATION_START + APPLICATION_LEN_OFFSET); /* use length encoded into unused vector address in user app */

	/* ask DSU to compute CRC */
	DSU->DATA.reg = 0xFFFFFFFF;
	DSU->CTRL.bit.CRC = 1;
	while (!DSU->STATUSA.bit.DONE);

	if (DSU->DATA.reg) //reverse CRC must be 0
	goto run_bootloader; /* CRC failed, so run bootloader */
	#endif

	return 0; /* we've checked everything and there is no reason to run the bootloader */

	run_bootloader:
	/*
	configure oscillator for crystal-free USB operation
	*/
	
	SYSCTRL->OSC8M.bit.PRESC = 0;

	SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33RDY | SYSCTRL_INTFLAG_BOD33DET | SYSCTRL_INTFLAG_DFLLRDY;

	NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_RWS_DUAL;

	SYSCTRL->DFLLCTRL.reg = 0; // See Errata 9905
	while (!SYSCTRL->PCLKSR.bit.DFLLRDY);

	SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_MUL(48000);
	SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE( NVM_READ_CAL(NVM_DFLL48M_COARSE_CAL) ) | SYSCTRL_DFLLVAL_FINE( NVM_READ_CAL(NVM_DFLL48M_FINE_CAL) );

	SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | SYSCTRL_DFLLCTRL_USBCRM | SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_BPLCKC | SYSCTRL_DFLLCTRL_CCDIS | SYSCTRL_DFLLCTRL_STABLE;

	while (!SYSCTRL->PCLKSR.bit.DFLLRDY);

	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC(GCLK_SOURCE_DFLL48M) | GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN;
	while (GCLK->STATUS.bit.SYNCBUSY);

	#ifdef FEATURE_UART
	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(1) | GCLK_GENCTRL_SRC(GCLK_SOURCE_OSC8M) | GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN;
	while (GCLK->STATUS.bit.SYNCBUSY);
	#endif

	#ifdef FEATURE_UART
	UART_Init();
	#endif
	
	/*
	initialize USB
	*/
	/* Configure PA24 and PA25 for USB simultaneously */
	PORT->Group[0].WRCONFIG.reg = PORT_WRCONFIG_HWSEL | PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_WRPMUX
	| PORT_WRCONFIG_PMUX(PORT_PMUX_PMUXE_G_Val) | PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_PINMASK((1 << (25 - 16)) | (1 << (24 - 16)));
	
	PM->APBBMASK.reg |= PM_APBBMASK_USB;

	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID(USB_GCLK_ID) | GCLK_CLKCTRL_GEN(0);

	USB->DEVICE.CTRLA.reg = USB_CTRLA_SWRST;
	while (USB->DEVICE.SYNCBUSY.bit.SWRST);

	USB->DEVICE.PADCAL.reg = USB_PADCAL_TRANSN( NVM_READ_CAL(NVM_USB_TRANSN) ) | USB_PADCAL_TRANSP( NVM_READ_CAL(NVM_USB_TRANSP) ) | USB_PADCAL_TRIM( NVM_READ_CAL(NVM_USB_TRIM) );

	USB->DEVICE.DESCADD.reg = (uint32_t)udc_mem;

	USB->DEVICE.CTRLA.reg = USB_CTRLA_MODE_DEVICE | USB_CTRLA_RUNSTDBY;
	USB->DEVICE.CTRLB.reg = USB_DEVICE_CTRLB_SPDCONF_FS;
	USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;

	
	/*
	service USB
	*/
	#ifdef LED_ENABLE
	volatile uint32_t delay = 0x40000;
	#endif
	while (
	#ifdef FEATURE_DFU
	USB_Service()
	#else
	true
	#endif
	&&
	#ifdef FEATURE_UART
	UART_Service()
	#else
	true
	#endif
	)
	{
		#ifdef LED_ENABLE
		if(delay-- == 0)
		{
			PORT->Group[0].OUTTGL.reg = 1 << LED_PIN;
			delay = 0x40000;
			//UART_Write('x');
		}
		#endif
	}
}
