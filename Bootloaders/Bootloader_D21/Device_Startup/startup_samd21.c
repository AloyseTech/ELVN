/*
* Copyright (c) 2019, Th�o Meyer <meyertheopro@gmail.com>
* Copyright (c) 2016-2017, Alex Taradov <alex@taradov.com>
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

#include "sam.h"

/*- Prototypes --------------------------------------------------------------*/
void irq_handler_reset(void);

/*- Variables ---------------------------------------------------------------*/
extern int main(void);
extern void _stack_top(void);
extern unsigned int _etext;
extern unsigned int _data;
extern unsigned int _edata;
extern unsigned int _bss;
extern unsigned int _ebss;

//-----------------------------------------------------------------------------
__attribute__ ((used, section(".vectors")))
void (* const vectors[])(void) =
{
	&_stack_top,
	irq_handler_reset,
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
__attribute__ ((noinline, section(".romfunc")))
void irq_handler_reset(void)
{
	unsigned int *src, *dst;

	src = &_etext;
	dst = &_data;
	while (dst < &_edata)
	*dst++ = *src++;

	dst = &_bss;
	while (dst < &_ebss)
	*dst++ = 0;

	main();
	
	/* if bootloader returns, we proceed to the user app */

	unsigned long reset_vector = *(unsigned long *)(APPLICATION_START + 4);

	//could be done in the application
	//SCB->VTOR = reset_vector;

	__set_MSP(*(unsigned long *)(APPLICATION_START));

	asm("bx %0"::"r" (reset_vector));
}
