/*----------------------------------------------------------------------------
 *      Name:    MEMORY.C
 *      Purpose: USB Mass Storage Demo
 *      Version: V1.10
 *----------------------------------------------------------------------------
 *      This software is supplied "AS IS" without any warranties, express,
 *      implied or statutory, including but not limited to the implied
 *      warranties of fitness for purpose, satisfactory quality and
 *      noninfringement. Keil extends you a royalty-free right to reproduce
 *      and distribute executable files created using this software for use
 *      on NXP Semiconductors LPC family microcontroller devices only. Nothing
 *      else gives you the right to use this software.
 *
 *      Copyright (c) 2005-2009 Keil Software.
 *---------------------------------------------------------------------------*/

#include "LPC177x_8x.h"

#include "lpc_types.h"

#include "usb.h"
#include "usbcfg.h"
#include "usbhw.h"
#include "usbcore.h"
#include "mscuser.h"

#include "memory.h"
#include "lpc177x_8x_clkpwr.h"
#include "lpc177x_8x_nvic.h"

#include "debug_frmwrk.h"

/* Example group ----------------------------------------------------------- */
/** @defgroup USBDEV_USBMassStorage	USB Mass Storage Device
 * @ingroup USBDEV_Examples
 * @{
 */

/** @defgroup USBDEV_MscUsbHw	USB-MSD USB Hardware
 * @ingroup USBDEV_USBMassStorage
 * @{
 */

/**
 * @}
 */


/** @defgroup USBDEV_MscUsbCore USB-MSD USB Core
 * @ingroup USBDEV_USBMassStorage
 * @{
 */

/**
 * @}
 */

/** @defgroup USBDEV_MscUsbDesc USB-MSD USB Descriptors
 * @ingroup USBDEV_USBMassStorage
 * @{
 */

/**
 * @}
 */


/** @defgroup USBDEV_MscUsbReg	USB-MSD USB Register
 * @ingroup USBDEV_USBMassStorage
 * @{
 */

/**
 * @}
 */


/** @defgroup USBDEV_MscUsbUser USB-MSD USB User
 * @ingroup USBDEV_USBMassStorage
 * @{
 */

/**
 * @}
 */


/** @defgroup USBDEV_MscUsbCfg	USB-MSD USB Configuration
 * @ingroup USBDEV_USBMassStorage
 * @{
 */

/**
 * @}
 */


/** @defgroup USBDEV_MscConf	USB-MSD MSC Configuration
 * @ingroup USBDEV_USBMassStorage
 * @{
 */

/**
 * @}
 */

#define MATRIXARB (*(volatile unsigned int *)0x400FC188)


extern uint8_t Memory[MSC_MemorySize];         /* MSC Memory in RAM */

volatile int i;

#define PLL1CFG_Val           0x00000023
#define USBCLKSEL_Val         ((0x02<<8) | 0x00000001)
/* Main Program */

//int main (void) __attribute__ ((section (".isrsection")));

int main (void) {
	uint32_t n, usbclk;
#if 1
	  LPC_SC->PLL1CFG   = PLL1CFG_Val;
	  LPC_SC->PLL1CON   = 0x01;             /* PLL1 Enable                        */
	  LPC_SC->PLL1FEED  = 0xAA;
	  LPC_SC->PLL1FEED  = 0x55;
	  while (!(LPC_SC->PLL1STAT & (1<<10)));/* Wait for PLOCK1                    */
	  LPC_SC->USBCLKSEL = USBCLKSEL_Val;    /* Setup USB Clock Divider            */
#endif
	  SCB->VTOR  = 0x10000000 & 0x3FFFFF80;

	SystemCoreClockUpdate();
	usbclk = CLKPWR_GetCLK(CLKPWR_CLKTYPE_USB);

	debug_frmwrk_init();

	_DBG_("USB Device");

	for (n = 0; n < MSC_ImageSize; n++) {     /* Copy Initial Disk Image */
		Memory[n] = DiskImage[n];               /*   from Flash to RAM     */
	}

	USB_Init();                               /* USB Initialization */
	USB_Connect(TRUE);                        /* USB Connect */

	while (1) {
		_DBG("*");
//		i++;
//		USB_IRQHandler();                                /* Loop forever */

	}
}

/**
 * @}
 */
