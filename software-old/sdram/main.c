/*
 * main.c
 *
 *  Created on: 28 de Jul de 2011
 *      Author: psampaio
 */
#include "debug_frmwrk.h"
#include "lpc177x_8x_gpio.h"
#include "lpc177x_8x_clkpwr.h"
#include "sdram_is42s16320b-7.h"

int main(void) {
	int cpu, emc_freq, usb, per;

	uint32_t dynamicMRD;

	SystemCoreClockUpdate();
	cpu = CLKPWR_GetCLK(CLKPWR_CLKTYPE_CPU);
	emc_freq = CLKPWR_GetCLK(CLKPWR_CLKTYPE_EMC);
	usb = CLKPWR_GetCLK(CLKPWR_CLKTYPE_USB);
	per = CLKPWR_GetCLK(CLKPWR_CLKTYPE_PER);

	GPIO_SetDir(2, (1 << 1), 1);
	GPIO_ClearValue(2, (1 << 1));

	debug_frmwrk_init();

	_DBG("CPU=");
	_DBD32(cpu);
	_DBG("\n\rEMC=");
	_DBD32(emc_freq);
	_DBG("\n\rUSB=");
	_DBD32(usb);
	_DBG("\n\rPER=");
	_DBD32(per);

	_DBG_("\n\rInit SDRAM...");
	SDRAM_Init();
	_DBG_("Init SDRAM Test");
	SDRAM_Test();
	_DBG_("Init SDRAM Full Test");
	if (SDRAM_FullTest() == 0)
		_DBG_("SDRAM Full Test no errors.");
	else
		_DBG_("***************** SDRAM Full Test Fail");
	_DBG_("Init SDRAM Full Test 2");
	if (SDRAM_FullTest() == 0)
		_DBG_("SDRAM Full Test no errors.");
	else
		_DBG_("***************** SDRAM Full Test Fail");


	GPIO_SetDir(4, (1 << 26), 1);
	for (;;) {
		GPIO_ClearValue(4, (1 << 26));
		GPIO_SetValue(4, (1 << 26));
	}
}
