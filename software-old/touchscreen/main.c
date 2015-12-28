/*
 * main.c
 *
 *  Created on: May 1, 2013
 *      Author: psampaio
 */

#include "system_LPC177x_8x.h"
#include "debug_frmwrk.h"
#include "lpc177x_8x_gpio.h"
#include "lpc177x_8x_clkpwr.h"
#include "anc-lpc1788.h"

#include "tsc2007.h"

void LED_DEBUG(void)
{
	volatile int *p;
	p = (int *)0x20098080;
	*p |= (1 << 27);
	for(;;) {
		p = (int *)0x2009809c;
		*p = (1 << 27);
		p = (int *)0x20098098;
		*p = (1 << 27);
	}
}



void LED(int i)
{
	volatile int *p;

	p = (int *)0x20098080;
	*p |= (1 << 26);
	p = (i) ? (int *)0x2009809c : (int *)0x20098098;
	*p = (1 << 26);
}

void Delay(int j);

int main(void) {
	int cpu, emc;
	volatile int y, res;
	TS_DSC ts;

	SystemCoreClockUpdate();
	cpu = CLKPWR_GetCLK(CLKPWR_CLKTYPE_CPU);
	emc = CLKPWR_GetCLK(CLKPWR_CLKTYPE_EMC);

	debug_frmwrk_init();

	_DBG_("Init TS Test...");

	res = TS_TSC2007_Init(400000);
	for (;;) {
		if (TS_IsPenOn()) {
			TS_Read(&ts);
			_DBD(ts.x);
			_DBC(' ');
			_DBD(ts.y);
			_DBG_("");
		}
		else {
			y++;
		}
		Delay(100);
	}
}



