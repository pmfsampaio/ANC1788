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
#include "lpc177x_8x_ssp.h"
#include "lcd_controller.h"

#include "anc-lpc1788.h"

void LED_DEBUG(void) {
	volatile int *p;
	p = (int *) 0x20098080;
	*p |= (1 << 27);
	for (;;) {
		p = (int *) 0x2009809c;
		*p = (1 << 27);
		p = (int *) 0x20098098;
		*p = (1 << 27);
	}
}

void LED(int i) {
	volatile int *p;

	p = (int *) 0x20098080;
	*p |= (1 << 26);
	p = (i) ? (int *) 0x2009809c : (int *) 0x20098098;
	*p = (1 << 26);

}

#include "TitleScreen.h"
#include "SettingsIcon.h"

int main(void) {
	int cpu, emc;
	SSP_CFG_Type SSP_ConfigStruct;

	SystemCoreClockUpdate();
	cpu = CLKPWR_GetCLK(CLKPWR_CLKTYPE_CPU );
	emc = CLKPWR_GetCLK(CLKPWR_CLKTYPE_EMC );

	GPIO_SetDir(2, (1 << 1), 1);
	GPIO_ClearValue(2, (1 << 1));
	GPIO_SetValue(2, (1 << 1));
	GPIO_ClearValue(2, (1 << 1));

	// initialize SSP configuration structure to default
	SSP_ConfigStructInit(&SSP_ConfigStruct);

	SSP_ConfigStruct.CPHA = 1 << 6;
	SSP_ConfigStruct.CPOL = 1 << 7;
	SSP_ConfigStruct.ClockRate = 10000;

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP0, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP0, ENABLE);

	GPIO_SetValue(2, (1 << 1));

	GPIO_SetDir(4, (1 << 26), 1);
	GPIO_ClearValue(4, (1 << 26));

	GPIO_SetValue(4, (1 << 26));

	debug_frmwrk_init();

	GPIO_SetValue(2, (1 << 1));
	_DBG_("Init LCD...");
	LCD_ControllerConfigure(0);
	LCD_Clear(RGB(0,0,0));
	//	LCD_On();
	for (;;) {
		LCD_Clear(RGB(255,0,0));
		LCD_Clear(RGB(0,255,0));
		LCD_Clear(RGB(0,0,255));
		//        LCD_DrawIcon(G_titleScreen, (800-320)/2, (480-240)/2);
		//		GPIO_ClearValue(4, (1 << 26));
		//		GPIO_SetValue(4, (1 << 26));
		LCD_DrawIcon(G_settingsIcon, 10, 19);
		LCD_DrawImageOnPage(0, G_titleScreen, (320 - 320) / 2, (240 - 240) / 2);

		GPIO_SetDir(4, (1 << 26), 1);
		for (;;) {
			GPIO_ClearValue(4, (1 << 26));
			GPIO_SetValue(4, (1 << 26));
		}
	}
}

