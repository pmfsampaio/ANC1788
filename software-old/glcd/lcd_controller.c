/*
 * lcd_controller.c
 *
 *  Created on: 31 de Jul de 2011
 *      Author: psampaio
 */
#include "lpc177x_8x_pinsel.h"
#include "lpc177x_8x_clkpwr.h"
#include "lcd_controller.h"
#include "lpc177x_8x_gpio.h"
#include "lpc177x_8x_ssp.h"


volatile T_pixelColor _fbMem[UEZ_LCD_DISPLAY_WIDTH*UEZ_LCD_DISPLAY_WIDTH] __attribute__ ((aligned (8)));

#define LCD_FRAME_BUFFER ((unsigned int)&_fbMem[0])

#define C_GLCD_PIX_CLK          (6.4*1000000l)

void CS_Init(void)
{
		PINSEL_ConfigPin(0, 16, 0);
		PINSEL_ConfigPin(0, 15, 2);
		PINSEL_ConfigPin(0, 17, 2);
		PINSEL_ConfigPin(0, 18, 2);
        GPIO_SetDir(0, (1<<16), 1);
        GPIO_SetValue(0, (1<<16));
        GPIO_ClearValue(0, (1<<16));
        GPIO_SetValue(0, (1<<16));
}

void CS_Force(int32_t state)
{
        if (state){
                GPIO_SetValue(0, (1<<16));
        }else{
                GPIO_ClearValue(0, (1<<16));
        }
}

uint8_t sspreadbuf[20];

static void write_reg(unsigned short reg, unsigned short val) {
	//      pr_debug("%s: writing %x to %x\n", __func__, reg, val);
	unsigned char cmd[3];
	SSP_DATA_SETUP_Type xferConfig;

	cmd[0] = 0x70;
	cmd[1] = 0;
	cmd[2] = reg;

    CS_Force(0);
    xferConfig.tx_data = cmd;
    xferConfig.rx_data = sspreadbuf;
    xferConfig.length = sizeof (cmd);
    SSP_ReadWrite(LPC_SSP0, &xferConfig, SSP_TRANSFER_POLLING);
    CS_Force(1);
//	SPISendStr(cmd, 3);

	cmd[0] = 0x72;
	cmd[1] = (val >> 8) & 0xff;
	cmd[2] = val;

	CS_Force(0);
    xferConfig.tx_data = cmd;
    xferConfig.rx_data = sspreadbuf;
    xferConfig.length = sizeof (cmd);
    SSP_ReadWrite(LPC_SSP0, &xferConfig, SSP_TRANSFER_POLLING);
    CS_Force(1);

//	SPISendStr(cmd, 3);

	//    SPISend(0x00700000 | reg);
	//    SPISend(0x00720000 | val);
}

void init_panel_hw(void) {
	int i;
    const unsigned short seq[] = {
            0x01, 0x6300,
          0x0A, 0x4008, // 0x4008, //0x7F1F
   };

    for (i = 0; i < sizeof(seq) / sizeof(seq[0]); i += 2)
		write_reg(seq[i], seq[i + 1]);
}

#define C_GLCD_REFRESH_FREQ     (50HZ)
#define C_GLCD_H_SIZE           320
#define C_GLCD_H_PULSE          30
#define C_GLCD_H_FRONT_PORCH    20
#define C_GLCD_H_BACK_PORCH     68//38
#define C_GLCD_V_SIZE           240
#define C_GLCD_V_PULSE          3
#define C_GLCD_V_FRONT_PORCH    4//5
#define C_GLCD_V_BACK_PORCH     18//15

#define C_GLCD_PWR_ENA_DIS_DLY  10000
#define C_GLCD_ENA_DIS_DLY      10000

#define CRSR_PIX_32     0
#define CRSR_PIX_64     1
#define CRSR_ASYNC      0
#define CRSR_FRAME_SYNC 2

#define TEXT_DEF_TAB_SIZE 5

#define TEXT_BEL1_FUNC()

#define MAX_GLCD_STR_SIZE       256

#define C_GLCD_CLK_PER_LINE     (C_GLCD_H_SIZE + C_GLCD_H_PULSE + C_GLCD_H_FRONT_PORCH + C_GLCD_H_BACK_PORCH)
#define C_GLCD_LINES_PER_FRAME  (C_GLCD_V_SIZE + C_GLCD_V_PULSE + C_GLCD_V_FRONT_PORCH + C_GLCD_V_BACK_PORCH)
#define C_GLCD_PIX_CLK          (6400000)


#define LCD_CFG_CLKDIV 0

#define LCD_POL_BCD 26
#define LCD_POL_IVS 11
#define LCD_POL_IHS 12
#define LCD_POL_IPC 13
#define LCD_POL_IOE 14
#define LCD_POL_CPL 16
// init Horizontal Timing
#define LCD_TIMH_HBP 24
#define LCD_TIMH_HFP 16
#define LCD_TIMH_HSW 8
#define LCD_TIMH_PPL 2
// init Vertical Timing
#define LCD_TIMV_VBP 24
#define LCD_TIMV_VFP 16
#define LCD_TIMV_VSW 10
#define LCD_TIMV_LPP 0


int LCD_ControllerConfigure(T_LCDControllerSettings *aSettings)
{
	volatile unsigned int regValue = 0;
	unsigned int* pPal = 0;
	volatile unsigned int dotClockDivider;
	int i = 0;
	volatile unsigned int calcDiv;
	volatile unsigned int *pDst;

	CS_Init();

	PINSEL_ConfigPin(1, 29, 7);
	PINSEL_ConfigPin(1, 28, 7);
	PINSEL_ConfigPin(1,	27, 7);
	PINSEL_ConfigPin(1, 26, 7);
	PINSEL_ConfigPin(2, 12, 7);
	PINSEL_ConfigPin(2, 13, 7);
	PINSEL_ConfigPin(0, 9, 7);
	PINSEL_ConfigPin(0, 8, 7);

	PINSEL_ConfigPin(1, 25, 7);
	PINSEL_ConfigPin(1, 24, 7);
	PINSEL_ConfigPin(1, 23, 7);
	PINSEL_ConfigPin(1, 22, 7);
	PINSEL_ConfigPin(1, 21, 7);
	PINSEL_ConfigPin(1, 20, 7);
	PINSEL_ConfigPin(0, 7, 7);
	PINSEL_ConfigPin(0, 6, 7);

	PINSEL_ConfigPin(2, 9, 7);
	PINSEL_ConfigPin(2, 8, 7);
	PINSEL_ConfigPin(2, 7, 7);
	PINSEL_ConfigPin(2, 6, 7);
	PINSEL_ConfigPin(4, 29, 7);
	PINSEL_ConfigPin(4, 28, 7);
	PINSEL_ConfigPin(0, 5, 7);
	PINSEL_ConfigPin(0, 4, 7);

	PINSEL_ConfigPin(2, 5, 7);
	PINSEL_ConfigPin(2, 4, 7);
	PINSEL_ConfigPin(2, 3, 7);
	PINSEL_ConfigPin(2, 2, 7);
//	PINSEL_ConfigPin(2, 1, 7);
//	PINSEL_ConfigPin(2, 0, 7);


    // Turn on the power to the subsystem
    LPC_SC->PCONP |= (1<<0);
    // disable the display`
    LPC_LCD->CTRL = 0;

    LPC_SC->LCD_CFG = 4; //(96000000 / C_GLCD_PIX_CLK) ;
    LPC_LCD->POL = (0 << LCD_POL_BCD) | (1 << LCD_POL_IVS) | (1 << LCD_POL_IHS) | (1
			<< LCD_POL_IPC) | (1 << LCD_POL_IOE) | ((C_GLCD_H_SIZE - 1)
			<< LCD_POL_CPL) | (4);
	// init Horizontal Timing
    LPC_LCD->TIMH = ((C_GLCD_H_BACK_PORCH - 1) << LCD_TIMH_HBP)
			| ((C_GLCD_H_FRONT_PORCH - 1) << LCD_TIMH_HFP) | ((C_GLCD_H_PULSE
			- 1) << LCD_TIMH_HSW)
			| (((C_GLCD_H_SIZE / 16) - 1) << LCD_TIMH_PPL);
	// init Vertical Timing
    LPC_LCD->TIMV = ((C_GLCD_V_BACK_PORCH) << LCD_TIMV_VBP)
			| ((C_GLCD_V_FRONT_PORCH) << LCD_TIMV_VFP) | ((C_GLCD_V_PULSE)
			<< LCD_TIMV_VSW) | ((C_GLCD_V_SIZE - 1) << LCD_TIMV_LPP);
    LPC_LCD->LE = 0;
	// Frame Base Address doubleword aligned
    LPC_LCD->UPBASE = LCD_FRAME_BUFFER;
 //   LPC_LCD->LPBASE = LCD_FRAME_BUFFER;
	// init colour pallet
    LPC_LCD->CTRL = LCD_CTRL_BGR | LCD_CTRL_LcdEn | LCD_CTRL_LcdTFT | (5 << LCD_CTRL_LcdBpp_BIT);
	for (i = C_GLCD_PWR_ENA_DIS_DLY; i; i--)
		;
	LPC_LCD->CTRL |= LCD_CTRL_LcdPwr;

    init_panel_hw();
	pDst = (unsigned int *) LCD_FRAME_BUFFER;
	for (i = 0; (C_GLCD_H_SIZE * C_GLCD_V_SIZE) > i; i++) {
		*pDst++ = -1;
	}

    return 0;
}

void LCD_Off(void)
{
    LPC_LCD->CTRL &= ~LCD_CTRL_LcdEn;
    LPC_LCD->CTRL &= ~LCD_CTRL_LcdPwr;
}

void LCD_On(void)
{
    LPC_LCD->CTRL |= LCD_CTRL_LcdEn;
    LPC_LCD->CTRL |= LCD_CTRL_LcdPwr;
}


void LCD_Clear(unsigned int rgb)
{
	unsigned int* pMem = 0;
	int i;

    pMem = (unsigned int*) LCD_FRAME_BUFFER;//aSettings->iBaseAddress;
    for(i= 0; i < 240*320;i++) {
    	*pMem = rgb;
    	pMem++;
    }
}

//void SUIPutPointRaw(unsigned char aPage, unsigned short x, unsigned short y, T_pixelColor pixel)  __attribute__((section(".startup")));

void SUIPutPointRaw(unsigned char aPage, unsigned short x, unsigned short y, T_pixelColor pixel)
{
    const unsigned int bytesPerPage = 768000; //UEZ_LCD_DISPLAY_WIDTH*UEZ_LCD_DISPLAY_HEIGHT*sizeof(T_pixelColor);

    if ((x >= UEZ_LCD_DISPLAY_WIDTH) || (y >= UEZ_LCD_DISPLAY_HEIGHT))
        return;

    *((T_pixelColor *)(
        LCD_FRAME_BUFFER + bytesPerPage*aPage +
        x*sizeof(T_pixelColor) +
        UEZ_LCD_DISPLAY_WIDTH*sizeof(T_pixelColor)*y)) = pixel;
}

void LCD_DrawIcon(
        const unsigned char *p,
        unsigned short aXOffset,
        unsigned short aYOffset)
{
    // First, load the background into the buffer
    const unsigned char *p_palette;
    unsigned int x, y;
    const unsigned char *p_pixels;
    unsigned int width;
    unsigned int height;
    unsigned int numColors;
    T_pixelColor rgbArray[256];
    unsigned int i;
    const unsigned char *rgb;

    width = *((unsigned short *)(p+12));
    height = *((unsigned short *)(p+14));
    numColors = p[6];
    numColors *= 256;
    numColors += p[5];

    p_palette = p+18;
    p_pixels = p+18+numColors*3;

    // Convert the palette to native pixels
    rgb = p_palette;
    for (i=0; i<numColors; i++, rgb+=3)
        rgbArray[i] = RGB(rgb[2], rgb[1], rgb[0]);

    for (y=0; y<height; y++) {
        const unsigned char *p_raster = &p_pixels[(height-1-y)*width];
        for (x=0; x<width; x++) {
            T_pixelColor color = rgbArray[*(p_raster++)];
            #if (SIMPLEUI_DOUBLE_SIZED_ICONS != 0)
                // Icons in VGA are drawn double sized.  Draw this
                // one by scaling 2:1 with larger pixels
                SUIPutPointRaw2x2(0, x*2+aXOffset, y*2+aYOffset, color);
            #else
                SUIPutPointRaw(0, x+aXOffset, y+aYOffset, color);
            #endif
        }
    }
}

void LCD_DrawImageOnPage(
        unsigned char aPage,
        const unsigned char *p,
        unsigned short aXOffset,
        unsigned short aYOffset)
{
    // First, load the background into the buffer
    const unsigned char *p_palette;
    register unsigned int x, y;
    const unsigned char *p_pixels;
    unsigned int width;
    unsigned int height;
    T_pixelColor rgbArray[256];
    unsigned int i;
    const unsigned char *rgb;

    width = *((unsigned short *)(p+12));
    height = *((unsigned short *)(p+14));

    p_palette = p+18;
    p_pixels = p+18+256*3;

    // Convert the palette to native pixels
    rgb = p_palette;
    for (i=0; i<256; i++, rgb+=3)
        rgbArray[i] = RGB(rgb[2], rgb[1], rgb[0]);

    for (y=0; y<height; y++) {
        const unsigned char *p_raster = &p_pixels[(height-1-y)*width];
        for (x=0; x<width; x++) {
            T_pixelColor color = rgbArray[*(p_raster++)];
            SUIPutPointRaw(aPage, x+aXOffset, y+aYOffset, color);
        }
    }
}

