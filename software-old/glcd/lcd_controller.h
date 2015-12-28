/*
 * lcd_controller.h
 *
 *  Created on: 31 de Jul de 2011
 *      Author: psampaio
 */

#ifndef LCD_CONTROLLER_H_
#define LCD_CONTROLLER_H_

typedef enum {
    LCD_STANDARD_TFT=0,
    LCD_ADVANCED_TFT,
    LCD_HIGHLY_REFLECTIVE_TFT,
    LCD_4BIT_MONO,
    LCD_8BIT_MONO,
    LCD_COLOR_STN
} T_lcdPanelType;

typedef enum {
	LCD_COLOR_RES_1=0,
	LCD_COLOR_RES_2,
	LCD_COLOR_RES_4,
	LCD_COLOR_RES_8,
	LCD_COLOR_RES_12_444,   // uses 16 bits to store 12 levels  0:R4:G4:B4
	LCD_COLOR_RES_16_I555,  // first bit is intensity bit (1=high/0=low)
	LCD_COLOR_RES_16_565,
	LCD_COLOR_RES_24,
} T_lcdColorResolution;


typedef enum {
    LCD_COLOR_ORDER_RGB,
    LCD_COLOR_ORDER_BGR,
} T_lcdColorOrder;

typedef struct {
    // LCD Panel type
    T_lcdPanelType iPanelType;

    // Pixels
    T_lcdColorResolution iColorResolution;

    // Horizontal settings
    unsigned short iHorizontalBackPorch;
    unsigned short iHorizontalFrontPorch;
    unsigned short iHorizontalSyncPulseWidth;
    unsigned short iHorizontalPixelsPerLine;

    // Vertical Settings
    unsigned short iVerticalBackPorch;
    unsigned short iVerticalFrontPorch;
    unsigned short iVerticalSyncPulseWidth;
    unsigned short iVerticalLinesPerPanel;

    // Line end control
    unsigned short iLineEndDelay; // 0 = disabled

    // Line polarity
    int iInvertOutputEnable;
    int iInvertPanelClock;
    int iInvertHorizontalSync;
    int iInvertVerticalSync;

    // Bias
    unsigned short iACBiasPinFrequency;

    // Misc
    int iIsDualPanel;
    int iIsBigEndian;
    int iIsRightToLeftPixels;
    int iIsBottomToTopLines;
    T_lcdColorOrder iColorOrder;
//    unsigned int iBaseAddress;

    // Need dot clock here!
    unsigned int iDotClockHz;
} T_LCDControllerSettings;


//#define LCD_POL_IVS             (1<<11)
//#define LCD_POL_IHS             (1<<12)
//#define LCD_POL_IPC             (1<<13)
//#define LCD_POL_IOE             (1<<14)
#define LCD_CTRL_LcdBpp_BIT     1
#define LCD_CTRL_LcdEn          (1<<0)
#define LCD_CTRL_LcdBW          (1<<4)
#define LCD_CTRL_LcdTFT         (1<<5)
#define LCD_CTRL_LcdMon8        (1<<6)
#define LCD_CTRL_LcdDual        (1<<7)
#define LCD_CTRL_BGR            (1<<8)
#define LCD_CTRL_LcdPwr         (1<<11)

#define PROCESSOR_OSCILLATOR_FREQUENCY 96000000
typedef unsigned int T_pixelColor;

#define UEZ_LCD_DISPLAY_WIDTH 	320
#define UEZ_LCD_DISPLAY_HEIGHT	240


#define RGB(r, g, b)      \
    ( ((r & 0xff)<<16)| \
      ((g & 0xff)<<8)| \
      ((b & 0xff)<<0) )

int LCD_ControllerConfigure(T_LCDControllerSettings *aSettings);
void LCD_Clear(unsigned int rgb);
void LCD_Off(void);
void LCD_On(void);
void LCD_DrawIcon(
        const unsigned char *p,
        unsigned short aXOffset,
        unsigned short aYOffset);
void LCD_DrawImageOnPage(
        unsigned char aPage,
        const unsigned char *p,
        unsigned short aXOffset,
        unsigned short aYOffset)  __attribute__((section(".startup")));

#endif /* LCD_CONTROLLER_H_ */
