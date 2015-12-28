/*
 * sdram_is42s16320b-7.c
 *
 *  Created on: 30 de Jul de 2011
 *      Author: psampaio
 */
#include "debug_frmwrk.h"
#include "lpc177x_8x_emc.h"
#include "sdram_is42s16320b-7.h"
#include "LPC177x_8x.h"
#include "system_LPC177x_8x.h"
#include "lpc177x_8x_timer.h"
#include "lpc177x_8x_clkpwr.h"

#define SDRAM_BASE_ADDR         0xA0000000

uint32_t Dummy, i;

volatile uint32_t ringosccount[2] = {0,0};

/*****************************************************************************
** Function name:		sdram_test
**
** Descriptions:		sdram test
**
** parameters:			None
**
** Returned value:		1 if test passed, otherwise 0
**
*****************************************************************************/
uint32_t sdram_test( void )
{
  volatile uint32_t *wr_ptr;
  volatile uint16_t *short_wr_ptr;
  uint32_t data;
  uint32_t i, j;

  wr_ptr = (uint32_t *)SDRAM_BASE_ADDR;
  short_wr_ptr = (uint16_t *)wr_ptr;
  /* Clear content before 16 bit access test */
//  for (i = 0; i < SDRAM_SIZE/4; i++)
//  {
//	*wr_ptr++ = 0;
//  }

  /* 16 bit write */
  for (i = 0; i < SDRAM_SIZE/0x40000; i++)
  {
	for (j = 0; j < 0x100; j++)
	{
	  *short_wr_ptr++ = (i + j);
	  *short_wr_ptr++ = (i + j) + 1;
	}
  }

  /* Verifying */
  wr_ptr = (uint32_t *)SDRAM_BASE_ADDR;
  for (i = 0; i < SDRAM_SIZE/0x40000; i++)
  {
	for (j = 0; j < 0x100; j++)
	{
	  data = *wr_ptr;
	  if (data != (((((i + j) + 1) & 0xFFFF) << 16) | ((i + j) & 0xFFFF)))
	  {
		return 0x0;
	  }
	  wr_ptr++;
 	}
  }
  return 0x1;
}

/*****************************************************************************
** Function name:		find_cmddly
**
** Descriptions:		find CMDDLY
**
** parameters:			None
**
** Returned value:		1 if test passed, otherwise 0
**
*****************************************************************************/
uint32_t find_cmddly(void)
{
  uint32_t cmddly, cmddlystart, cmddlyend, dwtemp;
  uint32_t ppass = 0x0, pass = 0x0;

  cmddly = 0x0;
  cmddlystart = cmddlyend = 0xFF;

  while (cmddly < 32)
  {
	dwtemp = LPC_SC->EMCDLYCTL & ~0x1F;
	LPC_SC->EMCDLYCTL = dwtemp | cmddly;

	if (sdram_test() == 0x1)
	{
	  /* Test passed */
	  if (cmddlystart == 0xFF)
	  {
		cmddlystart = cmddly;
	  }
	  ppass = 0x1;
	}
	else
	{
	  /* Test failed */
	  if (ppass == 1)
	  {
		cmddlyend = cmddly;
		pass = 0x1;
		ppass = 0x0;
	  }
	}

	/* Try next value */
	cmddly++;
  }

  /* If the test passed, the we can use the average of the min and max values to get an optimal DQSIN delay */
  if (pass == 0x1)
  {
	cmddly = (cmddlystart + cmddlyend) / 2;
  }
  else if (ppass == 0x1)
  {
	cmddly = (cmddlystart + 0x1F) / 2;
  }
  else
  {
	/* A working value couldn't be found, just pick something safe so the system doesn't become unstable */
	cmddly = 0x10;
  }

  dwtemp = LPC_SC->EMCDLYCTL & ~0x1F;
  LPC_SC->EMCDLYCTL = dwtemp | cmddly;

  return (pass | ppass);
}

/*****************************************************************************
** Function name:		find_fbclkdly
**
** Descriptions:		find FBCLKDLY
**
** parameters:			None
**
** Returned value:		1 if test passed, otherwise 0
**
*****************************************************************************/
uint32_t find_fbclkdly(void)
{
  uint32_t fbclkdly, fbclkdlystart, fbclkdlyend, dwtemp;
  uint32_t ppass = 0x0, pass = 0x0;

  fbclkdly = 0x01;
  fbclkdlystart = fbclkdlyend = 0xFF;

  while (fbclkdly < 32)
  {
	dwtemp = LPC_SC->EMCDLYCTL & ~0x1F00;
	LPC_SC->EMCDLYCTL = dwtemp | (fbclkdly << 8);

	if (sdram_test() == 0x1)
	{
	  /* Test passed */
	  if (fbclkdlystart == 0xFF)
	  {
		fbclkdlystart = fbclkdly;
	  }
	  ppass = 0x1;
	}
	else
	{
	  /* Test failed */
	  if (ppass == 1)
	  {
		fbclkdlyend = fbclkdly;
		pass = 0x1;
		ppass = 0x0;
	  }
	}

	/* Try next value */
	fbclkdly++;
  }

  /* If the test passed, the we can use the average of the min and max values to get an optimal DQSIN delay */
  if (pass == 0x1)
  {
	fbclkdly = (fbclkdlystart + fbclkdlyend) / 2;
  }
  else if (ppass == 0x1)
  {
	fbclkdly = (fbclkdlystart + 0x1F) / 2;
  }
  else
  {
	/* A working value couldn't be found, just pick something safe so the system doesn't become unstable */
	fbclkdly = 0x10;
  }

  dwtemp = LPC_SC->EMCDLYCTL & ~0x1F00;
  LPC_SC->EMCDLYCTL = dwtemp | (fbclkdly << 8);

  return (pass | ppass);
}

/*****************************************************************************
** Function name:		calibration
**
** Descriptions:		Calibration
**
** parameters:			None
**
** Returned value:		current ring osc count
**
*****************************************************************************/
uint32_t calibration( void )
{
  uint32_t dwtemp, i;
  uint32_t cnt = 0;

  for (i = 0; i < 10; i++)
  {
	dwtemp = LPC_SC->EMCCAL & ~0x4000;
	LPC_SC->EMCCAL = dwtemp | 0x4000;

	dwtemp = LPC_SC->EMCCAL;
	while ((dwtemp & 0x8000) == 0x0000)
	{
	  dwtemp = LPC_SC->EMCCAL;
	}
	cnt += (dwtemp & 0xFF);
  }
  return (cnt / 10);
}

/*****************************************************************************
** Function name:		adjust_timing
**
** Descriptions:		Adjust timing
**
** parameters:			None
**
** Returned value:		None
**
*****************************************************************************/
void adjust_timing( void )
{
  uint32_t dwtemp, cmddly, fbclkdly;

  /* Current value */
  ringosccount[1] = calibration();

  dwtemp = LPC_SC->EMCDLYCTL;
  cmddly = ((dwtemp & 0x1F) * ringosccount[0] / ringosccount[1]) & 0x1F;
  fbclkdly = ((dwtemp & 0x1F00) * ringosccount[0] / ringosccount[1]) & 0x1F00;
  LPC_SC->EMCDLYCTL = (dwtemp & ~0x1F1F) | fbclkdly | cmddly;
}


/*********************************************************************
 * @brief		Calculate refresh timer (the multiple of 16 CCLKs)
 * @param[in]	freq - frequency of EMC Clk
 * @param[in]	time - micro second
 * @return 		None
 **********************************************************************/
#define EMC_SDRAM_REFRESH(freq,time)  \
  (((uint64_t)((uint64_t)time * freq)/16000000ull)+1)

/*********************************************************************
 * @brief		Calculate EMC Clock from nano second
 * @param[in]	freq - frequency of EMC Clk
 * @param[in]	time - nano second
 * @return 		None
 **********************************************************************/
uint32_t NS2CLK(uint32_t freq,uint32_t time){
 return (((uint64_t)(time)*(freq)/1000000000ull));
}


void GetValues(uint32_t emc_freq)
{
	uint32_t dynamicRP     ;
	uint32_t dynamicRAS   ;
	uint32_t dynamicSREX ;
	uint32_t dynamicAPR ;
	uint32_t dynamicDAL;
	uint32_t dynamicWR;
	uint32_t dynamicRC;
	uint32_t dynamicRFC;
	uint32_t dynamicXSR;
	uint32_t dynamicRRD;
	uint32_t dynamicMRD;
	uint32_t dynamicRefresh;

	dynamicRP         = NS2CLK(emc_freq, 20);
	dynamicRAS        = NS2CLK(emc_freq, 45);
	dynamicSREX       = NS2CLK(emc_freq, 70);
	dynamicAPR        = NS2CLK(emc_freq, 14);
	dynamicDAL        = NS2CLK(emc_freq, 35);
	dynamicWR         = NS2CLK(emc_freq, 14);
	dynamicRC         = NS2CLK(emc_freq, 68);
	dynamicRFC        = NS2CLK(emc_freq, 68);
	dynamicXSR        = NS2CLK(emc_freq, 70);
	dynamicRRD        = NS2CLK(emc_freq, 15);
	dynamicMRD        = NS2CLK(emc_freq, 15);
	dynamicRefresh    = EMC_SDRAM_REFRESH(emc_freq, 64);
	emc_freq++;
}

/*********************************************************************//**
 * @brief 		Initialize external SDRAM memory ISSI IS42S32800D
 *				256Mbit(8M x 32)
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void SDRAM_Init( void )
{
	uint32_t i, dwtemp;
	TIM_TIMERCFG_Type TIM_ConfigStruct;
	uint32_t emc_freq;



	/* Initialize EMC */
	EMC_Init();
	emc_freq = CLKPWR_GetCLK(CLKPWR_CLKTYPE_EMC);

//	GetValues(emc_freq);
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 1;

	// Set configuration for Tim_config and Tim_MatchConfig
	TIM_Init(LPC_TIM0, TIM_TIMER_MODE,&TIM_ConfigStruct);
	LPC_SC->EMCDLYCTL = 0x1010;
	//Configure memory layout, but MUST DISABLE BUFFERs during configuration
	LPC_EMC->DynamicConfig0    = 0x00004680; /* 256MB, 8Mx32, 4 banks, row=13, column=9 */
	/*Configure timing for  ISSI IS42S32800D-7 */

	//Timing for 80MHz Bus
	LPC_EMC->DynamicRasCas0    = 0x00000302; /* 1 RAS, 3 CAS latency */
	LPC_EMC->DynamicReadConfig = 0x00000001; /* Command delayed strategy, using EMCCLKDELAY */
	LPC_EMC->DynamicRP         = NS2CLK(emc_freq, 20);
	LPC_EMC->DynamicRAS        = NS2CLK(emc_freq, 45);
	LPC_EMC->DynamicSREX       = NS2CLK(emc_freq, 70);
	LPC_EMC->DynamicAPR        = NS2CLK(emc_freq, 14);
	LPC_EMC->DynamicDAL        = NS2CLK(emc_freq, 35);
	LPC_EMC->DynamicWR         = NS2CLK(emc_freq, 14);
	LPC_EMC->DynamicRC         = NS2CLK(emc_freq, 68);
	LPC_EMC->DynamicRFC        = NS2CLK(emc_freq, 68);
	LPC_EMC->DynamicXSR        = NS2CLK(emc_freq, 70);
	LPC_EMC->DynamicRRD        = NS2CLK(emc_freq, 15);
	LPC_EMC->DynamicMRD        = NS2CLK(emc_freq, 15);

	TIM_Waitms(100);						   /* wait 100ms */
	LPC_EMC->DynamicControl    = 0x00000183; /* Issue NOP command */

	TIM_Waitms(200);						   /* wait 200ms */
	LPC_EMC->DynamicControl    = 0x00000103; /* Issue PALL command */
	LPC_EMC->DynamicRefresh    = 0x00000002; /* ( n * 16 ) -> 32 clock cycles */

	for(i = 0; i < 0x80; i++);	           /* wait 128 AHB clock cycles */

	LPC_EMC->DynamicRefresh    = EMC_SDRAM_REFRESH(emc_freq, 64);

	LPC_EMC->DynamicControl    = 0x00000083; /* Issue MODE command */

	//Timing for 48/60/72MHZ Bus
	dwtemp = *((volatile uint32_t *)(SDRAM_BASE_ADDR | (0x22<<(2+2+9)))); /* 4 burst, 2 CAS latency */
	LPC_EMC->DynamicControl    = 0x00000000; /* Issue NORMAL command */

	//[re]enable buffers
	LPC_EMC->DynamicConfig0    = 0x00084680; /* 256MB, 8Mx32, 4 banks, row=12, column=9 */

//	LPC_SC->EMCDLYCTL = 0x1301; // @108MHz
//	LPC_SC->EMCDLYCTL = 0x1506; // @120MHz
#if 1
	 /* Nominal value */
	  ringosccount[0] = calibration();

	  if (find_cmddly() == 0x0)
	  {
		while (1);	/* fatal error */
	  }

	  if (find_fbclkdly() == 0x0)
	  {
		while (1);	/* fatal error */
	  }

	  adjust_timing();
#else
	  LPC_SC->EMCDLYCTL = 0x1501;
#endif
}


int SDRAM_Test(void) {
	uint32_t i;
	TIM_TIMERCFG_Type TIM_ConfigStruct;
	volatile uint16_t *short_wr_ptr;

	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 1000;

	// Set configuration for Tim_config and Tim_MatchConfig
	TIM_Init(LPC_TIM0, TIM_TIMER_MODE,&TIM_ConfigStruct);

	TIM_Cmd(LPC_TIM0, DISABLE);
	TIM_ResetCounter(LPC_TIM0);
	_DBG("Clear content of SRAM...");
	TIM_Cmd(LPC_TIM0, ENABLE);
	short_wr_ptr = (uint16_t *) SDRAM_BASE_ADDR;
	for (i = 0; i < SDRAM_SIZE / 2; i++) {
		*short_wr_ptr++ = 0;
	}
	TIM_Cmd(LPC_TIM0, DISABLE);
	_DBD32(LPC_TIM0->TC);
	short_wr_ptr = (uint16_t *) SDRAM_BASE_ADDR;
	_DBG_("\nWriting in 16 bits format...");
	for (i = 0; i < (SDRAM_SIZE / 2); i++) {
		*short_wr_ptr++ = (uint16_t) i;
	}

	_DBG_("Verifying data...");
	short_wr_ptr = (uint16_t *) SDRAM_BASE_ADDR;
	for (i = 0; i < (SDRAM_SIZE / 2); i++) {
		if (*short_wr_ptr != (uint16_t) i) /* be aware of endianess */
		{
			/* 16-bit half word failure */
			_DBG_("********** Verifying fail, testing termintated!");
			while (1)
				; /* fatal error */
		}
		short_wr_ptr++;
	}_DBG_("No errors, testing termintated!");
	return 0;
}

/*
 *
 */

typedef unsigned datum; /* Set the data bus width to 8 bits. */

#define BASE_ADDRESS  (volatile datum *) 0xA0000000
#define NUM_BYTES     64 * 1024 * 1024

/**********************************************************************
 *
 * Function:    memTestDataBus()
 *
 * Description: Test the data bus wiring in a memory region by
 *              performing a walking 1's test at a fixed address
 *              within that region.  The address (and hence the
 *              memory region) is selected by the caller.
 *
 * Notes:
 *
 * Returns:     0 if the test succeeds.
 *              A non-zero result is the first pattern that failed.
 *
 **********************************************************************/
datum memTestDataBus(volatile datum * address) {
	datum pattern;
	/*
	 * Perform a walking 1's test at the given address.
	 */
	for (pattern = 1; pattern != 0; pattern <<= 1) {
		/*
		 * Write the test pattern.
		 */
		*address = pattern;
		/*
		 * Read it back (immediately is okay for this test).
		 */
		if (*address != pattern) {
			return (pattern);
		}
	}
	return (0);
} /* memTestDataBus() */

/**********************************************************************
 *
 * Function:    memTestAddressBus()
 *
 * Description: Test the address bus wiring in a memory region by
 *              performing a walking 1's test on the relevant bits
 *              of the address and checking for aliasing. This test
 *              will find single-bit address failures such as stuck
 *              -high, stuck-low, and shorted pins.  The base address
 *              and size of the region are selected by the caller.
 *
 * Notes:       For best results, the selected base address should
 *              have enough LSB 0's to guarantee single address bit
 *              changes.  For example, to test a 64-Kbyte region,
 *              select a base address on a 64-Kbyte boundary.  Also,
 *              select the region size as a power-of-two--if at all
 *              possible.
 *
 * Returns:     NULL if the test succeeds.
 *              A non-zero result is the first address at which an
 *              aliasing problem was uncovered.  By examining the
 *              contents of memory, it may be possible to gather
 *              additional information about the problem.
 *
 **********************************************************************/
datum *
memTestAddressBus(volatile datum * baseAddress, unsigned long nBytes) {
	unsigned long addressMask = (nBytes / sizeof(datum) - 1);
	unsigned long offset;
	unsigned long testOffset;
	datum pattern = (datum) 0xAAAAAAAA;
	datum antipattern = (datum) 0x55555555;
	/*
	 * Write the default pattern at each of the power-of-two offsets.
	 */
	for (offset = 1; (offset & addressMask) != 0; offset <<= 1) {
		baseAddress[offset] = pattern;
	}
	/*
	 * Check for address bits stuck high.
	 */
	testOffset = 0;
	baseAddress[testOffset] = antipattern;
	for (offset = 1; (offset & addressMask) != 0; offset <<= 1) {
		if (baseAddress[offset] != pattern) {
			return ((datum *) &baseAddress[offset]);
		}
	}
	baseAddress[testOffset] = pattern;
	/*
	 * Check for address bits stuck low or shorted.
	 */
	for (testOffset = 1; (testOffset & addressMask) != 0; testOffset <<= 1) {
		baseAddress[testOffset] = antipattern;
		if (baseAddress[0] != pattern) {
			return ((datum *) &baseAddress[testOffset]);
		}
		for (offset = 1; (offset & addressMask) != 0; offset <<= 1) {
			if ((baseAddress[offset] != pattern) && (offset != testOffset)) {
				return ((datum *) &baseAddress[testOffset]);
			}
		}
		baseAddress[testOffset] = pattern;
	}
	return (NULL);
} /* memTestAddressBus() */

/**********************************************************************
 *
 * Function:    memTestDevice()
 *
 * Description: Test the integrity of a physical memory device by
 *              performing an increment/decrement test over the
 *              entire region.  In the process every storage bit
 *              in the device is tested as a zero and a one.  The
 *              base address and the size of the region are
 *              selected by the caller.
 *
 * Notes:
 *
 * Returns:     NULL if the test succeeds.  Also, in that case, the
 *              entire memory region will be filled with zeros.
 *
 *              A non-zero result is the first address at which an
 *              incorrect value was read back.  By examining the
 *              contents of memory, it may be possible to gather
 *              additional information about the problem.
 *
 **********************************************************************/
datum *
memTestDevice(volatile datum * baseAddress, unsigned long nBytes) {
	unsigned long offset;
	unsigned long nWords = nBytes / sizeof(datum);
	datum pattern;
	datum antipattern;
	/*
	 * Fill memory with a known pattern.
	 */
	for (pattern = 1, offset = 0; offset < nWords; pattern++, offset++) {
		baseAddress[offset] = pattern;
	}
	/*
	 * Check each location and invert it for the second pass.
	 */
	for (pattern = 1, offset = 0; offset < nWords; pattern++, offset++) {
		if (baseAddress[offset] != pattern) {
			return ((datum *) &baseAddress[offset]);
		}
		antipattern = ~pattern;
		baseAddress[offset] = antipattern;
	}
	/*
	 * Check each location for the inverted pattern and zero it.
	 */
	for (pattern = 1, offset = 0; offset < nWords; pattern++, offset++) {
		antipattern = ~pattern;
		if (baseAddress[offset] != antipattern) {
			return ((datum *) &baseAddress[offset]);
		}
	}
	return (NULL);
} /* memTestDevice() */

/**********************************************************************
 *
 * Function:    memTest()
 *
 * Description: Test a 64-k chunk of SRAM.
 *
 * Notes:
 *
 * Returns:     0 on success.
 *              Otherwise -1 indicates failure.
 *
 **********************************************************************/
int SDRAM_FullTest(void) {
	if ((memTestDataBus(BASE_ADDRESS) != 0) || (memTestAddressBus(BASE_ADDRESS,
	NUM_BYTES) != NULL) || (memTestDevice(BASE_ADDRESS, NUM_BYTES) != NULL)) {
		return (-1);
	} else {
		return (0);
	}

} /* memTest() */
