/******************************************************************************
 * @file     main.c
 * @version  V0.10
 * $Revision: 2 $
 * $Date: 22/05/01 11:43a $
 * @brief    ElectroScope
 *
 * @note
 * Copyright (C) 2022 YanLigen. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "board.h"
#include "fft_lut.h"
#include "M051Series.h"

#define PLL_CLOCK       50000000

uint8_t g_boardPwrOn = 0;
uint8_t g_alertOn = 0;
uint32_t g_alertCounter = 50;
uint8_t g_noSignalCount = 0;
void SYSInit(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz clock */
//    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
//    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

	CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLK_S_HCLK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART1_MODULE);

	/* Enable TIME module clock, TIME0,1,2,3 */
	CLK_EnableModuleClock(TMR0_MODULE);
	CLK_EnableModuleClock(TMR1_MODULE);
	CLK_EnableModuleClock(TMR2_MODULE);
	CLK_EnableModuleClock(TMR3_MODULE);

    /* Enable PWM module clock */
    CLK_EnableModuleClock(PWM67_MODULE);

    /* Enable ADC module clock */
    CLK_EnableModuleClock(ADC_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART_S_PLL, CLK_CLKDIV_UART(1));

	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HCLK, 0);
	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1_S_HCLK, 0);
	CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2_S_HCLK, 0);
	CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3_S_HCLK, 0);

    /* Select PWM module clock source */
    CLK_SetModuleClock(PWM67_MODULE, CLK_CLKSEL2_PWM67_S_HCLK, 0);
    /* Reset PWMA channel4~channel7 */
    SYS_ResetModule(PWM47_RST);

    /* ADC clock source is 22.1184MHz, set divider to 7, ADC clock is 22.1184/7 MHz */
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_ADC_S_HIRC, CLK_CLKDIV_ADC(7));

	SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P0 multi-function pins for UART1 RXD and TXD */
    SYS->P0_MFP &= ~(SYS_MFP_P00_Msk | SYS_MFP_P01_Msk);
    SYS->P0_MFP |= SYS_MFP_P00_TXD1 | SYS_MFP_P01_RXD1;

	/*Set P0 multi-function pins for GPIO4,7*/
	SYS->P0_MFP &= ~(SYS_MFP_P04_Msk | SYS_MFP_P07_Msk);
    SYS->P0_MFP |= SYS_MFP_P04_GPIO| SYS_MFP_P07_GPIO;

    /* Set P2 multi-function pins for PWMA Channel6 */
    SYS->P2_MFP &= ~(SYS_MFP_P26_Msk);
    SYS->P2_MFP |= SYS_MFP_P26_PWM6;


    /* Disable the P1.4 - P1.5 digital input path to avoid the leakage current */
    GPIO_DISABLE_DIGITAL_PATH(P1, 0x30);

    /* Configure the P1.4 - P1.5 ADC analog input pins */
    SYS->P1_MFP &= ~(SYS_MFP_P14_Msk | SYS_MFP_P15_Msk);
    SYS->P1_MFP |= SYS_MFP_P14_AIN4 | SYS_MFP_P15_AIN5;

}

void PWMInit(void)
{
	/* Enable PWM Output pin */
	PWM_EnableOutput(PWMB, 0x4);
	
	PWM_ConfigOutputChannel(PWMB, PWM_CH2, 50, 50);

	printf("PPR 0x%x\r\n", (PWMB)->PPR);
	printf("PPR 0x%x\r\n", (PWMB)->CSR);

	
	/* Enable PWM Timer */
	PWM_Start(PWMB, 0x4);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART1Init(void)
{
    /* Reset IP */
    SYS_ResetModule(UART1_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART1, 115200);
}

void GPIOInit(void)
{
	/*3.3V PowerControl Configure P3.6 as Output mode*/
	GPIO_SetMode(P3, BIT6, GPIO_PMD_OUTPUT);
	BOARD_PC = 0; // Default Low Level

	/*BIST PowerControl Configure P2.4 as Output mode*/
	GPIO_SetMode(P2, BIT4, GPIO_PMD_OUTPUT);
	BIST_PC = 1; // Low Level Open, Default Close

	/*BIST Button Configure P1.2 as Input mode, HighLevel is valid*/
	GPIO_SetMode(P1, BIT2, GPIO_PMD_INPUT);

	/*SPI CS Configure P3.1 as Output mode*/
	GPIO_SetMode(P3, BIT1, GPIO_PMD_OUTPUT);
	SPI_CS = 1; // Low Level Valid, Default High
	/*SPI CLK Configure P3.2 as Output mode*/
	GPIO_SetMode(P3, BIT2, GPIO_PMD_OUTPUT);
	/*SPI MOSI Configure P3.0 as Output mode*/
	GPIO_SetMode(P3, BIT0, GPIO_PMD_OUTPUT);
	
	/*ALERT LED Configure P0.4 as Output mode*/
	GPIO_SetMode(P0, BIT4, GPIO_PMD_OUTPUT);
	ALERT_LED = 0; // High Level Open, Default Close

	/*ALERT BEEP Configure P0.7 as Output mode*/
	GPIO_SetMode(P0, BIT7, GPIO_PMD_OUTPUT);
	ALERT_BEEP = 0; // High Level Open, Default Close

}

void TIMERInit(void)
{
    uint32_t u32Freq;

	/* Open Timer0 time to 10ms  in periodic mode, and enable interrupt */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 100);
    TIMER_EnableInt(TIMER0);
	TIMER_Start(TIMER0); // Timer555 Power On

	/* Open Timer1 time to 1/SAMPLE_RATE s in periodic mode, and enable interrupt */
    u32Freq = TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, SAMPLE_RATE);
	printf("Time1 Frequency = %d\n\r", u32Freq);

    if (u32Freq != SAMPLE_RATE) {
        printf("# Warning. Sample rate does not equal to the pre-defined setting.\n\r");
    }
    TIMER_EnableInt(TIMER1);


	/* Open Timer2 time to 1s in periodic mode, and enable interrupt */
//    TIMER_Open(TIMER2, TIMER_PERIODIC_MODE, 2);
    TIMER_EnableInt(TIMER2);

	/* Enable Timer0 ~ Timer1 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);
	NVIC_EnableIRQ(TMR1_IRQn);
//	NVIC_EnableIRQ(TMR2_IRQn);
}

void ADCInit(void)
{
    /* Power on ADC module */
    ADC_POWER_ON(ADC);

    /* Set the ADC operation mode as single, input mode as single-end and enable the analog input channel 4 and 5 */
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, 0x3 << 4);

    /* clear the A/D interrupt flag for safe */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
}

void CloseSignalAlert(void)
{
	g_alertOn = 0;
	ALERT_LED = 0;
	ALERT_BEEP = 0;
}


/* Main function  */
bool BatterySample(void)
{
	int32_t batteryConvData;
	batteryConvData = ADCConversionFromChannel(BATTERY_CH);
	// printf("Battery Conversion: 0x%x (%d)\r\n", batteryConvData, batteryConvData);

	if (batteryConvData < 2122) { // LP2980 VIN-VO = 120mV
		// Add Alert
		CloseSignalAlert();
		g_boardPwrOn = 0;
		BOARD_PC= 0;
		return FALSE;
	}

	return TRUE;
}

bool IsBoardPowerOn(void)
{
	if (g_boardPwrOn) {
		return TRUE;
	}
	return FALSE;
}


void CheckNoSignalState(uint8_t signalLevel)
{
	if (signalLevel == 0 && IsBoardPowerOn()) {
		g_noSignalCount++;
		if (g_noSignalCount > 4) {
			g_noSignalCount = 0;
			printf("No Signal PowerOff\r\n");
//			g_boardPwrOn = 0; // Timer555 Power Off
//			BOARD_PC = 0;
		}
	}
}

uint8_t CheckSignalQuality4Intensity(int32_t intensity)
{
	uint8_t signalLevel = 0;
	float voltage = (float)intensity * 3300 / 1024.0;
	float temp = voltage / g_ampGear; // unit mV
	if (temp > 300)
		signalLevel = 3;
		else if (temp > 150)
			signalLevel = 2;
			else if (temp > 5)
				signalLevel = 1;
				else
					signalLevel = 0;
	return signalLevel;
}

uint8_t CheckSignalQuality3Intensity(int32_t intensity)
{
	uint8_t signalLevel = 0;
	float voltage = (float)intensity * 3300 / 1024.0;
	float temp = voltage / g_ampGear; // unit mV
	if (temp > 350)
		signalLevel = 3;
		else if (temp > 200)
			signalLevel = 2;
			else if (temp > 50)
				signalLevel = 1;
				else
					signalLevel = 0;
	return signalLevel;
}

uint8_t CheckSignalQuality2Intensity(int32_t intensity)
{
	uint8_t signalLevel = 0;
	float voltage = (float)intensity * 3300 / 1024.0;
	float temp = voltage / g_ampGear;
	if (temp > 400)
		signalLevel = 3;
		else if (temp > 250)
			signalLevel = 2;
			else if (temp > 100)
				signalLevel = 1;
				else
					signalLevel = 0;
	return signalLevel;
}

uint8_t CheckSignalQuality1Intensity(int32_t intensity)
{
	uint8_t signalLevel = 0;
	float voltage = (float)intensity * 3300 / 1024.0;
	float temp = voltage / g_ampGear;
	if (temp > 500)
		signalLevel = 3;
		else if (temp > 350)
			signalLevel = 2;
			else if (temp > 200)
				signalLevel = 1;
				else
					signalLevel = 0;
	return signalLevel;
}

bool DCCheck(int32_t max, int32_t min)
{
	if (max - min < 5) {
		return TRUE;
	}
	return FALSE;
}

void AutoAdjustGear(int32_t max, int32_t min)
{
	float average = (float)(max + min) / 2.0;
	float attitude = ((float)max - average) * 3.3 / 1024.0;
	float temp, radio;
	
	printf("max,min= %d,%d average = %f, voltage = %f \r\n", max, min, average, attitude);
	if (attitude == 0) {
		radio = 5.0; // max radio
	} else {
		temp = 1.2 / attitude; // Standard Voltage Is 1.2 V
		radio = temp * g_ampGear;
	}
	AdjustGear(radio);
	printf("temp = %f, radio = %f, gear = %f \r\n", temp, radio, g_ampGear);
}

uint8_t CheckSignal(void)
{
	uint8_t signalLevel = 0;
	// According Amp Gare To Calculate Signal Attidute
	if (g_signalInfo.radio > 35) // 50Hz Signal Quality Is Perfect
		signalLevel = CheckSignalQuality4Intensity(g_signalInfo.signalIntensity);
	else if (g_signalInfo.radio > 20) // 50Hz Signal Quality Is Excellent
		signalLevel = CheckSignalQuality3Intensity(g_signalInfo.signalIntensity);
		else if (g_signalInfo.radio > 15) // 50Hz Signal Quality Is Good
			signalLevel = CheckSignalQuality2Intensity(g_signalInfo.signalIntensity);
			else if (g_signalInfo.radio > 10) // 50Hz Signal Quality Is Pass
				signalLevel = CheckSignalQuality1Intensity(g_signalInfo.signalIntensity);
				else
					signalLevel = 0;
	AutoAdjustGear(g_signalInfo.attitudeMax, g_signalInfo.attitudeMin);
	return signalLevel;
}

const uint8_t altetLevel[3] = {50, 25, 12};

void OpenSignalAlert(uint8_t signalLevel)
{
	static uint8_t level = 1;
	if (signalLevel != level) {
		g_alertCounter = altetLevel[signalLevel - 1];
		level = signalLevel;
	}
	g_alertOn = 1;
}


int main(void)
{
	uint8_t mainState = 0;
	uint8_t signalLevel = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYSInit();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART1Init();

    printf("System clock rate: %d Hz\r\n", SystemCoreClock);

	/*Init Board GPIO*/
	GPIOInit();

	/*SetMCP41100Gear*/
	SetGear(g_ampGear);

	/* Init Timer*/
	TIMERInit();

    /* Init PWM channel 3 For Bist*/
	if (BIST_BT) {
		BIST_PC = 0;
		PWMInit();
	}

	/*BoardPower On*/
	g_boardPwrOn = 1;
	
    /* Init ADC to get the value of variable resistor */
    ADCInit();

    while (1) {
    	switch (mainState) {
		case 0:
			StartSignalSample(TIMER1);
			while (!g_sampleOver) {
				BatterySample();
			}
			g_sampleOver = 0;
			mainState = 1;
			break;
		case 1:
			FFT();
			signalLevel = CheckSignal();
			if (signalLevel > 0) {
				g_noSignalCount = 0;
				mainState = 2;
			} else {
				printf("have no signal \r\n");
				CloseSignalAlert();
				CheckNoSignalState(signalLevel);
				mainState = 0;
			}
			BatterySample();
			break;
		case 2:
			printf("Enable 555 signalLevel = %d \r\n", signalLevel);
			// g_boardPwrOn = 1;
			OpenSignalAlert(signalLevel);
			mainState = 0;
			break;
		default:
			mainState = 0;
			break;
		}
    }
}

/*** (C) COPYRIGHT 2022 YanLigen. ***/
