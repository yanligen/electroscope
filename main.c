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
#include "M051Series.h"

#define PLL_CLOCK       50000000

#define Board_PC	P36
#define BIST_PC		P24
#define BIST_BT		P12
#define ALERT_LED	P04
#define ALERT_BEEP	P07

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
	Board_PC = 0; // Default Low Level

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
	/* Open Timer0 frequency to 50 Hz in periodic mode, and enable interrupt */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 50);
    TIMER_EnableInt(TIMER0);

	/* Enable Timer0 ~ Timer3 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);
}

void ADCInit(void)
{
    /* Power on ADC module */
    ADC_POWER_ON(ADC);

    /* Set the ADC operation mode as single, input mode as single-end and enable the analog input channel 4 and 5 */
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE_CYCLE, 0x3 << 4);

    /* clear the A/D interrupt flag for safe */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
}

void ADCTest(void)
{
	  uint32_t channel;
    int32_t  conversionData;
	
    /* clear the A/D interrupt flag for safe */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
	
    /* start A/D conversion */
    ADC_START_CONV(ADC);

    /* Wait conversion done */
    while(!ADC_GET_INT_FLAG(ADC, ADC_ADF_INT));
	
	  for(channel = 4; channel < 6; channel++)
    {
        conversionData = ADC_GET_CONVERSION_DATA(ADC, channel);
        printf("Conversion result of channel %d: 0x%X (%d)\n", channel, conversionData, conversionData);
    }
}

/* Main function  */
int main(void)
{
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

	/* Init Timer*/
	TIMERInit();

    /* Init PWM channel 3 */
	if (BIST_BT) {
		BIST_PC = 0;
		PWMInit();
	}

    /* Init ADC to get the value of variable resistor */
    // ADCInit();
	
	//  ADCTest();

	TIMER_Start(TIMER0);

    while(1)
    {
    	CLK_SysTickLongDelay(1000000);
			ALERT_LED = ~ALERT_LED;
			ALERT_BEEP = ~ALERT_BEEP;
			printf("test led\r\n");
    }
}

/*** (C) COPYRIGHT 2022 YanLigen. ***/
