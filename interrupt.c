#include <stdio.h>
#include "board.h"
#include "fft_lut.h"
#include "M051Series.h"

/**
 * @brief       Timer0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer0 default IRQ, declared in startup_M051Series.s.
 */
void TMR0_IRQHandler(void)
{
	static uint32_t alertCount = 0;
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);
		if (g_boardPwrOn) {
			BOARD_PC = ~BOARD_PC;
		}
				
		if (g_alertOn) {
			alertCount++; // 20ms
			if (alertCount > g_alertCounter) {
				alertCount = 0;
				ALERT_LED = ~ALERT_LED;
				ALERT_BEEP = ~ALERT_BEEP;
			}
		} else {
			alertCount = 0;
			ALERT_LED = 0;
			ALERT_BEEP = 0;
		}

    }
}

/**
 * @brief       Timer1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer1 default IRQ, declared in startup_M051Series.s.
 */


void TMR1_IRQHandler(void)
{
	static uint16_t sampleCount = 0;
    if (TIMER_GetIntFlag(TIMER1) == 1) {
        /* Clear Timer1 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER1);
			g_i32ImageBuffer[REV(sampleCount)] = 0;
	    g_i32RealBuffer[REV(sampleCount)] = SHRINK(ADCConversionFromChannel(SIGNAL_CH));
		/* The image part buffer is initialized to zeros */
    	
		sampleCount++;
		if (sampleCount >= SEQUENCE_LEN) {
			TIMER_Stop(TIMER1);
			sampleCount = 0;
			g_sampleOver = 1;
		}
    }
}

/**
 * @brief       Timer2 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer2 default IRQ, declared in startup_M051Series.s.
 */
void TMR2_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER2) == 1)
    {
        /* Clear Timer2 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER2);
    }
}

/**
 * @brief       Timer3 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer3 default IRQ, declared in startup_M051Series.s.
 */
void TMR3_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER3) == 1)
    {
        /* Clear Timer3 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER3);

    }
}
