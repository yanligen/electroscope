/**************************************************************************//**
 * @file     sample.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 22/07/26 9:07p $
 * @brief    Sample Applition
 * @auther   YanLigen
 * @note
 * NO License
 *
 * Copyright (C) 2022 Nothing. All rights reserved.
 *****************************************************************************/

#include "stdio.h"
#include "math.h"
#include "M051Series.h"

uint8_t g_sampleOver = 0;


int32_t ADCConversionFromChannel(uint32_t channel)
{
    int32_t  conversionData;
	
    /* clear the A/D interrupt flag for safe */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

	/* set channel */
	ADC_SET_INPUT_CHANNEL(ADC, 0x1 << channel);
	
    /* start A/D conversion */
    ADC_START_CONV(ADC);

    /* Wait conversion done */
    while(!ADC_GET_INT_FLAG(ADC, ADC_ADF_INT));

	/* read conversion data*/
    conversionData = ADC_GET_CONVERSION_DATA(ADC, channel);
    // printf("Conversion result of channel %d: 0x%x (%d)\r\n", channel, conversionData, conversionData);

	return conversionData;
}

void StartSignalSample(TIMER_T *timer)
{
	if (!((timer->TCSR) & TIMER_TCSR_CEN_Msk)) {
		TIMER_Start(TIMER1);
	}
}

/* Look-up table for FFT twiddle factors and the bit-reversal index */
const int16_t g_ai16cos[SEQUENCE_LEN] = COS_LUT;
const int16_t g_ai16sin[SEQUENCE_LEN] = SIN_LUT;
const int16_t g_ai16rev[SEQUENCE_LEN] = REV_LUT;


/* The data buffer for storing the ADC data samples and the output spectrum */
int32_t g_i32RealBuffer [SEQUENCE_LEN];
int32_t g_i32ImageBuffer[SEQUENCE_LEN];
int32_t g_i32Spectrum   [SEQUENCE_LEN];

/*----------------------------------------------------------------------------------------------------------*/
/*  Function:    NuFFT                                                                                      */
/*  Parameters:  pi32RealBuffer  is the pointer to the real  part of the data buffer.                       */
/*  Parameters:  pi32ImageBuffer is the pointer to the image part of the data buffer.                       */
/*  Parameters:  u32Len indicates the length of the data sequence.                                          */
/*  Returns:     None.                                                                                      */
/*  Description: Fast Fourier Transform Algorithm. Transform the time-domain data                           */
/*               sequence into a frequency-domain spectrum.                                                 */
/*----------------------------------------------------------------------------------------------------------*/
void NuFFT(int32_t *pi32RealBuffer, int32_t *pi32ImageBuffer, uint32_t u32Len)
{
    const uint32_t N = u32Len;
    const uint32_t u32TotalStage = log(N) / log(2);

    uint32_t u32Stage, u32Span, u32Node, u32Twiddle, u32Angle;
    int32_t i32X1Real, i32X1Image, i32X2Real, i32X2Image;

    /* Iterations for log2(N) FFT butterfly stages */
    for (u32Stage = 0; u32Stage < u32TotalStage; u32Stage++)
    {
        /* Span indicates the buffer index for constituting a butterfly matrix */
        u32Span = pow(2, u32Stage);

        for (u32Twiddle = 0; u32Twiddle < u32Span; u32Twiddle++)
        {
            u32Angle = (N >> 1) / u32Span * u32Twiddle;

            for (u32Node = u32Twiddle; u32Node < N; u32Node += 2 * u32Span)
            {
                /* Get the real and image part of the input variable X1 */
                i32X1Real  = pi32RealBuffer [u32Node];
                i32X1Image = pi32ImageBuffer[u32Node];

                /* Get the real and image part of the input variable X2 */
                i32X2Real  = pi32RealBuffer [u32Node + u32Span];
                i32X2Image = pi32ImageBuffer[u32Node + u32Span];

                /* Y1 = X1 + X2 * twiddle factor
                 * The real part is   real * cos() + image * sin()
                 * The image part is -real * sin() + image * cos()
                 */
                pi32RealBuffer [u32Node] = i32X1Real  + RESCALE(i32X2Real * COS(u32Angle)) + RESCALE(i32X2Image * SIN(u32Angle));
                pi32ImageBuffer[u32Node] = i32X1Image - RESCALE(i32X2Real * SIN(u32Angle)) + RESCALE(i32X2Image * COS(u32Angle));

                /* Y2 = X1 - X2 * twiddle factor
                 * The real part is  -real * cos() - image * sin()
                 * The image part is  real * sin() - image * cos()
                 */
                pi32RealBuffer [u32Node + u32Span] = i32X1Real  - RESCALE(i32X2Real * COS(u32Angle)) - RESCALE(i32X2Image * SIN(u32Angle));
                pi32ImageBuffer[u32Node + u32Span] = i32X1Image + RESCALE(i32X2Real * SIN(u32Angle)) - RESCALE(i32X2Image * COS(u32Angle));
            }
        }
    }
}

/*----------------------------------------------------------------------------------------------------------*/
/*  Function:    Display_Spectrum                                                                           */
/*  Parameters:  pi32Spectrum is the spectrum buffer pointer.                                               */
/*  Parameters:  u32Len is the length of the spectrum buffer.                                               */
/*  Parameters:  u32SampleRate is the frequency of TIMER trigger EADC.                                      */
/*  Returns:     none.                                                                                      */
/*  Description: Show the detail informations from the spectrum bffer.                                      */
/*----------------------------------------------------------------------------------------------------------*/
int32_t DisplaySpectrum(int *pi32Spectrum, uint32_t u32Len, uint32_t u32SampleRate)
{
    const float  fconstResolution = (float)u32SampleRate / (float)u32Len;

    int i, i32Base, i32Accumulate = 0;

    printf("+-----------------------------------------------------------+\n\r");
    printf("Spectrum Parameters         \n\r");
    printf("# Sample rate: %d Hz        \n\r", u32SampleRate);
    printf("# Data length: %d samples   \n\r", u32Len);
    printf("# Resolution : %.2f Hz      \n\r", fconstResolution);
    printf("+-----------------------------------------------------------+\n\r");
    printf("#Index     #Frequency(Hz)     #Amplitude     #Ratio \n\r");

    for (i = 1; i <= u32Len / 2; i++) {
        i32Accumulate += pi32Spectrum[i];
    }

	printf("accumulate = %d\r\n", i32Accumulate);

    for (i = 0; i <= u32Len / 2; i++) {
        i32Base = (i <= u32Len / 2) ? (i) : (i - u32Len);

        printf(" %-7d    %-15.2f    %-11d    %.2f%%\n\r", i, (float)(fconstResolution * i32Base), pi32Spectrum[i], (float)pi32Spectrum[i] / (float)i32Accumulate * 100);

    }

    printf("+-----------------------------------------------------------+\n\r");
    printf("FFT complete\n\r");
	return (int32_t)((float)pi32Spectrum[25] / (float)i32Accumulate * 100);
}

void RemoveDCComponent(int *pi32DataBuffer, uint32_t u32Len)
{
    uint32_t i;

    for (i = 0; i < u32Len; i++)
    {
        pi32DataBuffer[i] -= 512; // 1.65V
    }
}

SIGNAL_INFO g_signalInfo = {0};


void FFT(void)
{
	uint32_t i;
	int32_t temp, max, min;

	printf("real value = %d\r\n", g_i32RealBuffer[0]);
	min = g_i32RealBuffer[0];
	max = g_i32RealBuffer[0];
    for (i = 1; i < SEQUENCE_LEN ; i++) {
        printf("real value = %d\r\n", g_i32RealBuffer[i]);
		max = g_i32RealBuffer[i] > max ? g_i32RealBuffer[i] : max;
		min = g_i32RealBuffer[i] < min ? g_i32RealBuffer[i] : min;
    }
		printf("max = %d, min = %d\r\n", max, min);

	RemoveDCComponent(g_i32RealBuffer, SEQUENCE_LEN);
    /* Transform the time domain signal into frequency domain */
    NuFFT(g_i32RealBuffer, g_i32ImageBuffer, SEQUENCE_LEN);

    for (i = 0; i < SEQUENCE_LEN ; i++) {
        /* The amplitude of the spectrum could be presented as the sum of square of the real part and the complex part. */
		temp = (g_i32RealBuffer[i] * g_i32RealBuffer[i]) + (g_i32ImageBuffer[i] * g_i32ImageBuffer[i]);
		if (i == 0) {
			g_i32Spectrum[i] = (int32_t)(sqrt(((double)temp)) * 1.0 / SEQUENCE_LEN);
		} else {
			g_i32Spectrum[i] = (int32_t)(sqrt(((double)temp)) * 2.0 / SEQUENCE_LEN);
		}
        
    }

	g_signalInfo.radio = DisplaySpectrum(g_i32Spectrum, SEQUENCE_LEN, SAMPLE_RATE);
	g_signalInfo.signalIntensity = g_i32Spectrum[25];
	g_signalInfo.attitudeMax = max;
	g_signalInfo.attitudeMin = min;
}





