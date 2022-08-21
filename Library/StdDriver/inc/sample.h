/**************************************************************************//**
 * @file     sample.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 22/07/26 9:07p $
 * @brief     Sample Applition
 * @auther   YanLigen
 * @note
 * NO License
 *
 * Copyright (C) 2022 Nothing. All rights reserved.
 *****************************************************************************/

#ifndef __SAMPLE_H__
#define __SAMPLE_H__

#include "M051Series.h"
#include "fft_lut.h"

#ifdef __cplusplus
extern "C"
{
#endif
#define SAMPLE_RATE	(512)
/* The parameter SAMPLE_DATA_RSHIFT is configurable, the purpose is to truncate
 * the data size by right shifting the raw ADC data, and it can prevent the buffer
 * from overflow when calculating a large amount N point of FFT. The longer data
 * length is selected, the higher SAMPLE_DATA_RSHIFT is required. If the spectrum
 * buffer is overflow, try to increase the SAMPLE_DATA_RSHIFT parameter.
 */
#define SAMPLE_DATA_RSHIFT          (2)
#define SHRINK(x)                   ((x) >> SAMPLE_DATA_RSHIFT)

/* ADC Channel DEFINE */
#define SIGNAL_CH	0x4
#define BATTERY_CH	0x5

/* Macros to simplify the math presentations for FFT algorithm */
#define COS(angle_index)            (g_ai16cos[angle_index])
#define SIN(angle_index)            (g_ai16sin[angle_index])
#define REV(origin_index)           (g_ai16rev[origin_index])

typedef struct _signal_info {
	int32_t signalIntensity;
	int32_t radio;
	int32_t attitudeMax;
	int32_t attitudeMin;
} SIGNAL_INFO;

int32_t ADCConversionFromChannel(uint32_t channel);
void StartSignalSample(TIMER_T *timer);
void FFT(void);

extern int32_t g_i32RealBuffer [SEQUENCE_LEN];
extern int32_t g_i32ImageBuffer[SEQUENCE_LEN];
extern int32_t g_i32Spectrum   [SEQUENCE_LEN];

/* Look-up table for FFT twiddle factors and the bit-reversal index */
extern const int16_t g_ai16cos[SEQUENCE_LEN];
extern const int16_t g_ai16sin[SEQUENCE_LEN];
extern const int16_t g_ai16rev[SEQUENCE_LEN];

extern SIGNAL_INFO g_signalInfo;
extern uint8_t g_sampleOver;
#ifdef __cplusplus
}
#endif
#endif
