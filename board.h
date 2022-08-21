/**************************************************************************//**
 * @file     board.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 22/07/26 9:07p $
 * @brief    BOARD DEFINE
 * @auther   YanLigen
 * @note
 * NO License
 *
 * Copyright (C) 2022 Nothing. All rights reserved.
 *****************************************************************************/

#ifndef __BOARD_H__
#define __BOARD_H__

#include "M051Series.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* GPIO Define */
#define BOARD_PC	P36
#define BIST_PC		P24
#define BIST_BT		P12
#define ALERT_LED	P04
#define ALERT_BEEP	P07

extern uint8_t g_boardPwrOn;
extern uint8_t g_alertOn;
extern uint32_t g_alertCounter;

#ifdef __cplusplus
}
#endif
#endif
