


/**************************************************************************//**
 * @file     mcp41100.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 22/07/26 9:07p $
 * @brief    MCP SW SPI Driver
 * @auther   YanLigen
 * @note
 * NO License
 *
 * Copyright (C) 2022 Nothing. All rights reserved.
 *****************************************************************************/


#ifndef __MCP41100_H__
#define __MCP41100_H__

#include "M051Series.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define SPI_CS		P31
#define SPI_CLK		P32
#define SPI_MOSI 	P30

/* MCP CMD */
#define WRITEDATA	0x11
#define SHUTDOWN	0x20

#ifdef __cplusplus
}
#endif
#endif
