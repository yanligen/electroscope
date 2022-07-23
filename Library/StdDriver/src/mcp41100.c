/**************************************************************************//**
 * @file     mcp41100.c
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

#include "M051Series.h"

void SpiSendByte(uint8_t data)
{
    uint8_t i = 0;
    uint8_t tmp = data;
    for (i = 0; i < 8; i++)
    {
        SPI_CLK = 0;
        CLK_SysTickDelay(1);
        if (tmp & 0x80) {
            SPI_MOSI = 1;
        } else {
            SPI_MOSI = 0;
        }
        CLK_SysTickDelay(1);
        SPI_CLK = 1;
		CLK_SysTickDelay(1);
        tmp <<= 1;
        CLK_SysTickDelay(1);
    }
}


//uint16_t SpiRecvByte(void)
//{
//    uint16_t data = 0;
//    uint8_t i = 0;

//    SPI_CLK = 0;
//    for (i = 0; i < 16; i++)
//    {
//        sleep_us(1);
//        data <<= 1;
//        sleep_us(1);
//        SPI_CLK = 1;
//        sleep_us(1);
//        if (SPI_MISO)
//        {
//            data |= 0x01;
//        }
//        sleep_us(1);
//        SPI_CLK = 0;
//    }

//    return data;
//}


void SendMcpWriteData(uint8_t data)
{
	uint8_t cmd = WRITEDATA;
    
    SPI_CLK = 0;
    SPI_CS = 0;
	CLK_SysTickDelay(1);
    SpiSendByte(cmd);
    SpiSendByte(data);
	SPI_CLK = 0;
	CLK_SysTickDelay(1);
    SPI_CS = 1;
	CLK_SysTickDelay(1);
}