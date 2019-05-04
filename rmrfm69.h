/*
 * THE FOLLOWING FIRMWARE IS PROVIDED:
 *  (1) "AS IS" WITH NO WARRANTY;
 *  (2) TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, HopeRF SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * Copyright (C) HopeRF
 *
 * website: www.HopeRF.com
 *          www.HopeRF.cn
 */

/*!
 * file       RFduino_RFM69.h
 * brief      driver for RFM69/RFM69C TRx
 *            but also for RFM65/RFM65C Rx
 * hardware   HopeRF's RFduino TRx & HopeRF RFM69/RFM69H/RFM69C/RFM69HC/RFM65 COB rf-module
 *
 *
 * version    1.0
 * date       May 27 2014
 * author     QY Ruan
 */
/*
 * rmrfm69.h
 *
 *  Created on: 25.04.2019
 *      Author: Tadeusz Studnik http://rasmatic.pl
 */

#ifndef RMRFM69_H_
#define RMRFM69_H_

#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <stdbool.h>

typedef union
{
 struct
 	{
	 uint8_t FreqL: 8;
	 uint8_t FreqM: 8;
	 uint8_t FreqH: 8;
	 uint8_t FreqX: 8;
 	}freq;
 	uint32_t Freq;
}FreqStruct;

enum modulationType {OOK, FSK, GFSK};

enum moduleType {RFM65, RFM65C, RFM69, RFM69C, RFM69H, RFM69HC};

enum modulationType Modulation;					//OOK/FSK/GFSK
enum moduleType COB;								//Chip on board
uint32_t Frequency;							//unit: KHz
uint32_t SymbolTime;							//unit: ns
uint32_t Devation;								//unit: KHz
uint16_t  BandWidth;							//unit: KHz
uint8_t  OutputPower;							//unit: dBm   range: 0-31 [-18dBm~+13dBm] for RFM69/RFM69C
											//            range: 0-31 [-11dBm~+20dBm] for RFM69H/RFM69HC
uint16_t  PreambleLength;						//unit: uint8_t

bool  CrcDisable;							//fasle: CRC enable£¬ & use CCITT 16bit
											//true : CRC disable
bool  CrcMode;								//false: CCITT

bool  FixedPktLength;						//false: for contain packet length in Tx message, the same mean with variable lenth
                                            //true : for doesn't include packet length in Tx message, the same mean with fixed length
bool  AesOn;								//false:
                                            //true:
bool  AfcOn;								//false:
											//true:
uint8_t  SyncLength;							//unit: none, range: 1-8[uint8_t], value '0' is not allowed!
uint8_t  SyncWord[8];
uint8_t  PayloadLength;						//PayloadLength is need to be set a value, when FixedPktLength is true.
uint8_t  AesKey[16];							//AES Key block, note [0]->[15] == MSB->LSB

extern void RMRFM69Delay_us(int us);
extern uint8_t RMRFM69ReadRegSpi(uint8_t reg);
extern int RMRFM69WriteRegSpi(uint16_t value);
extern int RMRFM69SpiBurstWrite(uint8_t reg, uint8_t * wbuffer, uint8_t wlen);
extern int RMRFM69SpiBurstRead(uint8_t reg, uint8_t * rbuffer, uint8_t rlen);
extern void RMRFM69ClrPor();
extern uint8_t RMRFM69DIO0();
extern void RMRFM69SetPor();

void RMRFM69vInitialize(void);
void RMRFM69vConfig(void);
void RMRFM69vGoRx(void);
void RMRFM69vGoStandby(void);
void RMRFM69vGoSleep(void);
bool RMRFM69bSendMessage(uint8_t msg[], uint8_t length);
uint8_t RMRFM69bGetMessage(uint8_t msg[]);
void RMRFM69vRF69SetAesKey(void);
void RMRFM69vTrigAfc(void);

void RMRFM69vDirectRx(void);						//go continuous rx mode, with init. inside
void RMRFM69vChangeFreq(uint32_t freq);				//change frequency
uint8_t RMRFM69bReadRssi(void);						//read rssi value

//private
FreqStruct FrequencyValue;
uint16_t BitRateValue;
uint16_t DevationValue;
uint8_t BandWidthValue;

void RMRFM69vRF69Reset(void);
uint8_t RMRFM69bSelectBandwidth(uint8_t rx_bw);
uint8_t RMRFM69bSelectRamping(uint32_t symbol);

#endif /* RMRFM69_H_ */
