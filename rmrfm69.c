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
 * file       RFduino_RFM69.cpp
 * brief      driver for RFM69/RFM69C TRx
 *            but also for RFM65/RFM65C Rx
 * hardware   HopeRF's RFduino TRx & HopeRF RFM69/RFM69H/RFM69C/RFM69HC/RFM65 COB rf-module
 *
 *
 * version    1.0
 * date       May 27 2014
 * author     QY Ruan
 */

/*!
 * modify 2015-3-16
 *      1. add aes function
 */
/*
 * rmrfm69.c
 *
 *  Created on: 25.04.2019
 *      Author: Tadeusz Studnik http://rasmatic.pl
 */

#include "rmrfm69.h"

/**********************************************************
**RF69 Register define
**********************************************************/
#define 	RegFifo 			0x00
#define 	RegOpMode 			0x01
#define 	RegDataModul 		0x02
#define 	RegBitrateMsb 		0x03
#define 	RegBitrateLsb 		0x04
#define 	RegFdevMsb 		    0x05
#define 	RegFdevLsb 		    0x06
#define 	RegFrMsb 			0x07
#define 	RegFrMid 			0x08
#define 	RegFrLsb 			0x09
#define 	RegOsc1 			0x0A
#define 	RegAfcCtrl	 		0x0B
#define 	RegListen1 		    0x0D
#define 	RegListen2 		    0x0E
#define 	RegListen3 		    0x0F
#define 	RegVersion 		    0x10
#define 	RegPaLevel 		    0x11
#define 	RegPaRamp 			0x12
#define 	RegOcp 			    0x13
#define 	RegLna 			    0x18
#define 	RegRxBw 			0x19
#define 	RegAfcBw 			0x1A
#define 	RegOokPeak 		    0x1B
#define 	RegOokAvg 			0x1C
#define 	RegOokFix 			0x1D
#define 	RegAfcFei 			0x1E
#define 	RegAfcMsb 			0x1F
#define 	RegAfcLsb 			0x20
#define 	RegFeiMsb 			0x21
#define 	RegFeiLsb 			0x22
#define 	RegRssiConfig 		0x23
#define 	RegRssiValue 		0x24
#define 	RegDioMapping1 	    0x25
#define 	RegDioMapping2 	    0x26
#define 	RegIrqFlags1 		0x27
#define 	RegIrqFlags2 		0x28
#define 	RegRssiThresh 		0x29
#define 	RegRxTimeout1 		0x2A
#define 	RegRxTimeout2 		0x2B
#define 	RegPreambleMsb 	    0x2C
#define 	RegPreambleLsb 	    0x2D
#define 	RegSyncConfig 		0x2E
#define 	RegSyncValue1		0x2F
#define 	RegSyncValue2       0x30
#define 	RegSyncValue3       0x31
#define 	RegSyncValue4       0x32
#define 	RegSyncValue5       0x33
#define 	RegSyncValue6       0x34
#define 	RegSyncValue7       0x35
#define 	RegSyncValue8       0x36
#define 	RegPacketConfig1 	0x37
#define 	RegPayloadLength 	0x38
#define 	RegNodeAdrs 		0x39
#define 	RegBroadcastAdrs 	0x3A
#define 	RegAutoModes 		0x3B
#define 	RegFifoThresh 		0x3C
#define 	RegPacketConfig2 	0x3D
#define 	RegAesKey1		    0x3E
#define 	RegAesKey2          0x3F
#define 	RegAesKey3          0x40
#define 	RegAesKey4          0x41
#define 	RegAesKey5          0x42
#define 	RegAesKey6          0x43
#define 	RegAesKey7          0x44
#define 	RegAesKey8          0x45
#define 	RegAesKey9          0x46
#define 	RegAesKey10         0x47
#define 	RegAesKey11         0x48
#define 	RegAesKey12         0x49
#define 	RegAesKey13         0x4A
#define 	RegAesKey14         0x4B
#define 	RegAesKey15         0x4C
#define 	RegAesKey16         0x4D
#define 	RegTemp1 			0x4E
#define 	RegTemp2 			0x4F
#define 	RegTestLna 		    0x58
#define 	RegTestPa1 		    0x5A
#define 	RegTestPa2 		    0x5C
#define 	RegTestDagc 		0x6F
#define 	RegTestAfc 		    0x71

/**********************************************************
**RF69 mode status
**********************************************************/
#define		RADIO_SLEEP			(0x00<<2)
#define		RADIO_STANDBY		(0x01<<2)
#define		RADIO_TX			(0x03<<2)
#define		RADIO_RX			(0x04<<2)

#define 	MODE_MASK			0xE3

#define		PacketMode			(0<<5)
#define		ConWithClkMode		(2<<5)
#define		ConWithoutClkMode	(3<<5)

#define		FskMode				(0<<3)
#define		OokMode				(1<<3)

#define		Shaping				2

//for PacketConfig
#define		VariablePacket		(1<<7)
#define		DcFree_NRZ			(0<<5)
#define		DcFree_MANCHESTER	(1<<5)
#define		DcFree_WHITENING	(2<<5)
#define		CrcOn				(1<<4)
#define		CrcDisAutoClear		(1<<3)
#define		AddrFilter_NONE		(0<<1)
#define		AddrFilter_NODE		(1<<1)
#define		AddrFilter_ALL		(2<<1)
#define		CrcCalc_CCITT		0x00
#define		CrcCalc_IBM			0x01


/**********************************************************
**Name:     vInitialize
**Function: initialize rfm69 or rfm69c
**Input:    none
**Output:   none
**********************************************************/
void RMRFM69vInitialize(void)
{
 RMRFM69ClrPor();
 //PORIn();
 //DIO0In();
 //Spi.vSpiInit();

 //端口初始化 for 32MHz
 FrequencyValue.Freq = (Frequency<<11)/125;		//Calc. Freq
 BitRateValue  = (SymbolTime<<5)/1000;			//Calc. BitRate
 DevationValue = (Devation<<11)/125;			//Calc. Fdev
 BandWidthValue = RMRFM69bSelectBandwidth(BandWidth);

 ////端口初始化 for 30MHz
 //FrequencyValue.Freq = (((Frequency<<12)/1875)<<3)+(((Frequency&0x00000007)<<12)/1875);//计算频率参数
 //BitRateValue  = (SymbolTime*30)/1000;			//计算速率参数
 //DevationValue = (Devation<<15)/1875;				//计算频偏参数
 //BandWidthValue = bSelectBandwidth(BandWidth);

 RMRFM69vConfig();
 RMRFM69vGoStandby();
}

/**********************************************************
**Name:     vConfig
**Function: config RF69
**Input:    none
**Output:   none
**********************************************************/
void RMRFM69vConfig(void)
{
 uint8_t i;
 uint8_t sync;
 RMRFM69vRF69Reset();
 RMRFM69vGoStandby();

 //Frequency
 RMRFM69WriteRegSpi(((uint16_t)RegFrMsb<<8)+FrequencyValue.freq.FreqH);
 RMRFM69WriteRegSpi(((uint16_t)RegFrMid<<8)+FrequencyValue.freq.FreqM);
 RMRFM69WriteRegSpi(((uint16_t)RegFrLsb<<8)+FrequencyValue.freq.FreqL);

 // Mode
 switch(Modulation)
 	{
 	case OOK:
 		RMRFM69WriteRegSpi(((uint16_t)RegDataModul<<8)+PacketMode+OokMode+Shaping);
 		break;
 	case GFSK:
 		RMRFM69WriteRegSpi(((uint16_t)RegDataModul<<8)+PacketMode+FskMode+Shaping);
 		break;
 	case FSK:
 	default:
 		RMRFM69WriteRegSpi(((uint16_t)RegDataModul<<8)+PacketMode+FskMode);
 		break;
 	}

 //BitRate
 RMRFM69WriteRegSpi(((uint16_t)RegBitrateMsb<<8)+(uint8_t)(BitRateValue>>8));
 RMRFM69WriteRegSpi(((uint16_t)RegBitrateLsb<<8)+(uint8_t)BitRateValue);

 //Devation
 RMRFM69WriteRegSpi(((uint16_t)RegFdevMsb<<8)+(((uint8_t)(DevationValue>>8))&0x3F));
 RMRFM69WriteRegSpi(((uint16_t)RegFdevLsb<<8)+(uint8_t)(DevationValue&0xFF));

 //AFC
 RMRFM69WriteRegSpi(((uint16_t)RegAfcCtrl<<8)+0x00);		//Standard

 //PA
 switch(COB)
 	{
 	case RFM69H:
 	case RFM69HC:
 		RMRFM69WriteRegSpi(((uint16_t)RegPaLevel<<8)+0x60+(OutputPower&0x1F));
 		break;
 	default:
 		RMRFM69WriteRegSpi(((uint16_t)RegPaLevel<<8)+0x80+(OutputPower&0x1F));
 		break;
 	}
 RMRFM69WriteRegSpi(((uint16_t)RegPaRamp<<8)+RMRFM69bSelectRamping(SymbolTime));

 //Ocp
 RMRFM69WriteRegSpi(((uint16_t)RegOcp<<8)+0x0F);			//Disable Ocp
 //LNA
 RMRFM69WriteRegSpi(((uint16_t)RegLna<<8)+0x88);			//High & LNA Enable
 //RxBw
 RMRFM69WriteRegSpi(((uint16_t)RegRxBw<<8)+0x20+BandWidthValue);	//CutOff for 8%
 //AfcBw
 RMRFM69WriteRegSpi(((uint16_t)RegAfcBw<<8)+0x20+BandWidthValue);
 //OOK
 RMRFM69WriteRegSpi(((uint16_t)RegOokPeak<<8)+0x40+(0x07<<3)+0x00);

 //AFC
 RMRFM69WriteRegSpi(((uint16_t)RegAfcFei<<8)+0x00);		//Disable

 RMRFM69WriteRegSpi(((uint16_t)RegPreambleMsb<<8)+(uint8_t)(PreambleLength>>8));
 RMRFM69WriteRegSpi(((uint16_t)RegPreambleLsb<<8)+(uint8_t)PreambleLength);
 if(SyncLength==0)
 	sync = 0;
 else
 	sync = SyncLength-1;
 RMRFM69WriteRegSpi(((uint16_t)RegSyncConfig<<8)+0x80+((sync&0x07)<<3));	//SyncOn Nuint8_t
 for(i=0;i<8;i++)								//SyncWordSetting
 	RMRFM69WriteRegSpi(((uint16_t)(RegSyncValue1+i)<<8)+SyncWord[i]);

 RMRFM69WriteRegSpi(((uint16_t)RegDioMapping2<<8)+0xF7);	//DIO4 PllLock / DIO5 ModeRdy / CLK Off

 i = DcFree_NRZ + AddrFilter_NONE;
 if(!FixedPktLength)
 	i += VariablePacket;
 if(!CrcDisable)
 	{
 	i += CrcOn;
	if(CrcMode)
		i += CrcCalc_IBM;
	else
		i += CrcCalc_CCITT;
	}
 RMRFM69WriteRegSpi(((uint16_t)RegPacketConfig1<<8)+i);
 if(AesOn)
 	RMRFM69WriteRegSpi(((uint16_t)RegPacketConfig2<<8)+0x01);//
 else
 	RMRFM69WriteRegSpi(((uint16_t)RegPacketConfig2<<8)+0);	//

 if(FixedPktLength)								//Set Packet length
 	RMRFM69WriteRegSpi(((uint16_t)RegPayloadLength<<8)+PayloadLength);
 else
 	RMRFM69WriteRegSpi(((uint16_t)RegPayloadLength<<8)+0x40);

 RMRFM69WriteRegSpi(((uint16_t)RegFifoThresh<<8)+0x01);
}

/**********************************************************
**Name:     vGoRx
**Function: set rf69 to receive mode
**Input:    none
**Output:   none
**********************************************************/
void RMRFM69vGoRx(void)
{
 uint8_t tmp;

 RMRFM69WriteRegSpi(((uint16_t)RegTestPa1<<8)+0x55);			//for NormalMode or RxMode
 RMRFM69WriteRegSpi(((uint16_t)RegTestPa2<<8)+0x70);

 if(CrcDisable)
 	RMRFM69WriteRegSpi(((uint16_t)RegDioMapping1<<8)+0x44);	//DIO0 PayloadReady / DIO1 FiflLevel / DIO2 Data /DIO3 FifoFull
 else
 	RMRFM69WriteRegSpi(((uint16_t)RegDioMapping1<<8)+0x04);	//DIO0 CrcOk  / DIO1 FiflLevel / DIO2 Data /DIO3 FifoFull

 tmp = RMRFM69ReadRegSpi(RegOpMode);
 tmp&= MODE_MASK;
 tmp |= RADIO_RX;
 RMRFM69WriteRegSpi(((uint16_t)RegOpMode<<8)+tmp);
}

/**********************************************************
**Name:     vGoStandby
**Function: set rf69 to standby mode
**Input:    none
**Output:   none
**********************************************************/
void RMRFM69vGoStandby(void)
{
 uint8_t tmp;
 tmp = RMRFM69ReadRegSpi(RegOpMode);
 tmp&= MODE_MASK;
 tmp |= RADIO_STANDBY;
 RMRFM69WriteRegSpi(((uint16_t)RegOpMode<<8)+tmp);
}

/**********************************************************
**Name:     vGoSleep
**Function: set rf69 to sleep mode
**Input:    none
**Output:   none
**********************************************************/
void RMRFM69vGoSleep(void)
{
 uint8_t tmp;
 tmp = RMRFM69ReadRegSpi(RegOpMode);
 tmp&= MODE_MASK;
 tmp |= RADIO_SLEEP;
 RMRFM69WriteRegSpi(((uint16_t)RegOpMode<<8)+tmp);
}

/**********************************************************
**Name:     bSendMessage
**Function: set rf69 to sleep mode
**Input:    msg------for which message to send
            length---message length
**Output:   true-----send ok
            false----send error/over time
**********************************************************/
bool RMRFM69bSendMessage(uint8_t msg[], uint8_t length)
{
 uint8_t tmp;
 uint32_t overtime;
 uint16_t bittime;

 switch(COB)
	{
	case RFM65:									//only for Rx
	case RFM65C:
		return(false);
	case RFM69H:
	case RFM69HC:
 		RMRFM69WriteRegSpi(((uint16_t)RegTestPa1<<8)+0x5D);		//for HighPower
	 	RMRFM69WriteRegSpi(((uint16_t)RegTestPa2<<8)+0x7C);
		break;
	default:
	case RFM69:
	case RFM69C:
	 	RMRFM69WriteRegSpi(((uint16_t)RegTestPa1<<8)+0x55);		//for NormalMode or RxMode
 		RMRFM69WriteRegSpi(((uint16_t)RegTestPa2<<8)+0x70);
		break;
	}

 RMRFM69WriteRegSpi(((uint16_t)RegDioMapping1<<8)+0x04);	//DIO0 PacketSend  / DIO1 FiflLevel / DIO2 Data /DIO3 FifoFull

 if(!FixedPktLength)
 	RMRFM69WriteRegSpi(((uint16_t)RegFifo<<8)+length);
 RMRFM69SpiBurstWrite(RegFifo, msg, length);

 tmp = RMRFM69ReadRegSpi(RegOpMode);
 tmp&= MODE_MASK;
 tmp |= RADIO_TX;
 RMRFM69WriteRegSpi(((uint16_t)RegOpMode<<8)+tmp);



 //等待发射完毕
 bittime  = SymbolTime/1000;		//unit: us
 overtime = SyncLength+PreambleLength+length;
 if(!FixedPktLength)				//SyncWord & PktLength & 2uint8_tCRC
    overtime += 1;
 if(!CrcDisable)
 	overtime += 2;
 overtime<<=3;					//8bit == 1uint8_t
 overtime*= bittime;
 overtime/= 1000;				//unit: ms
 if(overtime==0)
 	overtime = 1;
 overtime += (overtime>>3);		//add 12.5% for ensure
 HAL_Delay(overtime);			//
 for(tmp=0;tmp<100;tmp++)		//about 50ms for overtime
 	{
 	if(RMRFM69DIO0())
 		break;
 	RMRFM69Delay_us(500);
 	}
 RMRFM69vGoStandby();
 if(tmp>=100)
 	return(false);
 else
 	return(true);
}

/**********************************************************
**Name:     bGetMessage
**Function: check receive packet
**Input:    msg------for which message to read
**Output:   packet length
			0--------have no packet
**********************************************************/
uint8_t RMRFM69bGetMessage(uint8_t msg[])
{
 uint8_t length;
 if(RMRFM69DIO0())				//收到CrcOk or PayloadReady
 	{
 	if(FixedPktLength)
 		length = PayloadLength;
 	else
 		length = RMRFM69ReadRegSpi(RegFifo);
 	RMRFM69SpiBurstRead(RegFifo, msg, length);
 	RMRFM69vGoStandby();
 	RMRFM69vGoRx();
 	return(length);
 	}
 return(0);
}

/**********************************************************
**Name:     vRF69Reset
**Function: hardware reset rf69 chipset
**Input:    none
**Output:   none
**********************************************************/
void RMRFM69vRF69Reset(void)
{
 //POROut();
 RMRFM69SetPor();
 RMRFM69Delay_us(300);					//at least 100us for reset
 RMRFM69ClrPor();
 //PORIn();							//set POR for free
 HAL_Delay(10);						//wait for ready
}

/**********************************************************
**Name:     bSelectBandwidth
**Function:
**Input:    BandWidth
**Output:   BandWidthValue
**********************************************************/
uint8_t RMRFM69bSelectBandwidth(uint8_t rx_bw)
{
 if(rx_bw<3)
 	return 0x0F;						//2.6KHz
 else if(rx_bw<4)
 	return 0x07;						//3.91KHz
 else if(rx_bw<6)
 	return 0x16;						//5.21KHz
 else if(rx_bw<7)
 	return 0x0E;						//6.25KHz
 else if(rx_bw<8)
 	return 0x06;						//7.81KHz
 else if(rx_bw<11)
	return 0x15;						//10.4KHz 	Min
 else if(rx_bw<13)
 	return 0x0D;						//12.5KHz
 else if(rx_bw<16)
 	return 0x05;						//15.6KHz
 else if(rx_bw<21)
 	return 0x14;						//20.8KHz
 else if(rx_bw<=25)
 	return 0x0C;						//25.0KHz
 else if(rx_bw<32)
 	return 0x04;						//31.3KHz
 else if(rx_bw<42)
 	return 0x13;						//41.7KHz
 else if(rx_bw<=50)
 	return 0x0B;						//50.0KHz
 else if(rx_bw<63)
 	return 0x03;						//62.5KHz
 else if(rx_bw<84)
 	return 0x12;						//83.3KHz
 else if(rx_bw<=100)
 	return 0x0A;						//100KHz
 else if(rx_bw<=125)
 	return 0x02;						//125KHz
 else if(rx_bw<167)
 	return 0x11;						//167KHz
 else if(rx_bw<=200)
 	return 0x09;						//200KHz
 else if(rx_bw<=250)
 	return 0x01;						//250KHz
 else if(rx_bw<=333)
 	return 0x10;						//333KHz
 else if(rx_bw<=400)
 	return 0x08;						//400KHz
 else
 	return 0x00;						//500KHz Max


 /*  //for 30MHz
 if(rx_bw<3)
 	return	0x0F;						//2.93KHz
 else if(rx_bw<4)
 	return	0x07;						//3.66KHz
 else if(rx_bw<5)
 	return	0x16;						//4.88KHz
 else if(rx_bw<6)
 	return 0x0E;						//5.86KHz
 else if(rx_bw<8)
 	return 0x06;						//7.32KHz
 else if(rx_bw<10)
	return 0x15;						//9.77KHz 	Min
 else if(rx_bw<12)
 	return 0x0D;						//11.72KHz
 else if(rx_bw<15)
 	return 0x05;						//14.56KHz
 else if(rx_bw<20)
 	return 0x14;						//19.53KHz
 else if(rx_bw<24)
 	return 0x0C;						//23.44KHz
 else if(rx_bw<30)
 	return 0x04;						//29.3KHz
 else if(rx_bw<40)
 	return 0x13;						//39.06KHz
 else if(rx_bw<47)
 	return 0x0B;						//46.88KHz
 else if(rx_bw<59)
 	return 0x03;						//58.59KHz
 else if(rx_bw<79)
 	return 0x12;						//78.13KHz
 else if(rx_bw<=94)
 	return 0x0A;						//93.75KHz
 else if(rx_bw<=118)
 	return 0x02;						//117.19KHz
 else if(rx_bw<157)
 	return 0x11;						//156.25KHz
 else if(rx_bw<188)
 	return 0x09;						//187.5KHz
 else if(rx_bw<235)
 	return 0x01;						//234.38KHz
 else if(rx_bw<313)
 	return 0x10;						//312.5KHz
 else if(rx_bw<=375)
 	return 0x08;						//375KHz
 else
 	return 0x00;						//500KHz Max
 */
}

/**********************************************************
**Name:     bSelectRamping
**Function:
**Input:    symbol time
**Output:   ramping value
**********************************************************/
uint8_t RMRFM69bSelectRamping(uint32_t symbol)
{
 uint32_t SymbolRate;

 SymbolRate = symbol/1000;			//ns->us
 SymbolRate = SymbolRate/4;			// 1/4 ramping

 if(SymbolRate<=10)
 	return 0x0F;					//10us
 else if(SymbolRate<=12)
 	return 0x0E;					//12us
 else if(SymbolRate<=15)
 	return 0x0D;					//15us
 else if(SymbolRate<=20)
 	return 0x0C;					//20us
 else if(SymbolRate<=25)
 	return 0x0B;					//25us
 else if(SymbolRate<=31)
 	return 0x0A;					//31us
 else if(SymbolRate<=40)
 	return 0x09;					//40us
 else if(SymbolRate<=50)
 	return 0x08;					//50us
 else if(SymbolRate<=62)
 	return 0x07;					//62us
 else if(SymbolRate<=100)
 	return 0x06;					//100us
 else if(SymbolRate<=125)
 	return 0x05;					//125us
 else if(SymbolRate<=250)
 	return 0x04;					//250us
 else if(SymbolRate<=500)
 	return 0x03;					//500us
 else if(SymbolRate<=1000)
	return 0x02;					//1000us
 else if(SymbolRate<=2000)
 	return 0x01;					//2000us
 else
 	return 0x00;
}

/**********************************************************
**Name:     vRF69SetAesKey
**Function: Set AES128 Key block
**Input:    AesKey
**Output:   none
**********************************************************/
void RMRFM69vRF69SetAesKey(void)
{
 uint8_t i;
 for(i=0;i<16;i++)
 	RMRFM69WriteRegSpi(((uint16_t)(RegAesKey1+i)<<8)+AesKey[i]);
}

/**********************************************************
**Name:     vTrigAfc
**Function: Manual Trigger AFC
**Input:    AesKey
**Output:   none
**********************************************************/
void RMRFM69vTrigAfc(void)
{
 uint8_t tmp, i;
 tmp = RMRFM69ReadRegSpi(RegAfcFei);

 if(tmp&0x10)			//AFC has finished
 	{
 	RMRFM69WriteRegSpi(((uint16_t)RegAfcFei<<8)+tmp+0x01);	//TrigAfc

 	for(i=0; i<100; i++)		//
 		{
 		RMRFM69Delay_us(100);
 		tmp = RMRFM69ReadRegSpi(RegAfcFei);
 		if(tmp&0x10)	//AFC has finished
 			break;
 		}
 	}
}


/**********************************************************
**Name:     vDirectRx
**Function: Set RF69 to continuous rx mode(with init.)
**Input:    none
**Output:   none
**********************************************************/
void RMRFM69vDirectRx(void)
{
 RMRFM69ClrPor();
 //PORIn();
 //DIO0In();
 //Spi.vSpiInit();

 //端口初始化 for 32MHz
 FrequencyValue.Freq = (Frequency<<11)/125;		//Calc. Freq
 BitRateValue  = (SymbolTime<<5)/1000;			//Calc. BitRate
 DevationValue = (Devation<<11)/125;			//Calc. Fdev
 BandWidthValue = RMRFM69bSelectBandwidth(BandWidth);

 RMRFM69vRF69Reset();
 RMRFM69vGoStandby();

 //Frequency
 RMRFM69WriteRegSpi(((uint16_t)RegFrMsb<<8)+FrequencyValue.freq.FreqH);
 RMRFM69WriteRegSpi(((uint16_t)RegFrMid<<8)+FrequencyValue.freq.FreqM);
 RMRFM69WriteRegSpi(((uint16_t)RegFrLsb<<8)+FrequencyValue.freq.FreqL);

 // Mode
 switch(Modulation)
 	{
 	case OOK:
 		RMRFM69WriteRegSpi(((uint16_t)RegDataModul<<8)+ConWithClkMode+OokMode+Shaping);
 		break;
 	case GFSK:
 		RMRFM69WriteRegSpi(((uint16_t)RegDataModul<<8)+ConWithClkMode+FskMode+Shaping);
 		break;
 	case FSK:
 	default:
 		RMRFM69WriteRegSpi(((uint16_t)RegDataModul<<8)+ConWithClkMode+FskMode);
 		break;
 	}

 //BitRate
 RMRFM69WriteRegSpi(((uint16_t)RegBitrateMsb<<8)+(uint8_t)(BitRateValue>>8));
 RMRFM69WriteRegSpi(((uint16_t)RegBitrateLsb<<8)+(uint8_t)BitRateValue);

 //Devation
 RMRFM69WriteRegSpi(((uint16_t)RegFdevMsb<<8)+(((uint8_t)(DevationValue>>8))&0x3F));
 RMRFM69WriteRegSpi(((uint16_t)RegFdevLsb<<8)+(uint8_t)(DevationValue&0xFF));

 //AFC
 RMRFM69WriteRegSpi(((uint16_t)RegAfcCtrl<<8)+0x00);		//Standard

 //PA
 switch(COB)
 	{
 	case RFM69H:
 	case RFM69HC:
 		RMRFM69WriteRegSpi(((uint16_t)RegPaLevel<<8)+0x60+(OutputPower&0x1F));
 		break;
 	default:
 		RMRFM69WriteRegSpi(((uint16_t)RegPaLevel<<8)+0x80+(OutputPower&0x1F));
 		break;
 	}
 RMRFM69WriteRegSpi(((uint16_t)RegPaRamp<<8)+RMRFM69bSelectRamping(SymbolTime));

 //Ocp
 RMRFM69WriteRegSpi(((uint16_t)RegOcp<<8)+0x0F);			//Disable Ocp
 //LNA
 RMRFM69WriteRegSpi(((uint16_t)RegLna<<8)+0x88);			//High & LNA Enable
 //RxBw
 RMRFM69WriteRegSpi(((uint16_t)RegRxBw<<8)+0x20+BandWidthValue);	//CutOff for 8%
 //AfcBw
 RMRFM69WriteRegSpi(((uint16_t)RegAfcBw<<8)+0x20+BandWidthValue);
 //OOK
 RMRFM69WriteRegSpi(((uint16_t)RegOokPeak<<8)+0x40+(0x07<<3)+0x00);

 //AFC
 RMRFM69WriteRegSpi(((uint16_t)RegAfcFei<<8)+0x00);		//Disable

 RMRFM69WriteRegSpi(((uint16_t)RegPreambleMsb<<8)+(uint8_t)(PreambleLength>>8));
 RMRFM69WriteRegSpi(((uint16_t)RegPreambleLsb<<8)+(uint8_t)PreambleLength);

 RMRFM69WriteRegSpi(((uint16_t)RegSyncConfig<<8)+0x80+0x00);	//SyncOff Nuint8_t
 RMRFM69WriteRegSpi(((uint16_t)RegDioMapping2<<8)+0xF7);		//DIO4 PllLock / DIO5 ModeRdy / CLK Off

 RMRFM69WriteRegSpi(((uint16_t)RegPacketConfig1<<8)+0x00);
 RMRFM69WriteRegSpi(((uint16_t)RegPacketConfig2<<8)+0);

 RMRFM69vGoStandby();
}

/**********************************************************
**Name:     vChangeFreq
**Function: Change Frequency
**Input:    none
**Output:   none
**********************************************************/
void RMRFM69vChangeFreq(uint32_t freq)
{
 FreqStruct ChangeFreq;
 uint8_t tmp;

 RMRFM69vGoStandby();
 ChangeFreq.Freq = (freq<<11)/125;		//Calc. Freq

 //Frequency
 RMRFM69WriteRegSpi(((uint16_t)RegFrMsb<<8)+ChangeFreq.freq.FreqH);
 RMRFM69WriteRegSpi(((uint16_t)RegFrMid<<8)+ChangeFreq.freq.FreqM);
 RMRFM69WriteRegSpi(((uint16_t)RegFrLsb<<8)+ChangeFreq.freq.FreqL);

 RMRFM69vGoRx();
 do{
 	tmp = RMRFM69ReadRegSpi(RegIrqFlags1);
 	}while((tmp&0x40)!=0x40);

}

/**********************************************************
**Name:     vReadRssi
**Function:
**Input:    none
**Output:   none
**********************************************************/
uint8_t RMRFM69bReadRssi(void)
{
 uint8_t tmp;
 RMRFM69WriteRegSpi(((uint16_t)RegRssiConfig<<8)+0x01);		//TrigRssiADC

 do{
 	tmp = RMRFM69ReadRegSpi(RegRssiConfig);
 	RMRFM69Delay_us(10);
 	}while((tmp&0x02)!=0x02);

 return(RMRFM69ReadRegSpi(RegRssiValue));
}


