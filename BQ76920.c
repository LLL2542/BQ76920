
/*******************************************************************************
 * @file        BQ76920.h
 * @brief       C Library for BQ76920 an Analog Front-End Battery Manangement System
 * @version     1.0
 * @author      Nawat Kitiphuwadon
 * @date        2022-9-26
********************************************************************************

MIT License
Copyright (c) 2018 ETA Systems
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "BQ76920.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

void BQ76920_Initialise(BQ76920_t *BMS, I2C_HandleTypeDef*i2cHandle)
{
	// Boot BQ76920 via GPIO
	HAL_GPIO_WritePin(BMS -> bootPort, BMS -> bootPin, GPIO_PIN_SET);
	HAL_Delay(50); 		// wait for 50 ms
	HAL_GPIO_WritePin(BMS -> bootPort, BMS -> bootPin, GPIO_PIN_RESET);
	// Device StartUp
	BMS -> i2cHandle = i2cHandle;
	// Set CC_CFG to 0x19
	uint8_t CC_CFG_REG = 0x19;
	BQ76920_WriteRegister(BMS, CC_CFG_REG, &CC_CFG_REG);
	// Set protect1 register
	uint8_t PROTECT1_REG;
	BQ76920_ReadRegister(BMS,PROTECT1,&PROTECT1_REG);
	PROTECT1_REG = PROTECT1_REG |(SDC_100us_delay << 3);
	PROTECT1_REG = PROTECT1_REG | SCD_Threshold_89mV;
	BQ76920_WriteRegister(BMS, PROTECT1, &PROTECT1_REG);
	// Set protect2 register
	uint8_t PROTECT2_REG;
	BQ76920_ReadRegister(BMS,PROTECT2,&PROTECT2_REG);
	PROTECT2_REG = PROTECT2_REG | (ODC_160ms_delay << 5) ;
	PROTECT2_REG = PROTECT2_REG | (OCD_Threshold_39mV) ;
	BQ76920_WriteRegister(BMS, PROTECT2, &PROTECT2_REG);
	// Set protect3 register
	uint8_t PROTECT3_REG;
	BQ76920_ReadRegister(BMS,PROTECT3,&PROTECT3_REG);
	PROTECT3_REG = PROTECT3_REG | (UV_Delay_4s << 6) ;
	PROTECT3_REG = PROTECT3_REG | (OV_Delay_4s << 4) ;
	BQ76920_WriteRegister(BMS, PROTECT3, &PROTECT3_REG);
	// Set OV_Trip register
	uint8_t OV_TRIP_REG[2] = {0x2B,0xF9};	// 4.3V depend on ADC
	BQ76920_WriteRegister(BMS, OV_TRIP, &OV_TRIP_REG[0]);
	BQ76920_WriteRegister(BMS, OV_TRIP, &OV_TRIP_REG[1]);
	// Set UV_Trip register
	uint8_t UV_TRIP_REG[2] = {0x19,0x91}; 	// 2.5V depend on ADC
	BQ76920_WriteRegister(BMS, UV_TRIP, &UV_TRIP_REG[0]);
	BQ76920_WriteRegister(BMS, UV_TRIP, &UV_TRIP_REG[1]);
}



void BQ76920_Enable_ADC(BQ76920_t *BMS)
{
	uint8_t SYS_CTRL1_REG, SYS_CTRL2_REG;
	BQ76920_ReadRegister(BMS,SYS_CTRL1,&SYS_CTRL1_REG);
	SYS_CTRL1_REG = SYS_CTRL1_REG | (1<<4); // set ADC_EN
	BQ76920_ReadRegister(BMS,SYS_CTRL2,&SYS_CTRL2_REG);
	SYS_CTRL2_REG = SYS_CTRL2_REG | (1<<6); // set CC_EN

	// read adc gain
	uint8_t regData[2] = {0u,0u};	// declare buffer
	BQ76920_ReadRegister(BMS, ADCGAIN1, &regData[0]);
	BQ76920_ReadRegister(BMS, ADCGAIN2, &regData[1]);
	uint8_t ADCGAIN  = (((regData[1] & 0xE0)>>5) | ((regData[0] & 0x0C)<<1));
	uint16_t GAIN = ADCGAIN+365;
	// read adc offset
	BQ76920_ReadRegister(BMS, ADCOFFSET,&regData[0]);
	int8_t OFFSET = (int8_t)regData[0];
	// stored in BMS struct for later use
	BMS->GAIN 	= GAIN;
	BMS->OFFSET = OFFSET;
}


float getPackVoltage(BQ76920_t *BMS)
{
	uint8_t regData[2] = {0u,0u};	// declare buffer
	BQ76920_ReadRegister(BMS, BAT_LO, &regData[0]);	// Read data in BAT_LO register
	BQ76920_ReadRegister(BMS, BAT_HI, &regData[1]); // Read data in BAT_HI register
	// return float value of Vpack
	return	(float)((regData[1]<<8) | regData[0]) * 4 * (BMS->GAIN / 1000000.0f) + (8*(BMS->OFFSET)/1000) ;
}


float getCurrent(BQ76920_t *BMS)
{
	uint8_t regData[2] = {0u,0u};	// declare buffer
	BQ76920_ReadRegister(BMS, CC_LO, &regData[0]);
	BQ76920_ReadRegister(BMS, CC_HI, &regData[1]);
	if (regData[0]!=0 || regData[1]!= 0 ) // if data not available then don't convert
	{
		short I = ((regData[1]<<8) | regData[0]);
		if(I>((1<<15)-1)){
			I = -((~I)+1);
		}
		float Vs = I * 8.44/1000000;  //unit is V.
		float Ipack = Vs*1000/(RSENSE);
		// return float value of pack current
		return Ipack;
	}
	else
		return 0;
}

float getCellVoltage(BQ76920_t *BMS,int cell)
{
	if((cell == VC1)|| (cell == VC2)|| (cell == VC3) ||(cell == VC4))
	{
		uint8_t regData[2] = {0u,0u};	// declare buffer
		BQ76920_ReadRegister(BMS, cell, &regData[0]);
		BQ76920_ReadRegister(BMS, cell+1, &regData[1]);
		uint16_t VoltageCellRaw  = ((regData[1]<<8)) | regData[0];
		float VoltageCell = (BMS -> GAIN)*(VoltageCellRaw)+(BMS->OFFSET);
		// stored data in struct for balance Cell
		switch(cell)
		{
			case VC1:
				BMS->Vcell[0] = VoltageCell;
				break;
			case VC2:
				BMS->Vcell[1] = VoltageCell;
				break;
			case VC3:
				BMS->Vcell[2] = VoltageCell;
				break;
			case VC4:
				BMS->Vcell[3] = VoltageCell;
				break;
			default:
				break;
		}
		// return float value of requested cell
		return VoltageCell;
	}
	else
		return  0;
}


float getDieTemp(BQ76920_t *BMS)
{
	float temp;
	uint8_t TS1_HI_REG,TS1_LO_REG;
	uint16_t TS1_REG;
	BQ76920_ReadRegister(BMS, TS1_HI, &TS1_HI_REG);
	BQ76920_ReadRegister(BMS, TS1_LO, &TS1_LO_REG);
	if ((TS1_HI_REG != 0)||(TS1_LO_REG!=0))
	{
		TS1_REG = ((uint16_t)TS1_HI_REG <<8) | TS1_LO_REG;
		temp = 25.0f - (((TS1_REG*0.000382f)-1.20f)/0.0042f);
		return temp;
	}
	else
		return 0;
}


void EnableBalanceCell(BQ76920_t *BMS)
{
	if((BMS->Vcell[0] != 0)	&& (BMS->Vcell[1] != 0) && (BMS->Vcell[2] != 0) && (BMS->Vcell[3] != 0))
	{
		uint8_t balancingFlagsTarget,balancingFlags;
		float minVoltage = 4.2f;
		for(int i = 0; i<=3; i++) // find min voltage
		{
			if((BMS->Vcell[i]-minVoltage)<0)
			{
				minVoltage = BMS->Vcell[i];
			}
		}

		for(int i =0; i<=3; i++)
		{
			if((BMS->Vcell[i] - minVoltage) >= balanceThreshold)
			{
			      // try to enable balancing of current cell
			      balancingFlagsTarget = balancingFlags | (1 << i);
			      // check if attempting to balance adjacent cells
			      bool adjacentCellCollision =	((balancingFlagsTarget << 1) & balancingFlags) ||((balancingFlags << 1) & balancingFlagsTarget);
			      if (adjacentCellCollision == 0) {
			    	  balancingFlags = balancingFlagsTarget;
			      }
			}
		}
		BQ76920_WriteRegister(BMS,CELLBAL1,&balancingFlags);
	}
	else
		return;
}

void DisableBalanceCell(BQ76920_t *BMS)
{
	uint8_t balancingFlags = 0;	// disable balance
	BQ76920_WriteRegister(BMS,CELLBAL1,&balancingFlags);
}

void BQ76920_Shutdown(BQ76920_t *BMS)
{
	uint8_t SYS_CTRL2_REG;
	BQ76920_ReadRegister(BMS,SYS_CTRL2,&SYS_CTRL2_REG);
	SYS_CTRL2_REG = SYS_CTRL2_REG | 0;	// step 1
	BQ76920_WriteRegister(BMS, SYS_CTRL2, &SYS_CTRL2_REG);	// proceed step1
	SYS_CTRL2_REG = SYS_CTRL2_REG | 1;	// step 2
	BQ76920_WriteRegister(BMS, SYS_CTRL2, &SYS_CTRL2_REG);	// proceed step2
	SYS_CTRL2_REG = SYS_CTRL2_REG | 1<<1;	// step 3
	SYS_CTRL2_REG = SYS_CTRL2_REG & 0xFE;	// step 3
	BQ76920_WriteRegister(BMS, SYS_CTRL2, &SYS_CTRL2_REG);	// proceed step3
}


void BQ76920_ReadRegister(BQ76920_t 	*BMS,uint8_t reg, uint8_t *data)
{
	// data from I2C with be stored in *data
	// reg is register address
	 HAL_I2C_Mem_Read(BMS-> i2cHandle, BQ76920_ADDRESS|0x01, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 1000);
}

void BQ76920_ReadRegisters(BQ76920_t 	*BMS,uint8_t reg, uint8_t *data,uint8_t length)
{
	 HAL_I2C_Mem_Read(BMS-> i2cHandle, BQ76920_ADDRESS|0x01, reg, I2C_MEMADD_SIZE_8BIT, data,length, 1000);
}

void BQ76920_WriteRegister(BQ76920_t 	*BMS,uint8_t reg, uint8_t *data)
{
	 HAL_I2C_Mem_Write(BMS-> i2cHandle, BQ76920_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 1000);
}




