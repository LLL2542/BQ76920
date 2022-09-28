
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

#ifndef INC_BQ76920_H_
#define INC_BQ76920_H_

#ifdef	__cplusplus
extern "C" {
#endif

#ifndef STM32F7XX_H
#include "stm32f7xx_hal.h"
#endif

#ifndef STM32F7XX_HAL_I2C_H
#include "stm32f7xx_hal_i2c.h"
#endif

#define BQ76920_ADDRESS			(0x18<<1) // change address respectively to the IC 7 bit adressing
#define NUMBER_OF_CELLS 		4	 	// Number of cell
#define RSENSE					2	 	// mOhm
#define	balanceThreshold		0.05f	// V

/* Register mapping */
#define SYS_STAT 				0x00	// Clear bit in this address by writing a '1' to the corresponding bit
#define CELLBAL1 				0x01	// for BQ76920
#define CELLBAL2 				0x02	// for BQ76930
#define CELLBAL3 				0x03	// for BQ76940
#define SYS_CTRL1 				0x04	// ADC control and shut down
#define SYS_CTRL2 				0x05	// Coulomb counting control and FET control
#define PROTECT1				0x06	// protection setting
#define PROTECT2				0x07	// protection setting
#define PROTECT3				0x08	// protection setting
#define OV_TRIP					0X09	// Over voltage setting
#define UV_TRIP					0x0A	// Under voltage setting
#define CC_CFG					0x0B	// set to 0x19 during startup
#define VC1_HI					0x0C	// voltage cell 1 MSB bit[5-0]
#define VC1_LO					0x0D	// voltage cell 1 LSB bit[7-0] 14bit adc
#define VC2_HI					0x0E
#define VC2_LO					0x0F
#define VC3_HI					0x10
#define VC3_LO					0x11
#define VC4_HI					0x12
#define VC4_LO					0x13
#define VC5_HI					0x14
#define VC5_LO					0x15
#define BAT_HI					0x2A	// Bat MSB bit[7-0]
#define BAT_LO					0x2B	// Bat LSB bit[7-0] 16 bit adc
#define TS1_HI					0x2C	// DIETEMP ADC 6 bit MSB [5-0]
#define TS1_LO					0x2D	// DIETEMP ADC 8 bit LSB [7-0]
#define CC_HI					0x32	// coulomb counter upper 8 MSB
#define CC_LO					0x33	// coulomb counter lower 8 MSB
#define ADCGAIN1				0x50
#define ADCOFFSET				0x51
#define ADCGAIN2				0x59


enum SCD_D{	// Short circuit in discharge delay
	SDC_70us_delay			=	0x00, // 70  us
	SDC_100us_delay			=	0x01, // 100 us
	SDC_200us_delay			=	0x02, // 200 us
	SDC_400us_delay			=   0x03  // 400 us
};

enum SCD_T{ // Overcurrent in discharge delay
	SCD_Threshold_44mV		=	0x00,	// (RSNS = '1')
	SCD_Threshold_67mV		=	0x01,	// Voltage drop in Sense resistor
	SCD_Threshold_89mV		=	0x02,
	SCD_Threshold_111mV		=	0x03,
	SCD_Threshold_133mV		=	0x04,
	SCD_Threshold_155mV		=	0x05,
	SCD_Threshold_178mV		=	0x06,
	SCD_Threshold_200mV		=	0x07,
};

enum OCD_D{ // Overcurrent in discharge delay
	ODC_8ms_delay			=	0x00, // 8		ms
	ODC_20ms_delay			=	0x01, // 20		ms
	ODC_40ms_delay			=	0x02, // 40		ms
	ODC_80ms_delay			=	0x03, // 80 	ms
	ODC_160ms_delay			=	0x04, // 160 	ms
	ODC_320ms_delay			=	0x05, // 320 	ms
	ODC_640ms_delay			=	0x06, // 640 	ms
	ODC_1280ms_delay		=	0x07 // 1280	ms
};

enum OCD_T{ // Overcurrent in discharge delay
	OCD_Threshold_17mV		=	0x00, 	// (RSNS = '1')
	OCD_Threshold_22mV		=	0x01, 	// Voltage drop in Sense resistor
	OCD_Threshold_28mV		=	0x02,
	OCD_Threshold_33mV		=	0x03,
	OCD_Threshold_39mV		=	0x04,
	OCD_Threshold_44mV		=	0x05,
	OCD_Threshold_50mV		=	0x06,
	OCD_Threshold_56mV		=	0x07,
	OCD_Threshold_61mV		=	0x08,
	OCD_Threshold_67mV		=	0x09,
	OCD_Threshold_72mV		=	0x0A,
	OCD_Threshold_78mV		=	0x0B,
	OCD_Threshold_83mV		=	0x0C,
	OCD_Threshold_89mV		=	0x0D,
	OCD_Threshold_94mV		=	0x0E,
	OCD_Threshold_100mV		=	0x0F
};

enum UV_D{ 		// undervoltage cell delay
	UV_Delay_1s				= 0x00,
	UV_Delay_4s				= 0x01,
	UV_Delay_8s				= 0x02,
	UV_Delay_16s			= 0x03
};

enum OV_D{ 		// undervoltage cell delay
	OV_Delay_1s				= 0x00,
	OV_Delay_2s				= 0x01,
	OV_Delay_4s				= 0x02,
	OV_Delay_8s				= 0x03
};

enum cell{
	VC1		= VC1_HI,
	VC2		= VC2_HI,
	VC3		= VC3_HI,
	VC4		= VC5_HI // skip VC4 because batt pack is 4S
};

typedef struct	{
	/* SPI Handle for pass by reference*/
	I2C_HandleTypeDef 	*i2cHandle;
	GPIO_TypeDef 		*bootPort;
	uint16_t			bootPin;
	/* Add other here*/
	int8_t 				OFFSET;
	uint16_t 			GAIN;
	float				Vcell[4];
}BQ76920_t;

// HW Initialize
void BQ7690_Initialise(BQ76920_t *BMS, I2C_HandleTypeDef*i2cHandle);
void BQ76920_Enable_ADC(BQ76920_t *BMS);		// done
// Hardware Setting
void BQ76920_Shutdown(BQ76920_t *BMS);			// done
// Balance function
void EnableBalanceCell(BQ76920_t *BMS);			// done
void DisableBalanceCell(BQ76920_t *BMS);		// done
// Telemetry and Telecommand
float getCellVoltage(BQ76920_t *BMS,int cell);  // done
float getPackVoltage(BQ76920_t *BMS);			// done
float getCurrent(BQ76920_t *BMS);				// done
float getDieTemp(BQ76920_t *BMS);				// done

// Low-level function
void BQ76920_ReadRegister	(BQ76920_t 	*BMS,uint8_t reg, uint8_t *data);
void BQ76920_ReadRegisters	(BQ76920_t 	*BMS,uint8_t reg, uint8_t *data,uint8_t length);
void BQ76920_WriteRegister	(BQ76920_t 	*BMS,uint8_t reg, uint8_t *data);

#endif /* INC_BQ76920_H_ */

