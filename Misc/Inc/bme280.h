/*
 * bme280.h
 *
 *  Created on: Jul 12, 2022
 *      Author: Otavio
 */

#ifndef INC_BME280_H_
#define INC_BME280_H_

#include "main.h"

typedef struct
{
	uint8_t ui8Id;
	uint8_t ui8Reset[2];
	uint8_t ui8CtrlHum;
	uint8_t ui8Status;
	uint8_t ui8CtrlMeas;
	uint8_t ui8Config;
	uint8_t ui8Press;
	uint8_t ui8Temp;
	uint8_t ui8Hum;
	uint8_t ui8DigT1T3;
	uint8_t ui8DigP1P9;
	uint8_t ui8DigH1;
	uint8_t ui8DigH2H6;
} Bme280Reg;

typedef struct
{
	uint8_t  ui8Id;
	uint8_t  ui8Address;
	uint8_t  ui8PressMsb;
	uint8_t  ui8PressLsb;
	uint8_t  ui8PressXlsb;
	uint8_t  ui8TempMsb;
	uint8_t  ui8TempLsb;
	uint8_t  ui8TempXlsb;
	uint8_t  ui8HumMsb;
	uint8_t  ui8HumLsb;
	uint16_t ui16DigT1;
	int16_t  i16DigT2;
	int16_t  i16DigT3;
	uint16_t ui16DigP1;
	int16_t  i16DigP2;
	int16_t  i16DigP3;
	int16_t  i16DigP4;
	int16_t  i16DigP5;
	int16_t  i16DigP6;
	int16_t  i16DigP7;
	int16_t  i16DigP8;
	int16_t  i16DigP9;
	uint8_t  ui8DigH1;
	int16_t  i16DigH2;
	uint8_t  ui8DigH3;
	int16_t  i16DigH4;
	int16_t  i16DigH5;
	int8_t   i8DigH6;
	int32_t  i32Temperature;
	uint32_t ui32Pressure;
	uint32_t ui32Humidity;
} Bme280DeviceData;

typedef enum OVERSAMPLING
{
	OVER_SKIPPED	= 0x00,
	OVER_EN_X_1		= 0x01,
	OVER_EN_X_2		= 0x02,
	OVER_EN_X_4		= 0x03,
	OVER_EN_X_8		= 0x04,
	OVER_EN_X_16	= 0x05
} Oversampling;

typedef enum MODE
{
	SLEEP	= 0x00,
	FORCED	= 0x01,
	NORMAL	= 0x03
} Mode;

typedef enum MEASURING_STATUS
{
	AVAILABLE	= 0x00,
	UPDATING 	= 0x01,
	MEASURING	= 0x08
} MeasuringStatus;

typedef enum COMM_STATUS
{
	NOK			= 0x00,
	OK			= 0x01
} CommStatus;

typedef enum TIME_STANDBY
{
	T_0_5_MS	= 0x00,
	T_62_5_MS	= 0x01,
	T_125_MS	= 0x02,
	T_250_MS	= 0x03,
	T_500_MS	= 0x04,
	T_10000_MS	= 0x05,
	T_10_MS		= 0x06,
	T_20_MS		= 0x07
} TimeStandby;

typedef enum FILTER_COEF
{
	FILTER_COEF_OFF	= 0x00,
	FILTER_COEF_2	= 0x01,
	FILTER_COEF_4	= 0x02,
	FILTER_COEF_8	= 0x03,
	FILTER_COEF_16	= 0x04
} FilterCoef;

CommStatus bme280Init(I2C_HandleTypeDef *hi2c, Bme280DeviceData *sBme280Device);
uint8_t bme280CheckCommunication(I2C_HandleTypeDef *hi2c, Bme280DeviceData *sBme280Device);
void bme280GetCoefficients(I2C_HandleTypeDef *hi2c, Bme280DeviceData *sBme280Device);
uint8_t bme280CheckId(I2C_HandleTypeDef *hi2c, Bme280DeviceData *sBme280Device);
void bme280Reset(I2C_HandleTypeDef *hi2c, Bme280DeviceData *sBme280Device);
void bme280Control(I2C_HandleTypeDef *hi2c, Bme280DeviceData *sBme280Device, Oversampling hum, Oversampling press, Oversampling temp, Mode mode);
MeasuringStatus bme280GetStatus(I2C_HandleTypeDef *hi2c, Bme280DeviceData *sBme280Device);
void bme280Config(I2C_HandleTypeDef *hi2c, Bme280DeviceData *sBme280Device, TimeStandby timeStandby, FilterCoef filterCoef);
int32_t bme280ReadTemperature(I2C_HandleTypeDef *hi2c, Bme280DeviceData *sBme280Device);
uint32_t bme280ReadPressure(I2C_HandleTypeDef *hi2c, Bme280DeviceData *sBme280Device);
uint32_t bme280ReadHumidity(I2C_HandleTypeDef *hi2c, Bme280DeviceData *sBme280Device);

#endif /* INC_BME280_H_ */
