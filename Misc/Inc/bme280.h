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
	uint8_t id;
	uint8_t reset[2];
	uint8_t ctrlHum;
	uint8_t status;
	uint8_t ctrlMeas;
	uint8_t config;
	uint8_t press;
	uint8_t temp;
	uint8_t hum;
	uint8_t digT1T3;
	uint8_t digP1P9;
	uint8_t digH1;
	uint8_t digH2H6;
} Bme280Reg;

typedef struct
{
	uint8_t  id;
	uint8_t  address;
	uint8_t  pressMsb;
	uint8_t  pressLsb;
	uint8_t  pressXlsb;
	uint8_t  tempMsb;
	uint8_t  tempLsb;
	uint8_t  tempXlsb;
	uint8_t  humMsb;
	uint8_t  humLsb;
	uint16_t digT1;
	int16_t  digT2;
	int16_t  digT3;
	uint16_t digP1;
	int16_t  digP2;
	int16_t  digP3;
	int16_t  digP4;
	int16_t  digP5;
	int16_t  digP6;
	int16_t  digP7;
	int16_t  digP8;
	int16_t  digP9;
	uint8_t  digH1;
	int16_t  digH2;
	uint8_t  digH3;
	int16_t  digH4;
	int16_t  digH5;
	int8_t   digH6;
	int32_t  temperature;
	uint32_t pressure;
	uint32_t humidity;
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

CommStatus bme280Init(I2C_HandleTypeDef *hi2c, Bme280DeviceData *bme280Device);
uint8_t bme280CheckCommunication(I2C_HandleTypeDef *hi2c, Bme280DeviceData *bme280Device);
void bme280GetCoefficients(I2C_HandleTypeDef *hi2c, Bme280DeviceData *bme280Device);
uint8_t bme280CheckId(I2C_HandleTypeDef *hi2c, Bme280DeviceData *bme280Device);
void bme280Reset(I2C_HandleTypeDef *hi2c, Bme280DeviceData *bme280Device);
void bme280Control(I2C_HandleTypeDef *hi2c, Bme280DeviceData *bme280Device, Oversampling hum, Oversampling press, Oversampling temp, Mode mode);
MeasuringStatus bme280GetStatus(I2C_HandleTypeDef *hi2c, Bme280DeviceData *bme280Device);
void bme280Config(I2C_HandleTypeDef *hi2c, Bme280DeviceData *bme280Device, TimeStandby timeStandby, FilterCoef filterCoef);
int32_t bme280ReadTemperature(I2C_HandleTypeDef *hi2c, Bme280DeviceData *bme280Device);
uint32_t bme280ReadPressure(I2C_HandleTypeDef *hi2c, Bme280DeviceData *bme280Device);
uint32_t bme280ReadHumidity(I2C_HandleTypeDef *hi2c, Bme280DeviceData *bme280Device);

#endif /* INC_BME280_H_ */
