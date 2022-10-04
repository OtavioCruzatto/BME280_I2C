/*
 * bme280.c
 *
 *  Created on: Jul 12, 2022
 *      Author: Otavio
 */

#include "bme280.h"

static Bme280Reg bme280Reg;
static int32_t fineTemperature = 0;
static uint32_t timeoutI2C = 100;

CommStatus bme280Init(I2C_HandleTypeDef *hi2c, Bme280DeviceData *bme280Device)
{
	CommStatus communicationStatus = NOK;

	bme280Reg.id 			= 0xD0;
	bme280Reg.reset[0]		= 0xE0;
	bme280Reg.reset[1]		= 0xB6;
	bme280Reg.ctrlHum		= 0xF2;
	bme280Reg.status		= 0xF3;
	bme280Reg.ctrlMeas		= 0xF4;
	bme280Reg.config		= 0xF5;
	bme280Reg.press			= 0xF7;
	bme280Reg.temp			= 0xFA;
	bme280Reg.hum			= 0xFD;
	bme280Reg.digT1T3		= 0x88;
	bme280Reg.digP1P9		= 0x8E;
	bme280Reg.digH1			= 0xA1;
	bme280Reg.digH2H6		= 0xE1;

	bme280Device->id		= 0x60;
	bme280Device->address	= 0x76 << 1;
	bme280Device->pressMsb	= 0x00;
	bme280Device->pressLsb	= 0x00;
	bme280Device->pressXlsb	= 0x00;
	bme280Device->tempMsb	= 0x00;
	bme280Device->tempLsb	= 0x00;
	bme280Device->tempXlsb	= 0x00;
	bme280Device->humMsb	= 0x00;
	bme280Device->humLsb	= 0x00;
	bme280Device->digT1		= 0x0000;
	bme280Device->digT2		= 0x0000;
	bme280Device->digT3		= 0x0000;
	bme280Device->digP1		= 0x0000;
	bme280Device->digP2		= 0x0000;
	bme280Device->digP3		= 0x0000;
	bme280Device->digP4		= 0x0000;
	bme280Device->digP5		= 0x0000;
	bme280Device->digP6		= 0x0000;
	bme280Device->digP7		= 0x0000;
	bme280Device->digP8		= 0x0000;
	bme280Device->digP9		= 0x0000;
	bme280Device->digH1		= 0x00;
	bme280Device->digH2		= 0x0000;
	bme280Device->digH3		= 0x00;
	bme280Device->digH4		= 0x0000;
	bme280Device->digH5		= 0x0000;
	bme280Device->digH6		= 0x00;
	bme280Device->temperature	= 0x00000000;
	bme280Device->pressure	= 0x00000000;
	bme280Device->humidity    = 0x00000000;

	if (bme280CheckCommunication(hi2c, bme280Device) == NOK)
	{
		return NOK;
	}

	if (bme280CheckId(hi2c, bme280Device) == bme280Device->id)
	{
		bme280GetCoefficients(hi2c, bme280Device);
		bme280Config(hi2c, bme280Device, T_10_MS, FILTER_COEF_OFF);
		bme280Control(hi2c, bme280Device, OVER_EN_X_1, OVER_EN_X_1, OVER_EN_X_1, NORMAL);
		communicationStatus = OK;
	}

	return communicationStatus;
}

CommStatus bme280CheckCommunication(I2C_HandleTypeDef *hi2c, Bme280DeviceData *bme280Device)
{
	CommStatus communicationStatus = NOK;
	uint32_t ui32CommunicationTrials = 5;
	if (HAL_I2C_IsDeviceReady(hi2c, bme280Device->address, ui32CommunicationTrials, timeoutI2C) == HAL_OK)
	{
		communicationStatus = OK;
	}
	return communicationStatus;
}

void bme280GetCoefficients(I2C_HandleTypeDef *hi2c, Bme280DeviceData *bme280Device)
{
	uint8_t coefBytes[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	uint8_t temperatureCoefQtyBytes = 6;
	uint8_t pressureCoefQtyBytes = 18;
	uint8_t humidityCoefQtyBytes = 8 - 1;

	// Get temperature coefficients
	HAL_I2C_Master_Transmit(hi2c, bme280Device->address, &bme280Reg.digT1T3, sizeof(bme280Reg.digT1T3), timeoutI2C);
	HAL_I2C_Master_Receive(hi2c, bme280Device->address, coefBytes, temperatureCoefQtyBytes, timeoutI2C);
	bme280Device->digT1 = (coefBytes[1] << 8) | coefBytes[0];
	bme280Device->digT2  = (coefBytes[3] << 8) | coefBytes[2];
	bme280Device->digT3  = (coefBytes[5] << 8) | coefBytes[4];

	// Get pressure coefficients
	HAL_I2C_Master_Transmit(hi2c, bme280Device->address, &bme280Reg.digP1P9, sizeof(bme280Reg.digP1P9), timeoutI2C);
	HAL_I2C_Master_Receive(hi2c, bme280Device->address, coefBytes, pressureCoefQtyBytes, timeoutI2C);
	bme280Device->digP1 = (coefBytes[1] << 8) | coefBytes[0];
	bme280Device->digP2  = (coefBytes[3] << 8) | coefBytes[2];
	bme280Device->digP3  = (coefBytes[5] << 8) | coefBytes[4];
	bme280Device->digP4  = (coefBytes[7] << 8) | coefBytes[6];
	bme280Device->digP5  = (coefBytes[9] << 8) | coefBytes[8];
	bme280Device->digP6  = (coefBytes[11] << 8) | coefBytes[10];
	bme280Device->digP7  = (coefBytes[13] << 8) | coefBytes[12];
	bme280Device->digP8  = (coefBytes[15] << 8) | coefBytes[14];
	bme280Device->digP9  = (coefBytes[17] << 8) | coefBytes[16];

	// Get humidity coefficients
	HAL_I2C_Master_Transmit(hi2c, bme280Device->address, &bme280Reg.digH1, sizeof(bme280Reg.digH1), timeoutI2C);
	HAL_I2C_Master_Receive(hi2c, bme280Device->address, coefBytes, 1, timeoutI2C);
	bme280Device->digH1 = coefBytes[0];
	HAL_I2C_Master_Transmit(hi2c, bme280Device->address, &bme280Reg.digH2H6, sizeof(bme280Reg.digH2H6), timeoutI2C);
	HAL_I2C_Master_Receive(hi2c, bme280Device->address, coefBytes, humidityCoefQtyBytes, timeoutI2C);
	bme280Device->digH2 = (coefBytes[1] << 8) | coefBytes[0];
	bme280Device->digH3 = coefBytes[2];
	bme280Device->digH4 = (coefBytes[3] << 4) | (coefBytes[4] & 0x0F);
	bme280Device->digH5 = ((coefBytes[4] & 0xF0) >> 4) | (coefBytes[5] << 4);
	bme280Device->digH6  = coefBytes[6];
}

uint8_t bme280CheckId(I2C_HandleTypeDef *hi2c, Bme280DeviceData *bme280Device)
{
	uint8_t id = 0;
	HAL_I2C_Master_Transmit(hi2c, bme280Device->address, &bme280Reg.id, sizeof(bme280Reg.id), timeoutI2C);
	HAL_I2C_Master_Receive(hi2c, bme280Device->address, &id, sizeof(id), timeoutI2C);
	return id;
}

void bme280Reset(I2C_HandleTypeDef *hi2c, Bme280DeviceData *bme280Device)
{
	HAL_I2C_Master_Transmit(hi2c, bme280Device->address, bme280Reg.reset, sizeof(bme280Reg.reset), timeoutI2C);
}

void bme280Control(I2C_HandleTypeDef *hi2c, Bme280DeviceData *bme280Device, Oversampling hum, Oversampling press, Oversampling temp, Mode mode)
{
	uint8_t osrs_h = hum & 0x07;
	uint8_t osrs_t = temp & 0x07;
	uint8_t osrs_p = press & 0x07;
	uint8_t operation_mode = mode & 0x03;

	uint8_t ctrl_hum[2] =
	{
			bme280Reg.ctrlHum,
			osrs_h
	};

	uint8_t ctrl_meas[2] =
	{
			bme280Reg.ctrlMeas,
			(osrs_t << 5) | (osrs_p << 2) | operation_mode
	};

	HAL_I2C_Master_Transmit(hi2c, bme280Device->address, ctrl_hum, sizeof(ctrl_hum), timeoutI2C);
	HAL_I2C_Master_Transmit(hi2c, bme280Device->address, ctrl_meas, sizeof(ctrl_meas), timeoutI2C);
}

MeasuringStatus bme280GetStatus(I2C_HandleTypeDef *hi2c, Bme280DeviceData *bme280Device)
{
	MeasuringStatus status = AVAILABLE;
	HAL_I2C_Master_Transmit(hi2c, bme280Device->address, &bme280Reg.status, sizeof(bme280Reg.status), timeoutI2C);
	HAL_I2C_Master_Receive(hi2c, bme280Device->address, &status, sizeof(status), timeoutI2C);
	return status;
}

void bme280Config(I2C_HandleTypeDef *hi2c, Bme280DeviceData *bme280Device, TimeStandby timeStandby, FilterCoef filterCoef)
{
	uint8_t t_sb = timeStandby & 0x07;
	uint8_t filter = filterCoef & 0x07;
	uint8_t spi3w_en = 0 & 0x01;

	uint8_t config[2] =
	{
			bme280Reg.config,
			(t_sb << 5) | (filter << 2) | spi3w_en
	};

	bme280Control(hi2c, bme280Device, OVER_SKIPPED, OVER_SKIPPED, OVER_SKIPPED, SLEEP);
	HAL_I2C_Master_Transmit(hi2c, bme280Device->address, config, sizeof(config), timeoutI2C);
}

/**
 * Returns temperature in degC.
 * Output value of "5123" equals 51.23 degC
 */
int32_t bme280ReadTemperature(I2C_HandleTypeDef *hi2c, Bme280DeviceData *bme280Device)
{
	int32_t temperature_degC_x_100 = 0;
	int8_t temperatureDataQtyBytes = 3;
	uint8_t dataBytes[3] = {0, 0, 0};

	HAL_I2C_Master_Transmit(hi2c, bme280Device->address, &bme280Reg.temp, sizeof(bme280Reg.temp), timeoutI2C);
	HAL_I2C_Master_Receive(hi2c, bme280Device->address, dataBytes, temperatureDataQtyBytes, timeoutI2C);
	bme280Device->tempMsb = dataBytes[0];
	bme280Device->tempLsb = dataBytes[1];
	bme280Device->tempXlsb = dataBytes[2];

	int32_t adc_T = (bme280Device->tempMsb << 12) | (bme280Device->tempLsb << 4) | (bme280Device->tempXlsb >> 4);
	int32_t var1 = ((((adc_T >> 3) - ((int32_t) bme280Device->digT1 << 1))) * ((int32_t) bme280Device->digT2)) >> 11;
	int32_t var2 = (((((adc_T >> 4) - ((int32_t) bme280Device->digT1)) * ((adc_T >> 4) - ((int32_t) bme280Device->digT1))) >> 12) * ((int32_t) bme280Device->digT3)) >> 14;
	fineTemperature = var1 + var2;
	bme280Device->temperature = (fineTemperature * 5 + 128) >> 8;
	temperature_degC_x_100 = bme280Device->temperature;

	return temperature_degC_x_100;
}

/**
 * Returns pressure in Pa
 */
uint32_t bme280ReadPressure(I2C_HandleTypeDef *hi2c, Bme280DeviceData *bme280Device)
{
	uint32_t pressure_Pa = 0;
	int8_t pressureDataQtyBytes = 3;
	uint8_t dataBytes[3] = {0, 0, 0};

	HAL_I2C_Master_Transmit(hi2c, bme280Device->address, &bme280Reg.press, sizeof(bme280Reg.press), timeoutI2C);
	HAL_I2C_Master_Receive(hi2c, bme280Device->address, dataBytes, pressureDataQtyBytes, timeoutI2C);
	bme280Device->pressMsb = dataBytes[0];
	bme280Device->pressLsb = dataBytes[1];
	bme280Device->pressXlsb = dataBytes[2];

	int32_t adc_P = (bme280Device->pressMsb << 12) | (bme280Device->pressLsb << 4) | (bme280Device->pressXlsb >> 4);
	bme280ReadTemperature(hi2c, bme280Device);
	int64_t var1 = ((int64_t) fineTemperature) - 128000;
	int64_t var2 = var1 * var1 * (int64_t) bme280Device->digP6;
	var2 = var2 + ((var1 * (int64_t) bme280Device->digP5) << 17);
	var2 = var2 + (((int64_t) bme280Device->digP4) << 35);
	var1 = ((var1 * var1 * (int64_t) bme280Device->digP3) >> 8) + ((var1 * (int64_t) bme280Device->digP2) << 12);
	var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) bme280Device->digP1) >> 33;
	if (var1 == 0) { return 0; }
	int64_t p_aux = 1048576 - adc_P;
	p_aux = (((p_aux << 31) - var2) * 3125) / var1;
	var1 = (((int64_t) bme280Device->digP9) * (p_aux >> 13) * (p_aux >> 13)) >> 25;
	var2 = (((int64_t) bme280Device->digP8) * p_aux) >> 19;
	p_aux = ((p_aux + var1 + var2) >> 8) + (((int64_t) bme280Device->digP7) << 4);
	pressure_Pa = p_aux / 256;
	bme280Device->pressure = pressure_Pa;

	return bme280Device->pressure;
}

/**
 * Returns humidity in %RH
 */
uint32_t bme280ReadHumidity(I2C_HandleTypeDef *hi2c, Bme280DeviceData *bme280Device)
{
	uint32_t humidity_RH_x_10 = 0;
	int8_t humidityDataQtyBytes = 2;
	uint8_t dataBytes[2] = {0, 0};

	HAL_I2C_Master_Transmit(hi2c, bme280Device->address, &bme280Reg.hum, sizeof(bme280Reg.hum), timeoutI2C);
	HAL_I2C_Master_Receive(hi2c, bme280Device->address, dataBytes, humidityDataQtyBytes, timeoutI2C);
	bme280Device->humMsb = dataBytes[0];
	bme280Device->humLsb = dataBytes[1];

	int32_t adc_H = (bme280Device->humMsb << 8) | bme280Device->humLsb;
	bme280ReadTemperature(hi2c, bme280Device);
	int32_t v_x1_u32r = (fineTemperature - ((int32_t) 76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t) bme280Device->digH4) << 20) - (((int32_t) bme280Device->digH5) * v_x1_u32r)) +
			((int32_t) 16384)) >> 15) * (((((((v_x1_u32r * ((int32_t) bme280Device->digH6)) >> 10) *
			(((v_x1_u32r * ((int32_t) bme280Device->digH3)) >> 11) + ((int32_t) 32768))) >> 10) + ((int32_t) 2097152)) *
			((int32_t) bme280Device->digH2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t) bme280Device->digH1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	humidity_RH_x_10 = (((uint32_t) (v_x1_u32r >> 12)) / 102);
	bme280Device->humidity = humidity_RH_x_10;

	return bme280Device->humidity;
}



