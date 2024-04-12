/*
 *  MPU6500.h
 *
 *  Created on: Jan 12, 2024
 *      Author: BerkN
 *
 *  MPU6500 Object Oriented Module Driver
 *  Calibration and raw data transformation.
 *  SPI or I2C based communication. (SPI tested)
 *  Adaptive to changed settings.
 *
 * Updates and bug reports :  @ https://github.com/Berkin99/Sensors
 *
 *  12.01.2024 : Created MPU6500 module driver.
 *
 *  References:
 *  [0] PS-MPU-6500A-01-v1.3.pdf (Datasheet)
 *  [1] MPU-6500-Register-Map2.pdf
 *
 */

#ifndef MPU6500_H_
#define MPU6500_H_

#include <stdint.h>

typedef enum {
	ACCEL_RANGE_2G = 0,
	ACCEL_RANGE_4G,
	ACCEL_RANGE_8G,
	ACCEL_RANGE_16G
}MPU6500_AccelRange_e;

typedef enum {
	GYRO_RANGE_250DPS = 0,
	GYRO_RANGE_500DPS,
	GYRO_RANGE_1000DPS,
	GYRO_RANGE_2000DPS
}MPU6500_GyroRange_e;

typedef enum {
	DLPF_BANDWIDTH_250HZ = 0,
	DLPF_BANDWIDTH_184HZ,
	DLPF_BANDWIDTH_92HZ,
	DLPF_BANDWIDTH_41HZ,
	DLPF_BANDWIDTH_20HZ,
	DLPF_BANDWIDTH_10HZ,
	DLPF_BANDWIDTH_5HZ
}MPU6500_DLPFBandwidth_e;

typedef enum {
    SMPL_1000HZ = 0,
    SMPL_500HZ,
    SMPL_333HZ,
    SMPL_250HZ,
    SMPL_200HZ,
    SMPL_167HZ,
    SMPL_143HZ,
    SMPL_125HZ,
}MPU6500_SampleRateDivider_e;

typedef enum{
	CLOCK_INTERNAL = 0,
	CLOCK_PLL_XGYRO,
	CLOCK_PLL_YGYRO,
	CLOCK_PLL_ZGYRO,
	CLOCK_PLL_EXT32K,
	CLOCK_PLL_EXT19M,
	CLOCK_KEEP_RESET,
}MPU6500_ClockSource_e;

typedef enum {
	MPU6500_SPI_INTF,
	MPU6500_I2C_INTF
}MPU6500_Intf_e;

typedef int8_t (*MPU6500_Read_f)(uint8_t reg_addr, uint8_t *read_data, uint8_t len);
typedef int8_t (*MPU6500_Write_f)(uint8_t reg_addr, uint8_t *write_data, uint8_t len);
typedef void   (*MPU6500_DelayUs_f)(uint32_t period);

struct MPU6500_Settings{
	MPU6500_AccelRange_e acc_range;
	MPU6500_GyroRange_e gyr_range;
	MPU6500_DLPFBandwidth_e dlpf;
	MPU6500_SampleRateDivider_e srd;
	MPU6500_ClockSource_e clock_source;
};

struct MPU6500_CalibData{
	int16_t acc_cal[3];
	int16_t gyr_cal[3];
	float acc_coefficient; /* Coefficient for transform raw data to Gs */
	float gyr_coefficient; /* Coefficient for transform raw data to deg/s */
};

struct MPU6500_Device
{
    uint8_t chip_id;
    /* Interface Selection
     * For SPI, interface = BMP3_SPI_INTF
     * For I2C, interface = BMP3_I2C_INTF
     **/
    MPU6500_Intf_e intf;
    MPU6500_Read_f read; 			/* Read function pointer */
    MPU6500_Write_f write;    		/* Write function pointer */
    MPU6500_DelayUs_f delay_us;     /* Delay function pointer */

    /* Trim data */
    struct MPU6500_CalibData calib_data;
    struct MPU6500_Settings settings;
};

uint8_t MPU6500_Init(struct MPU6500_Device *dev);
uint8_t MPU6500_Test(struct MPU6500_Device *dev);
void MPU6500_ApplySettings(struct MPU6500_Device *dev, struct MPU6500_Settings settings);
void MPU6500_RequestSettings(struct MPU6500_Device *dev);

void MPU6500_AccCalibration(struct MPU6500_Device *dev,uint32_t time_ms);
void MPU6500_GyrCalibration(struct MPU6500_Device *dev,uint32_t time_ms);

void MPU6500_readRegister(struct MPU6500_Device *dev, uint8_t target, uint8_t* pRxBuffer, uint8_t len);
void MPU6500_writeRegister(struct MPU6500_Device *dev, uint8_t target, uint8_t* pData, uint8_t len);

uint8_t MPU6500_getDeviceID(struct MPU6500_Device *dev);
void MPU6500_GetDataRaw(struct MPU6500_Device *dev, int16_t* acc, int16_t* gyr);
void MPU6500_GetData(struct MPU6500_Device *dev, float* acc, float* gyr); /* Data transformed to SI Units : Gs and deg/s */
void MPU6500_SetSleepMode(struct MPU6500_Device *dev, uint8_t isSleep);

void MPU6500_SetAccelRange(struct MPU6500_Device *dev, MPU6500_AccelRange_e range);
void MPU6500_SetGyroRange(struct MPU6500_Device *dev, MPU6500_GyroRange_e range);
void MPU6500_SetDLPFBandwidth(struct MPU6500_Device *dev, MPU6500_DLPFBandwidth_e dlpf);
void MPU6500_SetSampleRateDivider(struct MPU6500_Device *dev, MPU6500_SampleRateDivider_e srd);
void MPU6500_SetClockSource(struct MPU6500_Device *dev, MPU6500_ClockSource_e clock_source);

void MPU6500_SetAccelDLPFBandwidth(struct MPU6500_Device *dev);

uint8_t MPU6500_GetAccelRange(struct MPU6500_Device *dev);
uint8_t MPU6500_GetGyroRange(struct MPU6500_Device *dev);
uint8_t MPU6500_GetDLPFBandwidth(struct MPU6500_Device *dev);
uint8_t MPU6500_GetSampleRateDivider(struct MPU6500_Device *dev);
uint8_t MPU6500_GetClockSource(struct MPU6500_Device *dev);


#endif /* MPU6500_H_ */
