/*
 *  MPU6500.c
 *
 *  Created on: Jan 12, 2024
 *      Author: BerkN
 *
 *  MPU6500 Object Oriented Module Driver
 *  Calibration and raw data transformation.
 *  SPI or I2C based communication. (SPI tested)
 *  Adaptive to changed settings.
 *
 *	Updates and bug reports :  @ https://github.com/Berkin99/MPU6500
 *
 *  12.01.2024 : Created MPU6500 module driver.
 *
 *  References:
 *  [0] PS-MPU-6500A-01-v1.3.pdf (Datasheet)
 *  [1] MPU-6500-Register-Map2.pdf
 *
 */


#include "MPU6500.h"

#define REG_XG_OFFS_TC           0x00
#define REG_YG_OFFS_TC           0x01
#define REG_ZG_OFFS_TC           0x02
#define REG_X_FINE_GAIN          0x03
#define REG_Y_FINE_GAIN          0x04
#define REG_Z_FINE_GAIN          0x05
#define REG_XA_OFFS_H            0x06
#define REG_XA_OFFS_L            0x07
#define REG_YA_OFFS_H            0x08
#define REG_YA_OFFS_L            0x09
#define REG_ZA_OFFS_H            0x0A
#define REG_ZA_OFFS_L            0x0B
#define REG_PRODUCT_ID           0x0C
#define REG_SELF_TEST_X          0x0D
#define REG_SELF_TEST_Y          0x0E
#define REG_SELF_TEST_Z          0x0F
#define REG_SELF_TEST_A          0x10
#define REG_XG_OFFS_USRH         0x13
#define REG_XG_OFFS_USRL         0x14
#define REG_YG_OFFS_USRH         0x15
#define REG_YG_OFFS_USRL         0x16
#define REG_ZG_OFFS_USRH         0x17
#define REG_ZG_OFFS_USRL         0x18
#define REG_SMPLRT_DIV           0x19
#define REG_CONFIG               0x1A
#define REG_GYRO_CONFIG          0x1B
#define REG_ACCEL_CONFIG         0x1C
#define REG_INT_PIN_CFG          0x37
#define REG_INT_ENABLE           0x38
#define REG_ACCEL_XOUT_H         0x3B
#define REG_ACCEL_XOUT_L         0x3C
#define REG_ACCEL_YOUT_H         0x3D
#define REG_ACCEL_YOUT_L         0x3E
#define REG_ACCEL_ZOUT_H         0x3F
#define REG_ACCEL_ZOUT_L         0x40
#define REG_TEMP_OUT_H           0x41
#define REG_TEMP_OUT_L           0x42
#define REG_GYRO_XOUT_H          0x43
#define REG_GYRO_XOUT_L          0x44
#define REG_GYRO_YOUT_H          0x45
#define REG_GYRO_YOUT_L          0x46
#define REG_GYRO_ZOUT_H          0x47
#define REG_GYRO_ZOUT_L          0x48
#define REG_USER_CTRL            0x6A
#define REG_PWR_MGMT_1           0x6B
#define REG_PWR_MGMT_2           0x6C
#define REG_BANK_SEL             0x6D
#define REG_MEM_START_ADDR       0x6E
#define REG_MEM_R_W              0x6F
#define REG_DMP_CFG_1            0x70
#define REG_DMP_CFG_2            0x71
#define REG_FIFO_COUNTH          0x72
#define REG_FIFO_COUNTL          0x73
#define REG_FIFO_R_W             0x74
#define REG_WHOAMI               0x75

#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10

#define READ_FLAG                   0x80

static uint8_t _buffer [14];

uint8_t MPU6500_Init(struct MPU6500_Device *dev){

	MPU6500_SetSleepMode(dev, 0);
	MPU6500_RequestSettings(dev);

	for(uint8_t i = 0; i<3;i++){
		dev->calib_data.acc_cal[i] = 0;
		dev->calib_data.gyr_cal[i] = 0;
	}

	return 1;
}

uint8_t MPU6500_Test(struct MPU6500_Device *dev){
	if(MPU6500_getDeviceID(dev) == 0x70) return 1;
	return 0;
}

void MPU6500_ApplySettings(struct MPU6500_Device *dev, struct MPU6500_Settings settings){
	MPU6500_SetAccelRange(dev,settings.acc_range);
	MPU6500_SetGyroRange(dev,settings.gyr_range);
	MPU6500_SetDLPFBandwidth(dev,settings.dlpf);
	MPU6500_SetSampleRateDivider(dev,settings.srd);
	MPU6500_SetClockSource(dev,settings.clock_source);
}

void MPU6500_RequestSettings(struct MPU6500_Device *dev){
	MPU6500_GetAccelRange(dev);
	MPU6500_GetGyroRange(dev);
	MPU6500_GetDLPFBandwidth(dev);
	MPU6500_GetSampleRateDivider(dev);
	MPU6500_GetClockSource(dev);
}

void MPU6500_AccCalibration(struct MPU6500_Device *dev, uint32_t time_ms){
	/* Accelerometer calibration for parallel surface*/
	double acc_cal[3] = {0,0,0};
	int16_t acc[3];
	int16_t gyr[3];

	for(uint8_t i = 0; i<3; i++){dev->calib_data.acc_cal[i] = 0;}

	uint16_t cal_iteration = time_ms*2;
	for(uint16_t i = 0; i<cal_iteration; i++){
		MPU6500_GetDataRaw(dev, acc, gyr);
		acc_cal[0]+= acc[0];
		acc_cal[1]+= acc[1];
		acc_cal[2]+= acc[2];
		dev->delay_us(500);
	}
	for(uint8_t i = 0; i<3; i++){dev->calib_data.acc_cal[i] = (int16_t)(acc_cal[i]/cal_iteration);}

	dev->calib_data.acc_cal[2] = 0; /* Do not calibrate Z axis */
}


void MPU6500_GyrCalibration(struct MPU6500_Device *dev,uint32_t time_ms){
	double gyr_cal[3] = {0,0,0};
	int16_t acc[3];
	int16_t gyr[3];

	for(uint8_t i = 0; i<3; i++){dev->calib_data.gyr_cal[i] = 0;}

	uint16_t cal_iteration = time_ms*2;
	for(uint16_t i = 0; i<cal_iteration; i++){
		MPU6500_GetDataRaw(dev, acc, gyr);
		gyr_cal[0]+= gyr[0];
		gyr_cal[1]+= gyr[1];
		gyr_cal[2]+= gyr[2];
		dev->delay_us(500);
	}
	for(uint8_t i = 0; i<3; i++){dev->calib_data.gyr_cal[i] = (int16_t)(gyr_cal[i]/cal_iteration);}
}

void MPU6500_readRegister(struct MPU6500_Device *dev, uint8_t target, uint8_t* pRxBuffer, uint8_t len){
	target |= READ_FLAG;
	dev->read(target,pRxBuffer,len);
}


void MPU6500_writeRegister(struct MPU6500_Device *dev, uint8_t target, uint8_t* pData, uint8_t len){
	dev->write(target,pData,len);
}

uint8_t MPU6500_getDeviceID(struct MPU6500_Device *dev){
	uint8_t buf = 0;
	MPU6500_readRegister(dev,REG_WHOAMI, &buf, 1);
	return buf;
}

void MPU6500_SetSleepMode(struct MPU6500_Device *dev, uint8_t isSleep){
	uint8_t reg = 0;
	MPU6500_readRegister(dev,REG_PWR_MGMT_1, &reg, 1);
	reg &= 0b10111111; 						// Clear the setting area
	reg |= ((isSleep & 0b00000001)<<6); 	// Add setting to config register value
	MPU6500_writeRegister(dev,REG_CONFIG, &reg, 1);
}

void MPU6500_GetDataRaw(struct MPU6500_Device *dev, int16_t* acc, int16_t* gyr){
	MPU6500_readRegister(dev,REG_ACCEL_XOUT_H,_buffer,14);

	acc[0] = ((((int16_t)_buffer[0])  << 8) | _buffer[1]);
	acc[1] = ((((int16_t)_buffer[2])  << 8) | _buffer[3]);
	acc[2] = ((((int16_t)_buffer[4])  << 8) | _buffer[5]);
	gyr[0] = ((((int16_t)_buffer[8])  << 8) | _buffer[9]);
	gyr[1] = ((((int16_t)_buffer[10]) << 8) | _buffer[11]);
	gyr[2] = ((((int16_t)_buffer[12]) << 8) | _buffer[13]);
}

void MPU6500_GetData(struct MPU6500_Device *dev, float* acc, float* gyr){

	int16_t _acc[3];
	int16_t _gyr[3];

	void MPU6500_GetDataRaw(dev, _acc, _gyr);

	for (uint8_t i = 0 ;i<3; i++){
		acc[i] = (float) (_acc[i] - dev->calib_data.acc_cal[i]) * dev->calib_data.acc_coefficient;
		gyr[i] = (float) (_gyr[i] - dev->calib_data.gyr_cal[i]) * dev->calib_data.gyr_coefficient;
	}
}



void MPU6500_SetAccelRange(struct MPU6500_Device *dev,MPU6500_AccelRange_e range){
	uint8_t reg = 0;
	MPU6500_readRegister(dev, REG_ACCEL_CONFIG, &reg, 1);
	reg &= 0b11100111; 				// Clear the setting area
	reg |= ((uint8_t)range << 3); 	// Add setting to config register value
	MPU6500_writeRegister(dev, REG_ACCEL_CONFIG, &reg, 1);
	dev->settings.acc_range = range;
	dev->calib_data.acc_coefficient = (float)((1<<range)*4000)/65536.0;

}

void MPU6500_SetGyroRange(struct MPU6500_Device *dev, MPU6500_GyroRange_e range){
	uint8_t reg = 0;
	MPU6500_readRegister(dev,REG_GYRO_CONFIG, &reg, 1);
	reg &= 0b11100111; 				// Clear the setting area
	reg |= ((uint8_t)range << 3); 	// Add setting to config register value
	MPU6500_writeRegister(dev,REG_GYRO_CONFIG, &reg, 1);
	dev->settings.gyr_range = range;
	dev->calib_data.gyr_coefficient = (float)((1<<range)*500)/65536.0;
}

void MPU6500_SetDLPFBandwidth(struct MPU6500_Device *dev, MPU6500_DLPFBandwidth_e dlpf){
	uint8_t reg = 0;
	MPU6500_readRegister(dev,REG_CONFIG, &reg, 1);
	reg &= 0b11111000; 			// Clear the setting area
	reg |= (uint8_t)dlpf; 		// Add setting to config register value
	MPU6500_writeRegister(dev,REG_CONFIG, &reg, 1);
	dev->settings.dlpf = dlpf;
}

void MPU6500_SetSampleRateDivider(struct MPU6500_Device *dev,MPU6500_SampleRateDivider_e srd){
	uint8_t temp = srd;
	MPU6500_writeRegister(dev, REG_SMPLRT_DIV,&temp,1);
	dev->settings.srd = srd;
}

void MPU6500_SetClockSource(struct MPU6500_Device *dev, MPU6500_ClockSource_e clock_source){
	uint8_t reg = 0;
	MPU6500_readRegister(dev,REG_PWR_MGMT_1, &reg, 1);
	reg &= 0b11111000; 					// Clear the setting area
	reg |= (uint8_t)clock_source;	 	// Add setting to config register value
	MPU6500_writeRegister(dev,REG_PWR_MGMT_1, &reg, 1);
	dev->settings.clock_source = clock_source;
}


uint8_t MPU6500_GetAccelRange(struct MPU6500_Device *dev){
	uint8_t reg = 0;
	MPU6500_readRegister(dev, REG_ACCEL_CONFIG, &reg, 1);
	reg &= 0b00011000;	// Clear unwanted area
	reg = reg>>3;		// Normalize
	dev->settings.acc_range = reg;
	return reg;
}

uint8_t MPU6500_GetGyroRange(struct MPU6500_Device *dev){
	uint8_t reg = 0;
	MPU6500_readRegister(dev ,REG_GYRO_CONFIG, &reg, 1);
	reg &= 0b00011000;	// Clear unwanted area
	reg = reg>>3;		// Normalize
	dev->settings.gyr_range = reg;
	return reg;
}

uint8_t MPU6500_GetDLPFBandwidth(struct MPU6500_Device *dev){
	uint8_t reg = 0;
	MPU6500_readRegister(dev, REG_CONFIG, &reg, 1);
	reg &= 0b00000111; // Clear unwanted area
	dev->settings.dlpf = reg;
	return reg;
}
uint8_t MPU6500_GetSampleRateDivider(struct MPU6500_Device *dev){
	uint8_t reg = 0;
	MPU6500_readRegister(dev, REG_SMPLRT_DIV, &reg, 1);
	dev->settings.srd = reg;
	return reg;

}

uint8_t MPU6500_GetClockSource(struct MPU6500_Device *dev){
	uint8_t reg = 0;
	MPU6500_readRegister(dev, REG_PWR_MGMT_1, &reg, 1);
	reg &= 0b00000111; 				// Clear unwanted area
	dev->settings.clock_source = reg;
	return reg;
}
