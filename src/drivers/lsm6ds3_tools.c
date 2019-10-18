/*
 * lsm6ds3_tools.c
 *
 *  Created on: Aug 27, 2019
 *      Author: michael
 *  Contains platform-based functions for ST driver
 */

#include <stdint.h>
#include <math.h>
//#include <stm32f4xx.h>
#include <stm32f10x.h>
#include <diag/Trace.h>

#include "lsm6ds3_reg.h"
#include "../state.h"
#include "vector.h"


#define LSM_TIMEOUT	1000
#define MG_TO_MPS2	9.80665 / 1000
#define MDPS_TO_RAD	M_PI / 180 / 1000

#define IMU_CALIBRATION 0

//	Accelerometer bias & transform matrix
// FIRST
//#define X_ACCEL_OFFSET		0.073985
//#define Y_ACCEL_OFFSET		0.064143
//#define Z_ACCEL_OFFSET		0.094132
//#define XX_ACCEL_TRANSFORM_MATIX	 1.005659
//#define YY_ACCEL_TRANSFORM_MATIX	 1.003159
//#define ZZ_ACCEL_TRANSFORM_MATIX	 1.007635
//#define XY_ACCEL_TRANSFORM_MATIX	 0.000026
//#define XZ_ACCEL_TRANSFORM_MATIX	-0.002485
//#define YZ_ACCEL_TRANSFORM_MATIX	 0.000322

// SECOND
#define X_ACCEL_OFFSET		0.0
#define Y_ACCEL_OFFSET		0.0
#define Z_ACCEL_OFFSET		0.0
#define XX_ACCEL_TRANSFORM_MATIX	 1.0
#define YY_ACCEL_TRANSFORM_MATIX	 1.0
#define ZZ_ACCEL_TRANSFORM_MATIX	 1.0
#define XY_ACCEL_TRANSFORM_MATIX	 0.0
#define XZ_ACCEL_TRANSFORM_MATIX	 0.0
#define YZ_ACCEL_TRANSFORM_MATIX	 0.0


//typedef union{
//  int16_t i16bit[3];
//  uint8_t u8bit[6];
//} axis3bit16_t;


static uint8_t whoamI, rst;

//SPI_HandleTypeDef	spi_lsm6ds3;
I2C_InitTypeDef	i2c_lsm6ds3;
GPIO_InitTypeDef  GPIO_InitStructure;
lsm6ds3_ctx_t lsm6ds3_dev_ctx;

#define LSM6DS3_I2C_ADD	0b11010111

void i2c_start_transmission()
{
	I2C_GenerateSTART(I2C2, ENABLE);
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C2, LSM6DS3_I2C_ADD, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
}


void i2c_stop_transmission()
{
	I2C_GenerateSTOP(I2C2, ENABLE);

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	int error = 0;

	if (handle == &i2c_lsm6ds3)
	{
		i2c_start_transmission();

		for(int i = 0; i < len; i++)
		{
			I2C_SendData(I2C2, bufp[i]);
			while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
		}

		i2c_stop_transmission();
//		error = HAL_I2C_Mem_Write(handle, LSM6DS3_I2C_ADD, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, LSM_TIMEOUT); TODO:
	}


	return error;
}


static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	int error = 0;

	if (handle == &i2c_lsm6ds3)
	{
		i2c_start_transmission();

		for(int i = 0; i < len; i++)
		{
			bufp[i] = I2C_ReceiveData(I2C2);
			while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
		}

		i2c_stop_transmission();
//		error = HAL_I2C_Mem_Read(handle, LSM6DS3_I2C_ADD, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, LSM_TIMEOUT); TODO:
	}


	return error;
}


int32_t lsm6ds3_bus_init(void* handle)
{
	int error = 0;
	if (handle == &i2c_lsm6ds3)
	{
		//	I2C init
//		i2c_lsm6ds3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//		i2c_lsm6ds3.Init.ClockSpeed = 400000;
//		i2c_lsm6ds3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//		i2c_lsm6ds3.Init.DutyCycle = I2C_DUTYCYCLE_2;
//		i2c_lsm6ds3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//		i2c_lsm6ds3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//		i2c_lsm6ds3.Init.OwnAddress1 = 0x00;
//		i2c_lsm6ds3.Instance = I2C2;
//		i2c_lsm6ds3.Mode = HAL_I2C_MODE_MASTER;

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

		/* Configure I2C_EE pins: SCL and SDA */
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		/* I2C configuration */
		i2c_lsm6ds3.I2C_Mode = I2C_Mode_I2C;
		i2c_lsm6ds3.I2C_DutyCycle = I2C_DutyCycle_2;
		i2c_lsm6ds3.I2C_OwnAddress1 = 0x00;
		i2c_lsm6ds3.I2C_Ack = I2C_Ack_Enable;
		i2c_lsm6ds3.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
		i2c_lsm6ds3.I2C_ClockSpeed = 100000;

		/* I2C Peripheral Enable */
		I2C_Cmd(I2C2, ENABLE);
		/* Apply I2C configuration after enabling it */
		I2C_Init(I2C2, &i2c_lsm6ds3);

//		error |= HAL_I2C_Init(&i2c_lsm6ds3);
//		HAL_Delay(200);
//		trace_printf("i2c_lsm6ds: %d\n", error);
	}
	else
		trace_printf("invalid lsm6ds3 handle\n");

	return error;
}


int32_t lsm6ds3_platform_init()
{
	int error = 0;

	lsm6ds3_dev_ctx.write_reg = platform_write;
	lsm6ds3_dev_ctx.read_reg = platform_read;
	lsm6ds3_dev_ctx.handle = &i2c_lsm6ds3;

	//	Set needed bus parameters
	error |= lsm6ds3_bus_init(lsm6ds3_dev_ctx.handle);

	// Reset to defaults
	error |= lsm6ds3_reset_set(&lsm6ds3_dev_ctx, PROPERTY_ENABLE);
	do {
		error = lsm6ds3_reset_get(&lsm6ds3_dev_ctx, &rst);
	} while (rst);

	// Check who_am_i
	error |= lsm6ds3_device_id_get(&lsm6ds3_dev_ctx, &whoamI);
	if (whoamI != LSM6DS3_ID)
	{
		trace_printf("lsm6ds3 not found, %d\terror: %d\n", whoamI, error);
		return -19;
	}
	else
		trace_printf("lsm6ds3 OK\n");

	error |= lsm6ds3_fifo_mode_set(&lsm6ds3_dev_ctx, PROPERTY_DISABLE);

	error |= lsm6ds3_block_data_update_set(&lsm6ds3_dev_ctx, PROPERTY_DISABLE);

	error |= lsm6ds3_xl_full_scale_set(&lsm6ds3_dev_ctx, LSM6DS3_4g);
	error |= lsm6ds3_gy_full_scale_set(&lsm6ds3_dev_ctx, LSM6DS3_1000dps);

	error |= lsm6ds3_xl_data_rate_set(&lsm6ds3_dev_ctx, LSM6DS3_XL_ODR_104Hz);
	error |= lsm6ds3_gy_data_rate_set(&lsm6ds3_dev_ctx, LSM6DS3_GY_ODR_104Hz);

	error |= lsm6ds3_xl_filter_analog_set(&lsm6ds3_dev_ctx, LSM6DS3_ANTI_ALIASING_200Hz);

	return error;
}


uint32_t lsm6ds3_get_xl_data_g(float* accel)
{
	axis3bit16_t data_raw_acceleration;
	uint8_t error;
	//	Read acceleration field data
	PROCESS_ERROR(lsm6ds3_acceleration_raw_get(&lsm6ds3_dev_ctx, data_raw_acceleration.u8bit));
	accel[0] = lsm6ds3_from_fs4g_to_mg(data_raw_acceleration.i16bit[0]) * MG_TO_MPS2;
	accel[1] = lsm6ds3_from_fs4g_to_mg(data_raw_acceleration.i16bit[1]) * MG_TO_MPS2;
	accel[2] = lsm6ds3_from_fs4g_to_mg(data_raw_acceleration.i16bit[2]) * MG_TO_MPS2;

//	if (!IMU_CALIBRATION)
//	{
//		//	Accelerometer bias and transform matrix (to provide real values)
//		float offset_vector[3] = {X_ACCEL_OFFSET, Y_ACCEL_OFFSET, Z_ACCEL_OFFSET};
//		float transform_matrix[3][3] =	{{XX_ACCEL_TRANSFORM_MATIX, XY_ACCEL_TRANSFORM_MATIX, XZ_ACCEL_TRANSFORM_MATIX},
//										 {XY_ACCEL_TRANSFORM_MATIX, YY_ACCEL_TRANSFORM_MATIX, YZ_ACCEL_TRANSFORM_MATIX},
//										 {XZ_ACCEL_TRANSFORM_MATIX, YZ_ACCEL_TRANSFORM_MATIX, ZZ_ACCEL_TRANSFORM_MATIX}};
//
//		vmv(accel, offset_vector, accel);
//		mxv(transform_matrix, accel, accel);
//	}

end:
	return error;
}


uint32_t lsm6ds3_get_g_data_rps(float* gyro)
{
	axis3bit16_t data_raw_angular_rate;
	uint8_t error;
	//	Read acceleration field data
	error = lsm6ds3_angular_rate_raw_get(&lsm6ds3_dev_ctx, data_raw_angular_rate.u8bit);
	gyro[0] = lsm6ds3_from_fs1000dps_to_mdps(data_raw_angular_rate.i16bit[0]) * MDPS_TO_RAD;
	gyro[1] = lsm6ds3_from_fs1000dps_to_mdps(data_raw_angular_rate.i16bit[1]) * MDPS_TO_RAD;
	gyro[2] = lsm6ds3_from_fs1000dps_to_mdps(data_raw_angular_rate.i16bit[2]) * MDPS_TO_RAD;
	return error;
}
