#include <mpu6050.h>
#include <main.h>
#include <stdio.h>

extern I2C_HandleTypeDef hi2c2;


void mpu6050_init()
{
    HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c2, (DEVICE_ADDRESS << 1) + 0, 1, 100);
    if (ret == HAL_OK)
    {
        printf("The device is ready \n");
    }
    else
    {
        printf("The device is not ready. Check cables \n ");
    }

    uint8_t temp_data = FS_GYRO_500;
    ret = HAL_I2C_Mem_Write(&hi2c2, (DEVICE_ADDRESS << 1) + 0, REG_CONFIG_GYRO, 1, &temp_data, 1, 100);
    if (ret == HAL_OK)
    {
        printf("Configuring gyro \n");
    }
    else
    {
        printf("Failed to configure gyro \n");
    }

    temp_data = FS_ACC_4G;
	ret = HAL_I2C_Mem_Write(&hi2c2, (DEVICE_ADDRESS << 1) + 0, REG_CONFIG_ACC, 1, &temp_data, 1, 100);
	if (ret == HAL_OK)
	{
		printf("Configuring Acc \n");
	}
	else
	{
		printf("Failed to configure acc \n");
	}

	temp_data = 0;
	ret = HAL_I2C_Mem_Write(&hi2c2, (DEVICE_ADDRESS << 1) + 0, REG_USR_CTRL, 1, &temp_data, 1, 100);
	if (ret == HAL_OK)
	{
		printf("Exiting from sleep mode \n");
	}
	else
	{
		printf("Failed to exit from sleep mode \n");
	}
}

int mpu6050_read()
{
	uint8_t data[2];
	int16_t z_gyro;
	HAL_I2C_Mem_Read(&hi2c2, (DEVICE_ADDRESS <<1) + 1, GYRO_Z,
							1, data, 2, 100);
	z_gyro = ((int16_t)data[0] << 8) + data[1];
	printf("z gyro : %d \n", z_gyro);
	return z_gyro;
}
