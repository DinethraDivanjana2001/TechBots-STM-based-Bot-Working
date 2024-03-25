/*
 * TPS55288.h
 *
 *  Created on: Mar 14, 2024
 *      Author: niros
 */

#ifndef TPS55288_H
#define TPS55288_H

#include "stm32f4xx_hal.h"

#define TPS55288_I2C_ADDR (0x74 << 1)

#define TPS55288_VREF_LSB 0x00
#define TPS55288_VREF_MSB 0x01
#define TPS55288_IOUT_LIMIT 0x02
#define TPS55288_VOUT_SR 0x03
#define TPS55288_VOUT_FS 0x04
#define TPS55288_CDC 0x05
#define TPS55288_MODE 0x06
#define TPS55288_STATUS 0x07

typedef struct {
	I2C_HandleTypeDef *i2cHandle;


}TPS55288;

uint8_t TPS55288_Initialize(TPS55288 *dev, I2C_HandleTypeDef *i2cHandle);
HAL_StatusTypeDef TPS55288_WriteRegister(TPS55288 *dev , uint8_t reg, uint8_t *data );


#endif /* INC_TPS55288_H_ */
