
#include "TPS55288.h"

uint8_t TPS55288_Initialize(TPS55288 *dev, I2C_HandleTypeDef *i2cHandle){
	dev->i2cHandle = i2cHandle;
	HAL_StatusTypeDef status;
	uint8_t modeRegData = 0xA0;
	uint8_t VOUT_FS_DATA = 0x83;
	status = TPS55288_WriteRegister(dev,TPS55288_VOUT_FS,&VOUT_FS_DATA);
	status = TPS55288_WriteRegister(dev,TPS55288_MODE,&modeRegData);
	return status;
}


HAL_StatusTypeDef TPS55288_WriteRegister(TPS55288 *dev , uint8_t reg, uint8_t *data ){

	return HAL_I2C_Mem_Write( dev->i2cHandle, TPS55288_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);

}
