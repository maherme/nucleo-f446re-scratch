/*****************************************************************************************************
* FILENAME :        i2c_driver.c
*
* DESCRIPTION :
*       File containing the APIs for configuring the I2C peripheral.
*
* PUBLIC FUNCTIONS :
*       void    I2C_Init(I2C_Handle_t* pI2C_Handle)
*       void    I2C_DeInit(I2C_RegDef_t* pI2Cx)
*       void    I2C_PerClkCtrl(I2C_RegDef_t* pI2Cx, uint8_t en_or_di)
*       void    I2C_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di)
*       void    I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
*       void    I2C_Enable(I2C_RegDef_t* pI2Cx, uint8_t en_or_di)
*       uint8_t I2C_GetFlagStatus(SPI_RegDef_t* pI2Cx, uint32_t flagname)
*       void    I2C_ApplicationEventCallback(I2C_Handle_t* pI2C_Handle, uint8_t app_event)
*
* NOTES :
*       For further information about functions refer to the corresponding header file.
*
**/

#include <stdint.h>
#include "i2c_driver.h"

void I2C_Init(I2C_Handle_t* pI2C_Handle){
}

void I2C_DeInit(I2C_RegDef_t* pI2Cx){
}

void I2C_PerClkCtrl(I2C_RegDef_t* pI2Cx, uint8_t en_or_di){
}

void I2C_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di){
}

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
}

void I2C_Enable(I2C_RegDef_t* pI2Cx, uint8_t en_or_di){
}

uint8_t I2C_GetFlagStatus(SPI_RegDef_t* pI2Cx, uint32_t flagname){
    return 0;
}

void I2C_ApplicationEventCallback(I2C_Handle_t* pI2C_Handle, uint8_t app_event){
}
