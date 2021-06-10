/*****************************************************************************************************
* FILENAME :        i2c_driver.h
*
* DESCRIPTION :
*       Header file containing the prototypes of the APIs for configuring the I2C peripheral.
*
* PUBLIC FUNCTIONS :
*
**/

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include <stdint.h>
#include "stm32f446xx.h"

/**
 * Configuration structure for I2C peripheral.
 */
typedef struct
{
    uint32_t I2C_SCLSpeed;
    uint8_t I2C_DeviceAddress;
    uint8_t I2C_ACKControl;
    uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

/**
 * Handle structure for I2Cx peripheral.
 */
typedef struct
{
    I2C_RegDef_t* pI2Cx;        /* Base address of the I2Cx peripheral */
    I2C_Config_t I2C_Config;    /* I2Cx peripheral configuration settings */
}I2C_Handle_t;

#endif /* I2C_DRIVER_H */
