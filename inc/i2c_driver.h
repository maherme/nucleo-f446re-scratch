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
 * @I2C_SCLSPEED
 * I2C possible SCL speed values.
 */
#define I2C_SCL_SPEED_SM        100000
#define I2C_SCL_SPEED_FM2K      200000
#define I2C_SCL_SPEED_FM4K      400000

/**
 * @I2C_ACKCONTROL
 * I2C possible ack control values.
 */
#define I2C_ACK_ENABLE          1
#define I2C_ACL_DISABLE         0

/**
 * @I2C_FMDUTYCYCLE
 * I2C possible duty cycle values.
 */
#define I2C_FM_DUTY_2           0
#define I2C_FM_DUTY_16_9        1



/**
 * Configuration structure for I2C peripheral.
 */
typedef struct
{
    uint32_t I2C_SCLSpeed;          /* Possible values from @I2C_SCLSPEED */
    uint8_t I2C_DeviceAddress;
    uint8_t I2C_ACKControl;         /* Possible values from @I2C_ACKCONTROL */
    uint16_t I2C_FMDutyCycle;       /* Possible values from @I2C_FMDUTYCYCLE */
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
