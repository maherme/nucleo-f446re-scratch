/********************************************************************************************************//**
* @file hil_i2c.h
*
* @brief Header file containing the prototypes of the APIs for testing the I2C peripheral in the hardware.
*
* Public Functions:
*       - void    I2C1_Config(void)
*       - void    I2C1_SendHello(void)
*       - void    I2C1_SendCmd(void)
*       - void    I2C1_SendCmdIT(void)
**/

#ifndef HIL_I2C_H
#define HIL_I2C_H

/***********************************************************************************************************/
/*                                       APIs Supported                                                    */
/***********************************************************************************************************/

/**
 * @brief Function to initialize and configure I2C1 peripheral.
 * @return void.
 */
void I2C1_Config(void);

/**
 * @brief Function to test I2C1 peripheral transmission.
 * @return void.
 */
void I2C1_SendHello(void);

/**
 * @brief Function to test I2C1 peripheral transmission and reception.
 * @return void.
 */
void I2C1_SendCmd(void);

/**
 * @brief Function to test I2C1 peripheral transmission and reception based on interruption.
 * @return void.
 */
void I2C1_SendCmdIT(void);

#endif /* HIL_I2C_H */
