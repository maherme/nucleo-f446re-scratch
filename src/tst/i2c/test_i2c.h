/********************************************************************************************************//**
* @file test_i2c.h
*
* @brief Header file containing the prototypes of the APIs for testing the I2C peripheral.
*
* Public Functions:
*       - void    I2C1_Config(void)
*       - void    I2C1_SendHello(void)
*       - void    I2C1_SendCmd(void)
*       - void    I2C1_SendCmdIT(void)
**/

#ifndef TEST_I2C_H
#define TEST_I2C_H

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

#endif /* TEST_I2C_H */
