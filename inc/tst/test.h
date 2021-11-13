/********************************************************************************************************//**
* @file test.h
*
* @brief Header file containing prototypes of functions for testing drivers.
*
* PUBLIC FUNCTIONS :
*       void    LED_GPIOInit(void)
*       void    Button_GPIOInit(void)
**/

#ifndef TEST_H
#define TEST_H

/***********************************************************************************************************/
/*                                       APIs Supported                                                    */
/***********************************************************************************************************/

/**
 * @brief Function to initialize and configure hardware needed for testing.
 * @return void.
 */
void test_init(void);

#endif /* TEST_H */
