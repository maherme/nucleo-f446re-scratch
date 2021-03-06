/********************************************************************************************************//**
* @file test.h
*
* @brief Header file containing prototypes of functions for testing drivers.
*
* PUBLIC FUNCTIONS :
*       void    test_init(void)
*       void    test_process(void)
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

/**
 * @brief Function to be executed as process in the main loop.
 * @return void.
 */
void test_process(void);

#endif /* TEST_H */
