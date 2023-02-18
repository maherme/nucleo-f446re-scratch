/********************************************************************************************************//**
* @file test_can.h
*
* @brief Header file containing the prototypes of the APIs for testing the CAN peripheral.
*
* Public Functions:
*       - void CAN1_Config(void)
*       - void CAN1_Send(void)
**/

#ifndef TEST_CAN_H
#define TEST_CAN_H

/***********************************************************************************************************/
/*                                       APIs Supported                                                    */
/***********************************************************************************************************/

void CAN1_Config(void);
void CAN1_Send(void);

#endif /* TEST_CAN_H */