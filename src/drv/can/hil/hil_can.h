/********************************************************************************************************//**
* @file hil_can.h
*
* @brief Header file containing the prototypes of the APIs for testing the CAN peripheral in the hardware.
*
* Public Functions:
*       - void CAN1_Config(void)
*       - void CAN1_Send(void)
*       - void CAN1_Send_Receive(void)
**/

#ifndef HIL_CAN_H
#define HIL_CAN_H

/***********************************************************************************************************/
/*                                       APIs Supported                                                    */
/***********************************************************************************************************/

/**
 * @brief Function to configure the CAN1 peripheral.
 * @return void
 */
void CAN1_Config(void);

/**
 * @brief Function for sending a message using CAN1 peripheral.
 * @return void
 */
void CAN1_Send(void);

/**
 * @brief Function for sending and receiving a message using CAN1 peripheral.
 * @return void
 */
void CAN1_Send_Receive(void);

#endif /* HIL_CAN_H */
