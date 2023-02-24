/********************************************************************************************************//**
* @file cortex_m4.h
*
* @brief Header file containing the prototypes of the APIs for configuring the Cortex M4.
*
* Public Functions:
*       - void IRQConfig(uint8_t IRQNumber, uint8_t en_or_di)
*       - void IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
*/

#ifndef CORTEX_M4_H
#define CORTEX_M4_H

#include <stdint.h>

/**
 * @brief Function to configure the IRQ number of the Cortex M4.
 * @param[in] IRQNumber number of the interrupt.
 * @param[in] en_or_di for enable or disable.
 * @return void.
 */
void IRQConfig(uint8_t IRQNumber, uint8_t en_or_di);

/**
 * @brief Function to configure the IRQ number of the Cortex M4.
 * @param[in] IRQNumber number of the interrupt.
 * @param[in] IRQPriority priority of the interrupt.
 * @return void.
 */
void IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

#endif /* CORTEX_M4_H */