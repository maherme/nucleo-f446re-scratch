/********************************************************************************************************//**
* @file utils.c
*
* @brief File containing utility functions.
*
* Public Functions:
*       - void    delay(void)
*
* @note
*       For further information about functions refer to the corresponding header file.
**/

#include <stdint.h>

/***********************************************************************************************************/
/*                                       Public API Definitions                                            */
/***********************************************************************************************************/

void delay(void){
    for(uint32_t i = 0; i < 500000; i++);
}
