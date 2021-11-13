/********************************************************************************************************//**
* @file main.c
*
* @brief File containing the main function.
**/

#include <stdio.h>
#include "test.h"

/** @brief Function needed for enabling semihosting */
extern void initialise_monitor_handles(void);

int main(void){

    initialise_monitor_handles();

    printf("Starting program!!!\n");

    test_init();

    for(;;){
    }

    return 0;
}
