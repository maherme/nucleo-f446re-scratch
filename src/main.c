/********************************************************************************************************//**
* @file main.c
*
* @brief File containing the main function.
**/

#include <stdio.h>
#include "hil.h"

/** @brief Function needed for enabling semihosting */
extern void initialise_monitor_handles(void);

int main(void){

    initialise_monitor_handles();

    printf("Starting program!!!\n");

    hil_init();

    for(;;){
        hil_process();
    }

    return 0;
}
