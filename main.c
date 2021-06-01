#include <stdio.h>
#include <stdint.h>

extern void initialise_monitor_handles(void);

int main(void){

    initialise_monitor_handles();

    printf("Starting program!!!\n");

    for(;;){
    }

    return 0;
}
