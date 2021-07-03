
/*****************************************************************************************************
* FILENAME :        I2CSlvRx.ino
*
* DESCRIPTION :
*       File containing the implementation of a I2C receptor.
*
* NOTES :
*           I2C pin:
*           SCL     A5 Serial clock.
*           SDA     A4 Serial data.
*
**/

#include <Wire.h>

#define ADDR        0x68
#define SERIAL_BR   9600

/*****************************************************************************************************/
/*                                       Static Function Definitions                                 */
/*****************************************************************************************************/

static void rxHandler(int bytes){

    uint8_t cnt = 0;
    uint8_t rx_buffer[32] = {0};

    while(Wire.available()){
        rx_buffer[cnt++] = Wire.read();
    }

    rx_buffer[cnt] = '\0';
    Serial.print("Received: ");
    Serial.println((char*)rx_buffer);
}

/*****************************************************************************************************/
/*                                       Main Functions Definitions                                  */
/*****************************************************************************************************/

void setup(void){

    uint8_t message[50] = {0};

    /* Initialize serial communication */
    Serial.begin(SERIAL_BR);
    /* Initialize I2C */
    Wire.begin(ADDR);
    /* Rx callback */
    Wire.onReceive(rxHandler);

    sprintf((char*)message, "Slave is ready: address: 0x%x", ADDR);
    Serial.println((char*)message);
    Serial.println("Waiting for data from master");
}

void loop(void){
}
