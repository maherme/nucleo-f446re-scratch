/*****************************************************************************************************
* FILENAME :        I2CSlvCmd.ino
*
* DESCRIPTION :
*       File containing the implementation of a I2C transmitter and receptor.
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

static uint8_t active_cmd = 0xFF;

/*****************************************************************************************************/
/*                                       Static Function Definitions                                 */
/*****************************************************************************************************/

/**
 * @fn rxHandler
 *
 * @brief function to be called when this slave device receives a transmission from a master.
 *
 * @param[in] bytes is the number of bytes read from the master.
 *
 * @return void
 */
static void rxHandler(int bytes){

    active_cmd = Wire.read();
    Serial.println("Rx: command received");
}

/**
 * @fn rqHandler
 *
 * @brief function to be called when a master requests data from this slave device.
 *
 * @param[in] void
 *
 * @return void
 */
static void rqHandler(void){

    uint8_t msg[32] = "Hello from Arduino!\n";

    if(active_cmd == 0x51){
        uint8_t len = strlen((char*)msg);
        Wire.write(&len, 1);
        active_cmd = 0xFF;
        Serial.println("Tx: command 0x51");
    }

    if(active_cmd == 0x52){
        Wire.write(msg, strlen((char*)msg));
        active_cmd = 0xFF;
        Serial.println("Tx: command 0x52");
    }
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
    Wire.onRequest(rqHandler);

    sprintf((char*)message, "Slave is ready: address: 0x%x", ADDR);
    Serial.println((char*)message);
    Serial.println("Waiting for data from master");
}

void loop(void){
}
