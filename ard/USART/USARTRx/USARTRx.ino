/*****************************************************************************************************
* FILENAME :        USARTRx.ino
*
* DESCRIPTION :
*       File containing the implementation of a USART receptor.
*
* NOTES :
*           USART pin:
*           TX    1 Transmitter.
*           RX    0 Receiver.
*
**/

#define SERIAL_BR   115200

/*****************************************************************************************************/
/*                                       Main Functions Definitions                                  */
/*****************************************************************************************************/

void setup(void){

    Serial.begin(SERIAL_BR);
    Serial.println("Arduino UART Receiver");
    Serial.println("*********************");
}

void loop(void){

    char data_read = 0;

    /* Wait UART reception */
    while(!Serial.available());

    /* Read the data */
    data_read = Serial.read();
    Serial.print(data_read);
}
