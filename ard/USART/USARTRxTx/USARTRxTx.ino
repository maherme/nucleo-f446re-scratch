/********************************************************************************************************//**
* @file USARTRxTx.ino
*
* @brief File containing the implementation of a USART receptor and transmitter.
*
* @note
*           USART pin:
*           TX    1 Transmitter.
*           RX    0 Receiver.
**/

#define SERIAL_BR   115200

/***********************************************************************************************************/
/*                                       Static Function Definitions                                       */
/***********************************************************************************************************/

/**
 * @brief function to change the case to the inverse of the input character.
 * @param[in] ch is a character.
 * @return the character entered as parameter with the opposite case.
 */
static char changeCase(char ch){

    if(ch >= 'A' && ch <= 'Z'){
        ch += 32;
    }
    else if(ch >= 'a' && ch <= 'z'){
        ch -= 32;
    }
    else{
        /* do nothing */
    }

    return ch;
}

/***********************************************************************************************************/
/*                                       Main Functions Definitions                                        */
/***********************************************************************************************************/

void setup(void){

    Serial.begin(SERIAL_BR);
    Serial.println("Arduino Case Converter running ...");
    Serial.println("**********************************");
}

void loop(void){

    char data_read = 0;

    /* Wait UART reception */
    while(!Serial.available());

    /* Read the data */
    data_read = Serial.read();
    Serial.print(changeCase(data_read));
}
