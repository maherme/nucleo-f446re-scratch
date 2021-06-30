/*****************************************************************************************************
* FILENAME :        SPISlvRx.ino
*
* DESCRIPTION :
*       File containing the implementation of a SPI receptor.
*
* NOTES :
*           SPI pin:
*           SCK     13 Serial clock.
*           MISO    12 Master input slave output.
*           MOSI    11 Master output slave input.
*           SS      10 Slave select (It shall be pulled low by the master).
*
**/

#include <SPI.h>

#define SIZE_BUF    200     /* Maximum number of characters to be received */
#define SERIAL_BR   9600    /* Baud rate to print message by console */

static char data_buffer[SIZE_BUF] = {0};

/*****************************************************************************************************/
/*                                       Static Function Definitions                                 */
/*****************************************************************************************************/

/**
 * @fn SPIInit
 *
 * @brief function to initialize SPI peripheral.
 *
 * @param[in] void
 *
 * @return void
 */
static void SPIInit(void){
    /* Initialize SPI pins */
    pinMode(SCK, INPUT);
    pinMode(MOSI, INPUT);
    pinMode(MISO, OUTPUT);
    pinMode(SS, INPUT);

    /* Enable SPI configured as slave (bit MSTR set 0 by default) */
    SPCR = (1 << SPE);
}

/**
 * @fn SPIRx
 *
 * @brief function to read data received by SPI peripheral.
 *
 * @param[in] void
 *
 * @return SPDR register when data is available.
 */
static uint8_t SPIRx(void){

    /* Wait until RX is completed */
    while(!(SPSR & (1 << SPIF)));
    /* Return data received in the data register */
    return SPDR;
}

/*****************************************************************************************************/
/*                                       Main Functions Definitions                                  */
/*****************************************************************************************************/

void setup(){

    /* Initialize serial communication */
    Serial.begin(SERIAL_BR);
    /* Initialize SPI */
    SPIInit();
    Serial.println("Initialized as slave SPI");
}

void loop(){

    uint8_t i = 0;
    uint8_t data_len = 0;

    /* Waiting for reception */
    Serial.println("Waiting for SS to go low");
    while(digitalRead(SS));

    i = 0;
    data_len = SPIRx();
    for(i = 0; i < data_len; i++){
        data_buffer[i] = SPIRx();
    }
    data_buffer[i] = '\0';

    /* Print received data by console */
    Serial.println("Received:");
    Serial.println(data_buffer);
    Serial.println("Length:");
    Serial.println(data_len);
}
