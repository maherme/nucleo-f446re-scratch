/*****************************************************************************************************
* FILENAME :        SPISlvTxIrq.ino
*
* DESCRIPTION :
*       File containing the implementation of a SPI transmitter.
*
* NOTES :
*           SPI pin:
*           SCK     13 Serial clock.
*           MISO    12 Master input slave output.
*           MOSI    11 Master output slave input.
*           SS      10 Slave select (It shall be pulled low by the master).
*
*           Carrier return option in the arduino serial monitor is needed.
**/

#include <SPI.h>

#define SERIAL_BR   9600
#define MAX_LEN     255

static const uint8_t pin_int = 8; /* Pin for transmission notification */

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

/**
 * @fn SPITx
 *
 * @brief function to transmit data by SPI peripheral.
 *
 * @param[in] data is the data to be transmitted.
 *
 * @return void
 */
static void SPITx(uint8_t data){

    /* Store data in the data register */
    SPDR = data;
    /* Wait until TX is completed */
    while(!(SPSR & (1 << SPIF)));
}

/**
 * @fn notify_controller
 *
 * @brief function to notify a transmission to the SPI master.
 *
 * @param[in] data is the data to be transmitted.
 *
 * @return void
 */
static void notify_controller(void){

    pinMode(pin_int, OUTPUT);
    digitalWrite(pin_int, HIGH);
    delayMicroseconds(50);
    digitalWrite(pin_int, LOW);
}

/*****************************************************************************************************/
/*                                       Main Functions Definitions                                  */
/*****************************************************************************************************/

void setup(){

    /* Initialize serial communication */
    Serial.begin(SERIAL_BR);

    /* Initialize pin for transmission notification */
    pinMode(pin_int, INPUT_PULLUP);

    /* Initialize SPI */
    SPIInit();
    Serial.println("Initialized as slave SPI");
}

void loop(){

    bool msg_complete = false;
    uint8_t user_buffer[MAX_LEN] = {0};
    uint8_t count = 0;
    uint8_t read_byte = 0;

    Serial.println("Type anything and send ... ");

    while(!msg_complete){
        if(Serial.available()){
            /* Read byte of serial data */
            read_byte = Serial.read();
            /* Store in to the buffer */
            user_buffer[count++] = read_byte;
            /* Check the end condition */
            if((read_byte == '\r') || (count == MAX_LEN)){
                msg_complete = true;
                user_buffer[count-1] = '\0'; /* Replace '\r' by '\0' */
            }
        }
    }

    Serial.println("Your message ...");
    Serial.println((char*)user_buffer);

    /* Notify master about the transmission */
    notify_controller();

    /* TX the user_buffer over SPI */
    for(uint8_t i = 0; i < count; i++){
        SPITx(user_buffer[i]);
    }
    count = 0;
    msg_complete = false;

    Serial.println("Message sent ...");

    while(!digitalRead(SS));
    Serial.println("Master ends communication");
}
