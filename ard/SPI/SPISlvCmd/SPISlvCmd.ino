/********************************************************************************************************//**
* @file SPISlvCmd.ino
*
* @brief File containing the implementation of a SPI command receptor.
*
* @note
*           SPI pin:
*           SCK     13 Serial clock.
*           MISO    12 Master input slave output.
*           MOSI    11 Master output slave input.
*           SS      10 Slave select (It shall be pulled low by the master).
**/

#include <SPI.h>

/* Acknowledge status */
#define NACK                0xA5
#define ACK                 0xF5

/* Command codes */
#define CMD_LED_CTRL        0x50
#define CMD_SENSOR_READ     0x51
#define CMD_LED_READ        0x52
#define CMD_PRINT           0x53
#define CMD_ID_READ         0x54

/* LED status */
#define LED_ON              1
#define LED_OFF             0

/* Analog pins */
#define AN_PIN0             0
#define AN_PIN1             1
#define AN_PIN2             2
#define AN_PIN3             3
#define AN_PIN4             4

#define BUFFER_SIZE         255
#define SERIAL_BR           9600

static const uint8_t led = 9; /* LED digital pin */
static uint8_t led_status = HIGH;
static const uint8_t board_id[11] = "ArduinoUno";

/***********************************************************************************************************/
/*                                       Static Function Definitions                                       */
/***********************************************************************************************************/

/**
 * @brief function to initialize SPI peripheral.
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
 * @brief function to read data received by SPI peripheral.
 * @return SPDR register when data is available.
 */
static uint8_t SPIRx(void){

    /* Wait until RX is completed */
    while(!(SPSR & (1 << SPIF)));
    /* Return data received in the data register */
    return SPDR;
}

/**
 * @brief function to transmit data by SPI peripheral.
 * @param[in] data is the data to be transmitted.
 * @return void
 */
static void SPITx(uint8_t data){

    /* Store data in the data register */
    SPDR = data;
    /* Wait until TX is completed */
    while(!(SPSR & (1 << SPIF)));
}

/**
 * @brief function handling for CMD_LED_CTRL.
 * @return void
 */
static void CmdLedCtrl(void){

    uint8_t pin = 0;
    uint8_t value = 0;

    pin = SPIRx();      /* Read pin number */
    value = SPIRx();    /* Read value */

    Serial.println("Rx: CMD_LED_CTRL");

    if(value == (uint8_t)LED_ON){
        digitalWrite(pin, HIGH);
    }
    else if(value == (uint8_t)LED_OFF){
        digitalWrite(pin, LOW);
    }
    else{
        /* do nothing */
    }
}

/**
 * @brief function handling for CMD_SENSOR_READ.
 * @return void
 */
static void CmdSensorRead(void){

    uint16_t an_read = 0;
    uint8_t pin = 0;
    uint8_t value = 0;

    pin = SPIRx();
    an_read = analogRead(pin + 14);
    value = map(an_read, 0, 1023, 0, 255);

    SPITx(value);
    (void)SPIRx(); /* dummy read */

    Serial.println("Rx: CMD_SENSOR_READ");
}

/**
 * @brief function handling for CMD_LED_READ.
 * @return void
 */
static void CmdLedRead(void){

    uint8_t pin = 0;
    uint8_t value = 0;

    pin = SPIRx();
    value = digitalRead(pin);

    SPITx(value);
    (void)SPIRx(); /* dummy read */

    Serial.println("Rx: CMD_LED_READ");
}

/**
 * @brief function handling for CMD_PRINT.
 * @return void
 */
static void CmdPrint(void){

    uint8_t len = 0;
    uint8_t data_buffer[BUFFER_SIZE] = {0};

    len = SPIRx();

    for(uint8_t i = 0; i < len; i++){
        data_buffer[i] = SPIRx();
    }

    Serial.println((char*)data_buffer);
    Serial.println("Rx: CMD_PRINT");
}

/**
 * @brief function handling for CMD_ID_READ.
 * @return void
 */
static void CmdIDRead(void){

    for(uint8_t i = 0; i < strlen((char*)board_id); i++){
        SPITx(board_id[i]);
    }
    (void)SPIRx();

    Serial.println("Rx: CMD_ID_READ");
}

/***********************************************************************************************************/
/*                                       Main Functions Definitions                                        */
/***********************************************************************************************************/

void setup(){

    /* Initialize serial communication */
    Serial.begin(SERIAL_BR);

    /* Initialize LED pin */
    pinMode(led, OUTPUT);
    digitalWrite(led, LOW);

    /* Initialize SPI */
    SPIInit();
    Serial.println("Initialized as slave SPI");
}

void loop(){

    uint8_t data = 0;
    uint8_t command = 0;
    uint8_t ack = NACK;

    /* Waiting for reception */
    Serial.println("Waiting for SS to go low");
    while(digitalRead(SS));

    /* Store a command */
    command = SPIRx();
    ack = ACK;

    /* Send ACK */
    SPITx(ack);
    (void)SPIRx(); /* dummy byte */

    switch(command){
        case CMD_LED_CTRL:
            CmdLedCtrl();
            break;
        case CMD_SENSOR_READ:
            CmdSensorRead();
            break;
        case CMD_LED_READ:
            CmdLedRead();
            break;
        case CMD_PRINT:
            CmdPrint();
            break;
        case CMD_ID_READ:
            CmdIDRead();
            break;
        default:
            break;
    }
}
