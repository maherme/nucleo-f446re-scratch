/********************************************************************************************************//**
* @file I2CMstRx.ino
*
* @brief File containing the implementation of a I2C receptor as a master.
*
* @note
*           I2C pin:
*           SCL     GPIO18 Serial clock.
*           SDA     GPIO19 Serial data.
**/

#include <Wire.h>

#define SLAVE_ADDR      0x61
#define SERIAL_BR       9600

/***********************************************************************************************************/
/*                                       Main Functions Definitions                                        */
/***********************************************************************************************************/

void setup(void){

    Serial.begin(SERIAL_BR);

    /* Join I2C bus, address is optional for master */
    Wire.begin();
}

void loop(void){

    uint8_t input = 0;
    uint8_t len = 0;
    uint8_t i = 0;
    uint8_t rx_buf[32] = {0};

    Serial.println("Arduino as I2C master");
    Serial.println("Send character \"s\" to begin");

    while(!Serial.available());
    input = Serial.read();
    Serial.println(input);
    if(input == 's'){
      Serial.println("Starting ...");

      Wire.beginTransmission(SLAVE_ADDR);
      Wire.write(0x51);
      Wire.endTransmission();

      Wire.requestFrom(SLAVE_ADDR, 1);

      if(Wire.available()){
          len = Wire.read();
      }
      Serial.print("Data length: ");
      Serial.println(String(len, DEC));

      Wire.beginTransmission(SLAVE_ADDR);
      Wire.write(0x52);
      Wire.endTransmission();

      Wire.requestFrom(SLAVE_ADDR, (int)len);

      for(i = 0; i <= len; i++){
          if(Wire.available()){
              rx_buf[i] = Wire.read();
          }
      }
      rx_buf[i] = '\0';

      Serial.print("Data: ");
      Serial.println((char*)rx_buf);
      Serial.println("***********************END***********************");
    }
}
