# NUCLEO-F446RE from Scratch
This is an embedded project for [NUCLEO-F446RE](https://www.st.com/en/evaluation-tools/nucleo-f446re.html) board, based on [STM32F446RE](https://www.st.com/en/microcontrollers-microprocessors/stm32f446re.html) microcontroller.

It contains drivers for GPIO, SPI, I2C and USART peripherals.

## OpenOCD
You can use OpenOCD (Open On-Chip Debugger) for programming or debugging this project. You can starting OpenOCD typing:
```console
openocd -f board/st_nucleo_f4.cfg
```
Or using the Makefile:
```console
make load
```
You can use a telnet connection for connecting to the OpenOCD server:
```console
telnet 127.0.0.1 4444
```
You can program the microcontroller using:
```console
reset init
flash write_image erase your_app.elf
reset
```
Remember you must enable semihosting in the telnet session if you compile the project for using this feature (you can see printf outputs):
```console
arm semihosting enable
```
## Test
### Test SPI Driver
For testing the SPI driver you need to set TEST_SPI to 1 in the [test.c](src/tst/test.c) file. 

You can use the SPI2_SendHello API placed in [test_spi.c](src/tst/test_spi.c) file which sends the "Hello World" string to the Arduino board, for testing with this API you need to use the [SPISlvRx.ino](ard/SPI/SPISlvRx/SPISlvRx.ino) sketch. You need to open a Serial Monitor using the Arduino IDE configuring a speed of 9600 baud for receiving this string.

You can test the SPI driver in a bidirectional way using the SPI_SendCmds API placed in [test_spi.c](src/tst/test_spi.c) file or using the different APIs for commands in that file. For testing the command APIs you need to use the [SPISlvCmd.ino](ard/SPI/SPISlvCmd/SPISlvCmd.ino) sketch. You need to open a Serial Monitor using the Arduino IDE configuring a speed of 9600 baud and you should use also the semi hosting binary for the STM32 microcontroller for watching further information.

Finally, if you want to test the interrupt mode in the SPI driver you need to use the [SPISlvTxIrq.ino](ard/SPI/SPISlvTxIrq/SPISlvTxIrq.ino) sketch. In this test you will send a string typed in the Serial Monitor of the Arduino IDE to the Nucleo board, and it will be printed in a terminal using semi hosting. You need to open a Serial Monitor using the Arduino IDE configuring a speed of 9600 baud for typing the string, and you need to use the semi hosting binary for the STM32 microcontroller in order to watch the received string.

For performing all these tests you need to follow the connection diagram below:
![Alt text](doc/img/nucleo-spi-test.png)
In this diagram:
| PIN Functionality      | Nucleo PIN | Arduino PIN |
|:----------------------:|:----------:|:-----------:|
| SPI MISO               | PB14       | Digital 12  |
| SPI MOSI               | PB15       | Digital 11  |
| SPI SCLK               | PB13       | Digital 13  |
| SPI NSS                | PB12       | Digital 10  |
| Interrupt notification | PC9        | Digital 8   |
