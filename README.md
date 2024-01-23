# NUCLEO-F446RE from Scratch
[![build](https://github.com/maherme/nucleo-f446re-scratch/actions/workflows/cicd.yml/badge.svg)](https://github.com/maherme/nucleo-f446re-scratch/actions)

This is an embedded project for [NUCLEO-F446RE](https://www.st.com/en/evaluation-tools/nucleo-f446re.html) board, based on [STM32F446RE](https://www.st.com/en/microcontrollers-microprocessors/stm32f446re.html) microcontroller.  
It contains drivers for GPIO, SPI, I2C, USART, RCC, TIMER, DMA, RTC, CAN and PWR peripherals.

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
For testing the SPI driver you need to use the Nucleo board, an Arduino UNO board and also a logic level converter; due to the Nucleo board works with 3.3V PIN level and Arduino UNO board works with 5V PIN level.

You need to set TEST_SPI to 1 in the [test.h](src/tst/test.h) file for enabling the code to test the SPI peripheral driver.

You can use the SPI2_SendHello API placed in [test_spi.c](src/tst/spi/test_spi.c) file which sends the "Hello World" string to the Arduino board, for testing with this API you need to use the [SPISlvRx.ino](ard/SPI/SPISlvRx/SPISlvRx.ino) sketch. You need to open a Serial Monitor using the Arduino IDE configuring a speed of 9600 baud for receiving this string.

You can test the SPI driver in a bidirectional way using the SPI_SendCmds API placed in [test_spi.c](src/tst/spi/test_spi.c) file or using the different APIs for commands in that file. For testing the command APIs you need to use the [SPISlvCmd.ino](ard/SPI/SPISlvCmd/SPISlvCmd.ino) sketch. You need to open a Serial Monitor using the Arduino IDE configuring a speed of 9600 baud and you should use also the semi hosting binary for the STM32 microcontroller for watching further information.

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

### Test I2C Driver
For testing the I2C driver you need to use the Nucleo board, an Arduino UNO board and also a logic level converter; due to the Nucleo board works with 3.3V PIN level and Arduino UNO board works with 5V PIN level.

You need to set TEST_I2C to 1 in the [test.h](src/tst/test.h) file for enabling the code to test the I2C peripheral driver.

You can use the I2C1_SendHello API placed in the [test_i2c.c](src/tst/i2c/test_i2c.c) file which sends the "Hello World" string to the Arduino board, for testing with this API you need to use the [I2CSlvRx.ino](ard/I2C/I2CSlvRx/I2CSlvRx.ino) sketch. You need to open a Serial Monitor using the Arduino IDE configuring a speed of 9600 baud for receiving this string. In this test the Nucleo board is the master so you need to set the I2C_MASTER define to 1 in the [test_i2c.c](src/tst/i2c/test_i2c.c) file.

You can test transmission and reception using the I2C1_SendCmd API placed in the [test_i2c.c](src/tst/i2c/test_i2c.c) file which sends two commands to the Arduino board and this will send an answer. These commands are:
| Command ID | Functionality                            |
|:----------:|:----------------------------------------:|
| 0x51       | Request the lenght of a string to send   |
| 0x52       | Request to send the string               |

For testing with this API you need to use the [I2CSlvCmd.ino](ard/I2C/I2CSlvCmd/I2CSlvCmd.ino) sketch.You need to open a Serial Monitor using the Arduino IDE configuring a speed of 9600 baud and you should use also the semi hosting binary for the STM32 microcontroller for watching further information. In this test the Nucleo board is the master so you need to set the I2C_MASTER define to 1 in the [test_i2c.c](src/tst/i2c/test_i2c.c) file.

If you use the I2C1_SendCmdIT API you will test the transmission and reception using interrupt mode.

You can also test the I2C driver in slave mode, you need to set the I2C_MASTER define to 0 in the [test_i2c.c](src/tst/i2c/test_i2c.c) file. For this test you need to use the [I2CMstRx.ino](ard/I2C/I2CMstRx/I2CMstRx.ino) sketch. You need to open a Serial Monitor using the Arduino IDE configuring a speed of 9600 baud for sending the commands from Arduino as master.

For performing all these tests you need to follow the connection diagram below:

![Alt text](doc/img/nucleo-i2c-test.png)

In this diagram:
| PIN Functionality      | Nucleo PIN | Arduino PIN |
|:----------------------:|:----------:|:-----------:|
| I2C SDA                | PB9        | GPIO 19     |
| I2C SLC                | PB8        | GPIO 18     |

### Test USART Driver

For testing the USART driver you need to use the Nucleo board, an Arduino UNO board and also a logic level converter; due to the Nucleo board works with 3.3V PIN level and Arduino UNO board works with 5V PIN level.

You need to set TEST_USART to 1 in the [test.h](src/tst/test.h) file for enabling the code to test the USART peripheral driver.

You can use the USART3_SendHello API placed in the [test_usart.c](src/tst/usart/test_usart.c) file which sends the "Hello World" string to the Arduino board, for testing with this API you need to use the [USARTRx.ino](ard/USART/USARTRx/USARTRx.ino) sketch. You need to open a Serial Monitor using the Arduino IDE configuring a speed of 115200 baud for receiving this string.

You can use the USART3_TxRx API for testing the transmission and reception APIs of the USART driver. This function is placed in the [test_usart.c](src/tst/usart/test_usart.c) file. You will need to use the [USARTRxTx.ino](ard/USART/USARTRxTx/USARTRxTx.ino) sketch for the Arduino board. You need to open a Serial Monitor using the Arduino IDE configuring a speed of 115200 baud for receiving this string.

For performing all these tests you need to follow the connection diagram below:

![Alt text](doc/img/nucleo-usart-test.png)

In this diagram:
| PIN Functionality      | Nucleo PIN | Arduino PIN |
|:----------------------:|:----------:|:-----------:|
| UART TX                | PC10       | GPIO 1      |
| UART RX                | PC11       | GPIO 0      |

Warning!!! You need to disconnect the cable from GPIO0 (RX PIN) of Arduino board before programming the sketch; in other way the programming process will fail.

### Test Reset and Clock Control (RCC) Driver
You need to set TEST_RCC to 1 in the [test.h](src/tst/test.h) file for enabling the code to test the RCC peripheral driver.
Each API of the test enable and cofigure a different clock configuration, SetHSEBypass() and SetPLLMax() APIs will show you the configuration set using the semihosting console, while SetMCO_LSE_HSE() and SetMCO_PLL() will send two clock signals through the GPIO PA8 and PA9.

Console output when SetHSEBypass() API is executed:
```console
Starting program!!!
PCLK1 Value: 8000000
PCLK2 Value: 8000000
PLL Output Clock: 96000000
```
Console output when SetPLLBypass() API is executed:
```console
Starting program!!!
PCLK1 Value: 45000000
PCLK2 Value: 90000000
PLL Output Clock: 180000000
```
Snapshot of GPIO PA8 (using MCO1 with LSE crystal) (D0 channel) and PC9 (using MCO2 with HSE clock and prescaler set to 4) (D1 channel) using a logic analyzer when SetMCO_LSE_HSE() API is executed:

![Alt text](doc/img/nucleo-rcc-mco-lse-hse-test.png)

### Test Timer Driver
You need to set TEST_TIMER to 1 in the [test.h](src/tst/test.h) file for enabling the code to test the Timer peripheral driver.
- If you use Timer6 related APIs you will test a basic timer (TIM6) which generates an interrupt for toggling a LED.
- If you use Timer2 related APIs you will test the input capture functionality (TIM2), if you introduce a clock signal in the GPIO PA0 the frequency will be calculated and showed using the semihosting console. For this test the MCO1 is configured using the LSE crystal and output using GPIO PA8. So connect the GPIO PA8 to PA0 and you will see this output console:

  ```console
  Starting program!!!
  Frequency: 0.001863
  Frequency: 32786.885246
  Frequency: 32786.885246
  Frequency: 32786.885246
  ...
  ```
- If you use Timer4 related APIs you will test the output compare functionality (TIM4). A four different clock sources will be generated through GPIO PB6, PB7, PB8 and PB9. If you use a logic analyzer you should get a result as the following:

  ![Alt text](doc/img/nucleo-timeroc-test.png)
- If you use Timer3 related APIs you will test the PWM functionality (TIM3). In this test an external LED connected to GPIO PA6 is controlled, the brightness of the LED is changed using a PWM which duty-cycle is modified during a period of time. You can connect the external LED as follow:
<p align="center"><img src="doc/img/nucleo-pwm-test.png" width="456" height="497"></p>

### Test DMA Driver
You need to set TEST_DMA to 1 in the [test.h](src/tst/test.h) file for enabling the code to test the DMA peripheral driver.

In this test the DMA1 Stream3 is used for sending a string stored in FLASH memory to the USART3 peripheral for transmitting.

The request for sending the string is done using the external button (B1 in nucleo board) interrupt, which also toggle a LED (LD2 in nucleo board).

For watching the string you need to connect an USB to RS232 cable converter between the nucleo board and your PC. You can use a software like minicom in your PC for monitoring the UART (115200 8N1).

The USART3 pins are configured as follow:
| PIN Functionality      | Nucleo PIN |
|:----------------------:|:----------:|
| UART TX                | PC10       |
| UART RX                | PC11       |

### Test RTC Driver
You need to set TEST_RTC to 1 in the [test.h](src/tst/test.h) file for enabling the code to test the RTC peripheral driver.

In this test the RTC is configured to an specific date and time, and the two alarm are also configured. The timer TIM6 is configured to rise an interrupt every second, and date and time are printed using this interrupt. When the alarms are triggered a message signaling this event is printed.

The alarms can be configured using the RTC alarm irq, to do this you need to set TEST_RTC_IRQ to 1 in the [test_rtc.c](src/tst/rtc/test_rtc.c) file. Is TEST_RTC_IRQ is set to 0 the check of the alarm is done via polling.

The information is printed using semihosting, so you do not need to follow any connection diagram for testing the RTC, only connect the nucleo board to the PC using the USB port.

When you press the button of the board, the date and time are restored to the initial values in the RTC.

You should be an output similar to this:

```console
Starting program!!!
Time: 11:59:50 PM
Date: 98-12-31
Time: 11:59:50 PM
Date: 98-12-31
Time: 11:59:51 PM
Date: 98-12-31
Time: 11:59:52 PM
Date: 98-12-31
Time: 11:59:53 PM
Date: 98-12-31
Time: 11:59:54 PM
Date: 98-12-31
Time: 11:59:55 PM
Date: 98-12-31
Time: 11:59:56 PM
Date: 98-12-31
Time: 11:59:57 PM
Date: 98-12-31
Time: 11:59:58 PM
Date: 98-12-31
Time: 11:59:59 PM
Date: 98-12-31
ALARM A!!!
Time: 12:00:00 AM
Date: 99-01-01
Time: 12:00:01 AM
Date: 99-01-01
...
Time: 12:00:28 AM
Date: 99-01-01
Time: 12:00:28 AM
Date: 99-01-01
Time: 12:00:29 AM
Date: 99-01-01
ALARM B!!!
Time: 12:00:30 AM
Date: 99-01-01
Time: 12:00:31 AM
Date: 99-01-01
Time: 12:00:32 AM
...
Time: 12:01:27 AM
Date: 99-01-01
Time: 12:01:28 AM
Date: 99-01-01
Time: 12:01:29 AM
Date: 99-01-01
ALARM B!!!
Time: 12:01:30 AM
Date: 99-01-01
Time: 12:01:31 AM
Date: 99-01-01
Time: 12:01:32 AM
Date: 99-01-01
```

### Test PWR Driver
You need to set TEST_PWR to 1 in the [test.h](src/tst/test.h) file for enabling the code to test the PWR peripheral driver.  You can use a multimeter connected to the IDD pins in the nucleo board to measure the consumption of the microcontroller. You will need to be cautious if you connect a debug probe to the device, since the debug event can wake up or prevent to enter the device into low power mode; for this reason some traces are printed using the USART3 peripheral.  
You can follow the connection diagram below for performing these tests:

![Alt text](doc/img/nucleo-pwr-test.png)

The USART3 pins are configured as follow:
| PIN Functionality      | Nucleo PIN |
|:----------------------:|:----------:|
| UART TX                | PC10       |


- If you use ```Test_PWR_SetPLLMax``` function (uncomment this function from [test.c](src/tst/test.c) file) you will set the system clock to the maximum allowed frequency using the PLL when you push the user button.
- If you use the ```Test_SleepOnExit``` function (uncomment this function from [test.c](src/tst/test.c) file) the device will enter in SLEEPONEXIT mode, so it will be in low power mode until an interrupt occurs, a timer (TIM6) irq is set to wake up the device periodically.
- If you use the ```Test_WFE_init``` and ```Test_WFE_process``` functions (uncomment these functions from [test.c](src/tst/test.c) file) the device will execute the WFE instuction and it will wake up when the user button was pressed (the SEVONPEND functionality is enable for waking up from WFE).
- If you use the ```Test_BKRAM_init``` and ```Test_BKRAM_process``` functions (uncomment these functions from [test.c](src/tst/test.c) file) the device will write some data in the backup SRAM and will enter in standby mode when you press the user button. You need to generate an event connecting the PA0 pin to the VDD in order to wake up the device. If you use the ```PWR_EnableBackupRegulator``` in the ```Test_BKRAM_process``` function, the data written in the backup SRAM will be preserved, if you do not use this function, the data will be lost.  
You should get an output like this when the backup regulator is disable:

  ```console
  Press user button for entering standby mode
  Entering in standby mode
  Wake up from standby mode
  Backup SRAM not preserved
  Press user button for entering standby mode
  ```
  You should get an output like this when the backup regulator is enable:
  
  ```console
  Press user button for entering standby mode
  Entering in standby mode
  Wake up from standby mode
  Backup SRAM preserved
  Press user button for entering standby mode
  ```
- If you use the ```Test_StopMode_init```, ```Test_StopMode_process``` and ```Test_StopMode_irq``` functions (uncomment these functions from [test.c](src/tst/test.c) file) the device will enter in stop mode. Depending of your selection when calling the ```PWR_EnterStopMode``` function inside the ```Test_StopMode_process``` function placed in the file [test_pwr.c](src/tst/pwr/test_pwr.c), the device will enter in one of the all possible stop modes (for futher information about this consult the reference manual of the microcontroller). For exiting the stop mode you need to press the user button in the board, a message indicating the exit will be printed, and the program will executed the code for entering in the stop mode again, printing a message also.  
You should get an output like this:

  ```console
  [2023-04-05 19:26:16] Entering in stop mode
  [2023-04-05 19:26:22] Exiting from stop mode
  [2023-04-05 19:26:22] Entering in stop mode
  [2023-04-05 19:26:36] Exiting from stop mode
  [2023-04-05 19:26:36] Entering in stop mode
  ```
