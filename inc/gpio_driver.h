/*****************************************************************************************************
* FILENAME :        gpio_driver.h
*
* DESCRIPTION :
*       Header file containing the prototypes of the APIs for configuring the GPIO peripheral.
*
* PUBLIC FUNCTIONS :
*       void        GPIO_Init(GPIO_Handle_t* pGPIOHandle) 
*       void        GPIO_DeInit(GPIO_RegDef_t* pGPIOx)
*       void        GPIO_PerClkCtrl(GPIO_RegDef_t* pGPIOx, uint8_t en_or_di)
*       uint8_t     GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t pin_number)
*       uint16_t    GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx)
*       void        GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pin_number, uint8_t value)
*       void        GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t value)
*       void        GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pin_number)
*       void        GPIO_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di)
*       void        GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
*       void        GPIO_IRQHandling(uint8_t pin_number)
*
**/

#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include <stdint.h>
#include "stm32f446xx.h"

/**
 * @GPIO_PIN_NO
 * GPIO pin possible number.
 */
#define GPIO_PIN_NO_0       0
#define GPIO_PIN_NO_1       1
#define GPIO_PIN_NO_2       2
#define GPIO_PIN_NO_3       3
#define GPIO_PIN_NO_4       4
#define GPIO_PIN_NO_5       5
#define GPIO_PIN_NO_6       6
#define GPIO_PIN_NO_7       7
#define GPIO_PIN_NO_8       8
#define GPIO_PIN_NO_9       9
#define GPIO_PIN_NO_10      10
#define GPIO_PIN_NO_11      11
#define GPIO_PIN_NO_12      12
#define GPIO_PIN_NO_13      13
#define GPIO_PIN_NO_14      14
#define GPIO_PIN_NO_15      15

/**
 * @GPIO_PIN_MODES
 * GPIO pin possible modes.
 */
#define GPIO_MODE_IN        0   /* Input direction mode */
#define GPIO_MODE_OUT       1   /* General purpose output mode */
#define GPIO_MODE_ALTFN     2   /* Alternate function mode */
#define GPIO_MODE_ANALOG    3   /* Analog mode */
#define GPIO_MODE_IT_FT     4   /* Interrupt falling edge trigger */
#define GPIO_MODE_IT_RT     5   /* Interrupt rising edge trigger */
#define GPIO_MODE_IT_RFT    6   /* Interrupt rising and falling edge trigger */

/**
 * @GPIO_PIN_OPTYPE
 * GPIO pin possible output types.
 */
#define GPIO_OP_TYPE_PP     0   /* Output push-pull */
#define GPIO_OP_TYPE_OD     1   /* Output open-drain */

/**
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speed.
 */
#define GPIO_SPEED_LOW      0
#define GPIO_SPEED_MEDIUM   1
#define GPIO_SPEED_FAST     2
#define GPIO_SPEED_HIGH     3

/**
 * @GPIO_PIN_PULL
 * GPIO pin pull-up / pull-down configuration.
 */
#define GPIO_NO_PULL        0   /* No pull-up, pull-down */
#define GPIO_PIN_PU         1   /* Pull-up */
#define GPIO_PIN_PD         2   /* Pull-down */

/**
 * Configuration structure for a GPIO pin.
 */
typedef struct
{
    uint8_t GPIO_PinNumber;         /* Possible values from @GPIO_PIN_NO */
    uint8_t GPIO_PinMode;           /* Possible values from @GPIO_PIN_MODES */
    uint8_t GPIO_PinSpeed;          /* Possible values from @GPIO_PIN_SPEED */
    uint8_t GPIO_PinPuPdControl;    /* Possible values from @GPIO_PIN_PULL */
    uint8_t GPIO_PinOPType;         /* Possible values from @GPIO_PIN_OPTYPE */
    uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/**
 * Handle structure for a GPIO pin.
 */
typedef struct
{
    GPIO_RegDef_t* pGPIOx;              /* Base address of the GPIO port to which the pin belongs */
    GPIO_PinConfig_t GPIO_PinConfig;    /* GPIO pin configuration settings */
}GPIO_Handle_t;

/*****************************************************************************************************/
/*                                       APIs Supported                                              */
/*****************************************************************************************************/

/**
 * @fn GPIO_Init
 *
 * @brief function to initialize GPIO port.
 *
 * @param[in] pGPIOHandle handle structure for the GPIO pin.
 *
 * @return void
 */
void GPIO_Init(GPIO_Handle_t* pGPIOHandle);

/**
 *@fn GPIO_DeInit
 *
 * @brief function to reset all register of a GPIO port.
 *
 * @param[in] pGPIOx the base address of the GPIOx peripheral port.
 *
 * @return void
 */
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);

/**
 * @fn GPIO_PerClkCtrl
 *
 * @brief function to control the peripheral clock of the GPIO port.
 *
 * @param[in] pGPIOx the base address of the GPIOx peripheral port.
 * @param[in] en_or_di for enable or disable.
 *
 * @return void
 */
void GPIO_PerClkCtrl(GPIO_RegDef_t* pGPIOx, uint8_t en_or_di);

/**
 * @fn GPIO_ReadFromInputPin
 *
 * @brief function to read from an input pin of the GPIO port.
 *
 * @param[in] pGPIOx the base address of the GPIOx peripheral port.
 * @param[in] pin_number the pin number of the port.
 *
 * @return value of the pin: 1 or 0.
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t pin_number);

/**
 * @fn GPIO_ReadFromInputPort
 *
 * @brief function to read the GPIO port.
 *
 * @param[in] pGPIOx the base address of the GPIOx peripheral port.
 *
 * @return value of the all pins of the GPIO port.
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx);

/**
 * @fn GPIO_WriteToOutputPin
 *
 * @brief function to write to output pin of the GPIO port.
 *
 * @param[in] pGPIOx the base address of the GPIOx peripheral port.
 * @param[in] pin_number the pin number of the port.
 * @param[in] value to write to output in the pin.
 *
 * @return void.
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pin_number, uint8_t value);

/**
 * @fn GPIO_WriteToOutputPort
 *
 * @brief function to write to output of the entire GPIO port.
 *
 * @param[in] pGPIOx the base address of the GPIOx peripheral port.
 * @param[in] value to write to output in the port.
 *
 * @return void.
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t value);

/**
 * @fn GPIO_ToggleOutputPin
 *
 * @brief function to toggle the output pin of the GPIO port.
 *
 * @param[in] pGPIOx the base address of the GPIOx peripheral port.
 * @param[in] pin_number the pin number of the port.
 *
 * @return void.
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pin_number);

/**
 * @fn GPIO_IRQConfig
 *
 * @brief function to configure the IRQ number of the GPIO pin.
 *
 * @param[in] IRQNumber number of the interrupt.
 * @param[in] en_or_di for enable or disable.
 *
 * @return void.
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di);

/**
 * @fn GPIO_IRQPriorityConfig
 *
 * @brief function to configure the IRQ number of the GPIO pin.
 *
 * @param[in] IRQNumber number of the interrupt.
 * @param[in] IRQPriority priority of the interrupt.
 *
 * @return void.
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * @fn GPIO_IRQHandling
 *
 * @brief function to handle the interrupt of the GPIO pin.
 *
 * @param[in] pin_number the pin number of the port.
 *
 * @return void.
 */
void GPIO_IRQHandling(uint8_t pin_number);

#endif
