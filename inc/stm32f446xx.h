/********************************************************************************************************//**
* @file stm32f446xx.h
*
* @brief File containing macros definition and structures for configuring the microcontroller.
*
* @note
*       These file is for the specific STM32F446xx microcontroller.
*/

#ifndef STM32F446XX_H
#define STM32F446XX_H

#include <stdint.h>

/***********************************************************************************************************/
/*                          Generic macros                                                                 */
/***********************************************************************************************************/

/**
 * @name Some Generic Macros.
 * @{
 */
#define ENABLE              1           /**< @brief Enable value */
#define DISABLE             0           /**< @brief Disable value */
#define SET                 ENABLE      /**< @brief Set value */
#define RESET               DISABLE     /**< @brief Reset value */
#define GPIO_PIN_SET        SET         /**< @brief Set value for GPIO PIN */
#define GPIO_PIN_RESET      RESET       /**< @brief Reset value for GPIO PIN */
#define FLAG_SET            SET         /**< @brief Set value for a flag */
#define FLAG_RESET          RESET       /**< @brief Reset value for a flag */
/** @} */

/***********************************************************************************************************/
/*                          ARM Cortex M4 Processor Specific Registers                                     */
/***********************************************************************************************************/

/**
 * @name ARM Cortex M4 Processor Specific Registers.
 * @{
 */
#define NVIC_ISER0      ((volatile uint32_t*)0xE000E100)        /**< @brief NVIC ISER0 Register Addr */
#define NVIC_ISER1      ((volatile uint32_t*)0xE000E104)        /**< @brief NVIC ISER1 Register Addr */
#define NVIC_ISER2      ((volatile uint32_t*)0xE000E108)        /**< @brief NVIC ISER2 Register Addr */
#define NVIC_ISER3      ((volatile uint32_t*)0xE000E10C)        /**< @brief NVIC ISER3 Register Addr */

#define NVIC_ICER0      ((volatile uint32_t*)0xE000E180)        /**< @brief NVIC ICER0 Register Addr */
#define NVIC_ICER1      ((volatile uint32_t*)0xE000E184)        /**< @brief NVIC ICER1 Register Addr */
#define NVIC_ICER2      ((volatile uint32_t*)0xE000E188)        /**< @brief NVIC ICER2 Register Addr */
#define NVIC_ICER3      ((volatile uint32_t*)0xE000E18C)        /**< @brief NVIC ICER3 Register Addr */

#define NVIC_PR_BASEADDR    ((volatile uint32_t*)0xE000E400)    /**< @brief NVIC Priority Register Addr */

#define DBGMCU_BASEADDR     0xE0042000                          /**< @brief Debug Peripheral Base Addr */

#define NO_PR_BITS_IMPLEMENTED  4 /**< @brief Numb of priority bits implemented in the Priority Register */
/** @} */

/***********************************************************************************************************/
/*                          Memory and Bus Base Address Definition                                         */
/***********************************************************************************************************/

/**
 * @name Base and End Addresses of Memories.
 * @{
 */
#define FLASH_BASEADDR      0x08000000U     /**< @brief Base addr of flash memory, size 512KB */
#define FLASH_ENDADDR       0x081FFFFFU     /**< @brief End addr of the total flash memory */
#define SRAM1_BASEADDR      0x20000000U     /**< @brief Base addr of system RAM first block, size 112KB */
#define SRAM1_ENDADDR       0x2001BFFFU     /**< @brief End addr of the total SRAM1 memory */
#define SRAM2_BASEADDR      0x2001C000U     /**< @brief Base addr of system RAM second block, size 16KB */
#define SRAM2_ENDADDR       0x2001FFFFU     /**< @brief End addr of the total SRAM2 memory */
#define SRAM                SRAM1_BASEADDR  /**< @brief Base addr of the SRAM memory, size 128KB */
#define ROM                 0x1FFF0000U     /**< @brief Base addr of system memory or ROM, size 30KB */
#define BKPSRAM_ENDADDR     0x40024FFFU     /**< @brief End addr of the total backup SRAM, size 4KB */
#define OTP_BASEADDR        0x1FFF7800      /**< @brief Base addr of One-time programmable bytes */
#define OTP_SECTOR_SIZE     32              /**< @brief Size of OTP sector in bytes */
/** @} */

/**
 * @name Base Addresses of AHBx and APBx Bus Peripheral.
 * @{
 */
#define PERIPH_BASEADDR     0x40000000U     /**< @brief Base addr of peripherals */
#define APB1_BASEADDR       PERIPH_BASEADDR /**< @brief Base addr of peripheral connected to APB1 bus */
#define APB2_BASEADDR       0x40010000U     /**< @brief Base addr of peripheral connected to APB2 bus */
#define AHB1_BASEADDR       0x40020000U     /**< @brief Base addr of peripheral connected to AHB1 bus */
#define AHB2_BASEADDR       0x50000000U     /**< @brief Base addr of peripheral connected to AHB2 bus */
#define AHB3_BASEADDR       0xA0000000U     /**< @brief Base addr of peripheral connected to AHB3 bus */
/** @} */

/***********************************************************************************************************/
/*                          Peripheral Base Address Definition                                             */
/***********************************************************************************************************/

/**
 * @name Base addresses of peripheral connected to AHB1 bus.
 * @{
*/
#define GPIOA_BASEADDR      (AHB1_BASEADDR + 0x0000)    /**< @brief Base address of GPIOA */
#define GPIOB_BASEADDR      (AHB1_BASEADDR + 0x0400)    /**< @brief Base address of GPIOB */
#define GPIOC_BASEADDR      (AHB1_BASEADDR + 0x0800)    /**< @brief Base address of GPIOC */
#define GPIOD_BASEADDR      (AHB1_BASEADDR + 0x0C00)    /**< @brief Base address of GPIOD */
#define GPIOE_BASEADDR      (AHB1_BASEADDR + 0x1000)    /**< @brief Base address of GPIOE */
#define GPIOF_BASEADDR      (AHB1_BASEADDR + 0x1400)    /**< @brief Base address of GPIOF */
#define GPIOG_BASEADDR      (AHB1_BASEADDR + 0x1800)    /**< @brief Base address of GPIOG */
#define GPIOH_BASEADDR      (AHB1_BASEADDR + 0x1C00)    /**< @brief Base address of GPIOH */
#define CRC_BASEADDR        (AHB1_BASEADDR + 0x3000)    /**< @brief Base address of CRC */
#define RCC_BASEADDR        (AHB1_BASEADDR + 0x3800)    /**< @brief Base address of RCC */
#define FLASHINTR_BASEADDR  (AHB1_BASEADDR + 0x3C00)    /**< @brief Base address of flash interface regist */
#define BKPSRAM_BASEADDR    (AHB1_BASEADDR + 0x4000)    /**< @brief Base address of BKPSRAM */
#define DMA1_BASEADDR       (AHB1_BASEADDR + 0x6000)    /**< @brief Base address of DMA1 */
#define DMA2_BASEADDR       (AHB1_BASEADDR + 0x6400)    /**< @brief Base address of DMA2 */
#define USBOTGHS_BASEADDR   (AHB1_BASEADDR + 0x40000)   /**< @brief Base address of USB OTG HS */
/** @} */

/**
 * @name Base addresses of peripheral connected to APB1 bus.
 * @{
 */
#define TIM2_BASEADDR       (APB1_BASEADDR + 0x0000)    /**< @brief Base address of TIM2 */
#define TIM3_BASEADDR       (APB1_BASEADDR + 0x0400)    /**< @brief Base address of TIM3 */
#define TIM4_BASEADDR       (APB1_BASEADDR + 0x0800)    /**< @brief Base address of TIM4 */
#define TIM5_BASEADDR       (APB1_BASEADDR + 0x0C00)    /**< @brief Base address of TIM5 */
#define TIM6_BASEADDR       (APB1_BASEADDR + 0x1000)    /**< @brief Base address of TIM6 */
#define TIM7_BASEADDR       (APB1_BASEADDR + 0x1400)    /**< @brief Base address of TIM7 */
#define TIM12_BASEADDR      (APB1_BASEADDR + 0x1800)    /**< @brief Base address of TIM12 */
#define TIM13_BASEADDR      (APB1_BASEADDR + 0x1C00)    /**< @brief Base address of TIM13 */
#define TIM14_BASEADDR      (APB1_BASEADDR + 0x2000)    /**< @brief Base address of TIM14 */
#define RTCBKP_BASEADDR     (APB1_BASEADDR + 0x2800)    /**< @brief Base address of RTC and BKP registers */
#define WWDG_BASEADDR       (APB1_BASEADDR + 0x2C00)    /**< @brief Base address of WWDG */
#define IWDG_BASEADDR       (APB1_BASEADDR + 0x3000)    /**< @brief Base address of IWDG */
#define SPI2_I2S2_BASEADDR  (APB1_BASEADDR + 0x3800)    /**< @brief Base address of SPI2 / I2S2 */
#define SPI3_I2S3_BASEADDR  (APB1_BASEADDR + 0x3C00)    /**< @brief Base address of SPI3 / I2S3 */
#define SPDIF_RX_BASEADDR   (APB1_BASEADDR + 0x4000)    /**< @brief Base address of SPDIF-RX */
#define USART2_BASEADDR     (APB1_BASEADDR + 0x4400)    /**< @brief Base address of USART2 */
#define USART3_BASEADDR     (APB1_BASEADDR + 0x4800)    /**< @brief Base address of USART3 */
#define UART4_BASEADDR      (APB1_BASEADDR + 0x4C00)    /**< @brief Base address of UART4 */
#define UART5_BASEADDR      (APB1_BASEADDR + 0x5000)    /**< @brief Base address of UART5 */
#define I2C1_BASEADDR       (APB1_BASEADDR + 0x5400)    /**< @brief Base address of I2C1 */
#define I2C2_BASEADDR       (APB1_BASEADDR + 0x5800)    /**< @brief Base address of I2C2 */
#define I2C3_BASEADDR       (APB1_BASEADDR + 0x5C00)    /**< @brief Base address of I2C3 */
#define CAN1_BASEADDR       (APB1_BASEADDR + 0x6400)    /**< @brief Base address of CAN1 */
#define CAN2_BASEADDR       (APB1_BASEADDR + 0x6800)    /**< @brief Base address of CAN2 */
#define HDMI_CEC_BASEADDR   (APB1_BASEADDR + 0x6C00)    /**< @brief Base address of HDMI-CEC */
#define PWR_BASEADDR        (APB1_BASEADDR + 0x7000)    /**< @brief Base address of PWR */
#define DAC_BASEADDR        (APB1_BASEADDR + 0x7400)    /**< @brief Base address of DAC */
/** @} */

/**
 * @name Base addresses of peripheral connected to AHB2 bus.
 * @{
 */
#define USBOTGFS_BASEADDR   (AHB2_BASEADDR + 0x0000)    /**< @brief Base address of USB OTG FS */
#define DCMI_BASEADDR       (AHB2_BASEADDR + 0x50000)   /**< @brief Base address of DCMI */
/** @} */

/**
 * @name Base addresses of peripheral connected to APB2 bus.
 * @{
 */
#define TIM1_BASEADDR       (APB2_BASEADDR + 0x0000)    /**< @brief Base address of TIM1 */
#define TIM8_BASEADDR       (APB2_BASEADDR + 0x0400)    /**< @brief Base address of TIM8 */
#define USART1_BASEADDR     (APB2_BASEADDR + 0x1000)    /**< @brief Base address of USART1 */
#define USART6_BASEADDR     (APB2_BASEADDR + 0x1400)    /**< @brief Base address of USART6 */
#define ADC1_2_3_BASEADDR   (APB2_BASEADDR + 0x2000)    /**< @brief Base address of ADC1 - ADC2 - ADC3 */
#define SDMMC_BASEADDR      (APB2_BASEADDR + 0x2C00)    /**< @brief Base address of SDMMC */
#define SPI1_BASEADDR       (APB2_BASEADDR + 0x3000)    /**< @brief Base address of SPI1 */
#define SPI4_BASEADDR       (APB2_BASEADDR + 0x3400)    /**< @brief Base address of SPI4 */
#define SYSCFG_BASEADDR     (APB2_BASEADDR + 0x3800)    /**< @brief Base address of SYSCFG */
#define EXTI_BASEADDR       (APB2_BASEADDR + 0x3C00)    /**< @brief Base address of EXTI */
#define TIM9_BASEADDR       (APB2_BASEADDR + 0x4000)    /**< @brief Base address of TIM9 */
#define TIM10_BASEADDR      (APB2_BASEADDR + 0x4400)    /**< @brief Base address of TIM10 */
#define TIM11_BASEADDR      (APB2_BASEADDR + 0x4800)    /**< @brief Base address of TIM11 */
#define SAI1_BASEADDR       (APB2_BASEADDR + 0x5800)    /**< @brief Base address of SAI1 */
#define SAI2_BASEADDR       (APB2_BASEADDR + 0x5C00)    /**< @brief Base address of SAI2 */
/** @} */

/**
 * @name Base addresses of peripheral connected to AHB3 bus.
 * @{
 */
#define FMC_BASEADDR        (AHB3_BASEADDR + 0x0000)    /**< @brief Base addr of FMC control register */
#define QUADSPI_BASEADDR    (AHB3_BASEADDR + 0x1000)    /**< @brief Base addr of QUADSPI register */
/** @} */

/***********************************************************************************************************/
/*                          Peripheral Register Definition Structures                                      */
/***********************************************************************************************************/

/**
 * @brief Peripheral register definition structure for GPIO.
 */
typedef struct
{
    volatile uint32_t MODER;    /**< @brief GPIO port mode register                      Addr offset 0x00 */
    volatile uint32_t OTYPER;   /**< @brief GPIO port output type register               Addr offset 0x04 */
    volatile uint32_t OSPEEDER; /**< @brief GPIO port output speed register              Addr offset 0x08 */
    volatile uint32_t PUPDR;    /**< @brief GPIO port pull-up/pull-down register         Addr offset 0x0C */
    volatile uint32_t IDR;      /**< @brief GPIO port input data register                Addr offset 0x10 */
    volatile uint32_t ODR;      /**< @brief GPIO port output data register               Addr offset 0x14 */
    volatile uint32_t BSRR;     /**< @brief GPIO port bit set/reset register             Addr offset 0x18 */
    volatile uint32_t LCKR;     /**< @brief GPIO port configuration lock register        Addr offset 0x1C */
    volatile uint32_t AFR[2];   /**< @brief GPIO alternate funct low register AFR[0]  Addr offset 0x20<br> */
                                /**< @brief GPIO alternate function high register AFR[1] Addr offset 0x24 */
} GPIO_RegDef_t;

/**
 * @brief Peripheral register definition structure for RCC.
 */
typedef struct
{
    volatile uint32_t CR;           /**< @brief RCC clock control register                   Addr off 0x00 */
    volatile uint32_t PLLCFGR;      /**< @brief RCC PLL configuration register               Addr off 0x04 */
    volatile uint32_t CFGR;         /**< @brief RCC clock configuration register             Addr off 0x08 */
    volatile uint32_t CIR;          /**< @brief RCC clock interrupt register                 Addr off 0x0C */
    volatile uint32_t AHB1RSTR;     /**< @brief RCC AHB1 peripheral reset register           Addr off 0x10 */
    volatile uint32_t AHB2RSTR;     /**< @brief RCC AHB2 peripheral reset register           Addr off 0x14 */
    volatile uint32_t AHB3RSTR;     /**< @brief RCC AHB3 peripheral reset register           Addr off 0x18 */
    uint32_t RESERVED0;             /**< @brief Reserved                                     Addr off 0x1C */
    volatile uint32_t APB1RSTR;     /**< @brief RCC APB1 peripheral reset register           Addr off 0x20 */
    volatile uint32_t APB2RSTR;     /**< @brief RCC APB2 peripheral reset register           Addr off 0x24 */
    uint32_t RESERVED1;             /**< @brief Reserved                                     Addr off 0x28 */
    uint32_t RESERVED2;             /**< @brief Reserved                                     Addr off 0x2C */
    volatile uint32_t AHB1ENR;      /**< @brief RCC AHB1 peripheral clock enable register    Addr off 0x30 */
    volatile uint32_t AHB2ENR;      /**< @brief RCC AHB2 peripheral clock enable register    Addr off 0x34 */
    volatile uint32_t AHB3ENR;      /**< @brief RCC AHB3 peripheral clock enable register    Addr off 0x38 */
    uint32_t RESERVED3;             /**< @brief Reserved                                     Addr off 0x3C */
    volatile uint32_t APB1ENR;      /**< @brief RCC APB1 peripheral clock enable register    Addr off 0x40 */
    volatile uint32_t APB2ENR;      /**< @brief RCC APB2 peripheral clock enable register    Addr off 0x44 */
    uint32_t RESERVED4;             /**< @brief Reserved                                     Addr off 0x48 */
    uint32_t RESERVED5;             /**< @brief Reserved                                     Addr off 0x4C */
    volatile uint32_t AHB1LPENR;    /**< @brief RCC AHB1 peri clk en in low power mode reg   Addr off 0x50 */
    volatile uint32_t AHB2LPENR;    /**< @brief RCC AHB2 peri clk en in low power mode reg   Addr off 0x54 */
    volatile uint32_t AHB3LPENR;    /**< @brief RCC AHB3 peri clk en in low power mode reg   Addr off 0x58 */
    uint32_t RESERVED6;             /**< @brief Reserved                                     Addr off 0x5C */
    volatile uint32_t APB1LPENR;    /**< @brief RCC APB1 peri clk en in low power mode reg   Addr off 0x60 */
    volatile uint32_t APB2LPENR;    /**< @brief RCC APB2 peri clk en in low power mode reg   Addr off 0x64 */
    uint32_t RESERVED7;             /**< @brief Reserved                                     Addr off 0x68 */
    uint32_t RESERVED8;             /**< @brief Reserved                                     Addr off 0x6C */
    volatile uint32_t BDCR;         /**< @brief RCC Backup domain control register           Addr off 0x70 */
    volatile uint32_t CSR;          /**< @brief RCC clock control & status register          Addr off 0x74 */
    uint32_t RESERVED9;             /**< @brief Reserved                                     Addr off 0x78 */
    uint32_t RESERVED10;            /**< @brief Reserved                                     Addr off 0x7C */
    volatile uint32_t SSCGR;        /**< @brief RCC spread spectrum clk generation reg       Addr off 0x80 */
    volatile uint32_t PLLI2SCFGR;   /**< @brief RCC PLLI2S configuration register            Addr off 0x84 */
    volatile uint32_t PLLSAICFGR;   /**< @brief RCC PLL configuration register               Addr off 0x88 */
    volatile uint32_t DCKCFGR;      /**< @brief RCC dedicated clock configuration register   Addr off 0x8C */
    volatile uint32_t CKGATENR;     /**< @brief RCC clocks gated enable register             Addr off 0x90 */
    volatile uint32_t DCKCFGR2;     /**< @brief RCC dedicated clocks configuration reg 2     Addr off 0x94 */
} RCC_RegDef_t;

/**
 * @brief Peripheral register definition structure for EXTI.
 */
typedef struct
{
    volatile uint32_t IMR;      /**< @brief Interrupt mask register              Address offset 0x00 */
    volatile uint32_t EMR;      /**< @brief Event mask register                  Address offset 0x04 */
    volatile uint32_t RTSR;     /**< @brief Rising trigger selection register    Address offset 0x08 */
    volatile uint32_t FTSR;     /**< @brief Falling trigger selection register   Address offset 0x0C */
    volatile uint32_t SWIER;    /**< @brief Software interrupt event register    Address offset 0x10 */
    volatile uint32_t PR;       /**< @brief Pending register                     Address offset 0x14 */
}EXTI_RegDef_t;

/**
 * @brief Peripheral register definition structure for SYSCFG.
 */
typedef struct
{
    volatile uint32_t MEMRMP;       /**< @brief SYSCFG memory remap register            Addr offset 0x00 */
    volatile uint32_t PMC;          /**< @brief SYSCFG peripheral mode config register  Addr offset 0x04 */
    volatile uint32_t EXTICR[4];    /**< @brief SYSCFG ext int cfg reg 1 EXTICR[0]  Addr offset 0x08<br> */
                                    /**< @brief SYSCFG ext int cfg reg 2 EXTICR[1]  Addr offset 0x0C<br> */
                                    /**< @brief SYSCFG ext int cfg reg 3 EXTICR[2]  Addr offset 0x10<br> */
                                    /**< @brief SYSCFG ext int cfg reg 4 EXTICR[3]  Addr offset 0x14 */
    uint32_t RESERVED0;             /**< @brief Reserved                                Addr offset 0x18 */
    uint32_t RESERVED1;             /**< @brief Reserved                                Addr offset 0x1C */
    volatile uint32_t CMPCR;        /**< @brief Compensation cell control register      Addr offset 0x20 */
    uint32_t RESERVED2;             /**< @brief Reserved                                Addr offset 0x24 */
    uint32_t RESERVED3;             /**< @brief Reserved                                Addr offset 0x28 */
    volatile uint32_t CFGR;         /**< @brief SYSCFG configuration register           Addr offset 0x2C */
}SYSCFG_RegDef_t;

/**
 * @brief Peripheral register definition structure for SPI.
 */
typedef struct
{
    volatile uint32_t CR1;          /**< @brief SPI control register 1          Address offset 0x00 */
    volatile uint32_t CR2;          /**< @brief SPI control register 2          Address offset 0x04 */
    volatile uint32_t SR;           /**< @brief SPI status register             Address offset 0x08 */
    volatile uint32_t DR;           /**< @brief SPI data register               Address offset 0x0C */
    volatile uint32_t CRCPR;        /**< @brief SPI CRC polynomial register     Address offset 0x10 */
    volatile uint32_t RXCRCR;       /**< @brief SPI RX CRC register             Address offset 0x14 */
    volatile uint32_t TXCRCR;       /**< @brief SPI TX CRC register             Address offset 0x18 */
    volatile uint32_t I2SCFGR;      /**< @brief SPI_I2S configuration register  Address offset 0x1C */
    volatile uint32_t I2SPR;        /**< @brief SPI_I2S prescaler register      Address offset 0x20 */
}SPI_RegDef_t;

/**
 * @brief Peripheral register definition structure for I2C.
 */
typedef struct
{
    volatile uint32_t CR1;          /**< @brief I2C control register 1          Address offset 0x00 */
    volatile uint32_t CR2;          /**< @brief I2C control register 2          Address offset 0x04 */
    volatile uint32_t OAR1;         /**< @brief I2C own address register 1      Address offset 0x08 */
    volatile uint32_t OAR2;         /**< @brief I2C own address register 2      Address offset 0x0C */
    volatile uint32_t DR;           /**< @brief I2C data register               Address offset 0x10 */
    volatile uint32_t SR1;          /**< @brief I2C status register 1           Address offset 0x14 */
    volatile uint32_t SR2;          /**< @brief I2C status register 2           Address offset 0x18 */
    volatile uint32_t CCR;          /**< @brief I2C clock control register      Address offset 0x1C */
    volatile uint32_t TRISE;        /**< @brief I2C TRISE register              Address offset 0x20 */
    volatile uint32_t FLTR;         /**< @brief I2C FLTR register               Address offset 0x24 */
}I2C_RegDef_t;

/**
 * @brief Peripheral register definition structure for USART.
 */
typedef struct
{
    volatile uint32_t SR;       /**< @brief USART status register                Address offset 0x00 */
    volatile uint32_t DR;       /**< @brief USART data register                  Address offset 0x04 */
    volatile uint32_t BRR;      /**< @brief USART baud rate register             Address offset 0x08 */
    volatile uint32_t CR1;      /**< @brief USART control register 1             Address offset 0x0C */
    volatile uint32_t CR2;      /**< @brief USART control register 2             Address offset 0x10 */
    volatile uint32_t CR3;      /**< @brief USART control register 3             Address offset 0x14 */
    volatile uint32_t GTPR;     /**< @brief USART guard time and prescaler reg   Address offset 0x18 */
}USART_RegDef_t;

/**
 * @brief Peripheral register definition structure for CRC.
 */
typedef struct
{
    volatile uint32_t DR;           /**< @brief CRC data register               Address offset 0x00 */
    volatile uint32_t IDR;          /**< @brief CRC independent data register   Address offset 0x04 */
    volatile uint32_t CR;           /**< @brief CRC control register            Address offset 0x08 */
}CRC_RegDef_t;

/**
 * @brief Peripheral register definition structure for DBG.
 */
typedef struct
{
    volatile uint32_t IDCODE;   /**< @brief Debug MCU ID code register          Address offset 0x00 */
    volatile uint32_t CR;       /**< @brief Debug MCU configuration register    Address offset 0x04 */
    volatile uint32_t APB1_FZ;  /**< @brief Debug MCU APB1 freeze register      Address offset 0x08 */
    volatile uint32_t APB2_FZ;  /**< @brief Debug MCU APB2 freeze register      Address offset 0x0C */
}DBG_RegDef_t;

/**
 * @brief Peripheral register definition structure for FLASH Interface.
 */
typedef struct
{
    volatile uint32_t ACR;          /**< @brief Flash access control register   Address offset 0x00 */
    volatile uint32_t KEYR;         /**< @brief Flash key register              Address offset 0x04 */
    volatile uint32_t OPTKEYR;      /**< @brief Flash option key register       Address offset 0x08 */
    volatile uint32_t SR;           /**< @brief Flash status register           Address offset 0x0C */
    volatile uint32_t CR;           /**< @brief Flash control register          Address offset 0x10 */
    volatile uint32_t OPTCR;        /**< @brief Flash option control register   Address offset 0x14 */
}FLASHINTR_RegDef_t;

/***********************************************************************************************************/
/*                          Bit Position Definition of Peripheral Register                                 */
/***********************************************************************************************************/

/**
 * @name Bit position definitions SPI control register 1.
 * @{
 */
#define SPI_CR1_CPHA        0   /**< @brief Clock phase offset */
#define SPI_CR1_CPOL        1   /**< @brief Clock polarity offset */
#define SPI_CR1_MSTR        2   /**< @brief Master selection offset */
#define SPI_CR1_BR          3   /**< @brief Baud rate control offset */
#define SPI_CR1_SPE         6   /**< @brief SPI enable offset */
#define SPI_CR1_LSB_FIRST   7   /**< @brief Frame format offset */
#define SPI_CR1_SSI         8   /**< @brief Internal slave select offset */
#define SPI_CR1_SSM         9   /**< @brief Software slave management offset */
#define SPI_CR1_RXONLY      10  /**< @brief Receive only mode enable offset */
#define SPI_CR1_DFF         11  /**< @brief Data frame format offset */
#define SPI_CR1_CRC_NEXT    12  /**< @brief CRC transfer next offset */
#define SPI_CR1_CRC_EN      13  /**< @brief Hardware CRC calculation enable offset */
#define SPI_CR1_BIDI_OE     14  /**< @brief Output enable in bidirectional mode offset */
#define SPI_CR1_BIDI_MODE   15  /**< @brief Bidirectional data mode enable offset */
/** @} */

/**
 * @name Bit position definition SPI control register 2.
 * @{
 */
#define SPI_CR2_RXDMAEN     0   /**< @brief Rx buffer DMA enable offset */
#define SPI_CR2_TXDMAEN     1   /**< @brief Tx buffer DMA enable offset */
#define SPI_CR2_SSOE        2   /**< @brief SS output enable offset */
#define SPI_CR2_FRF         4   /**< @brief Frame format offset */
#define SPI_CR2_ERRIE       5   /**< @brief Error interrupt enable offset */
#define SPI_CR2_RXNEIE      6   /**< @brief Rx buffer not empty interrupt enable offset*/
#define SPI_CR2_TXEIE       7   /**< @brief Tx buffer empty interrupt enable offset */
/** @} */

/**
 * @name Bit position definition SPI status register.
 * @{
 */
#define SPI_SR_RXNE         0   /**< @brief Receive buffer not empty */
#define SPI_SR_TXE          1   /**< @brief Transmit buffer empty offset */
#define SPI_SR_CHSIDE       2   /**< @brief Channel side offset */
#define SPI_SR_UDR          3   /**< @brief Underrun flag offset */
#define SPI_SR_CRCERR       4   /**< @brief CRC error flag offset */
#define SPI_SR_MODF         5   /**< @brief Mode fault offset */
#define SPI_SR_OVR          6   /**< @brief Overrun flag offset */
#define SPI_SR_BSY          7   /**< @brief Busy flag offset */
#define SPI_SR_FRE          8   /**< @brief Frame error offset */
/** @} */

/**
 * @name Bit position definition I2C control register 1.
 * @{
 */
#define I2C_CR1_PE          0   /**< @brief Peripheral enable offset */
#define I2C_CR1_SMBUS       1   /**< @brief SMBus mode offset */
#define I2C_CR1_SMBTYPE     3   /**< @brief SMBus type offset */
#define I2C_CR1_ENARP       4   /**< @brief ARP enable offset */
#define I2C_CR1_ENPEC       5   /**< @brief PEC enable offset */
#define I2C_CR1_ENGC        6   /**< @brief General call enable offset */
#define I2C_CR1_NOSTRETCH   7   /**< @brief Clock stretching disable (Slave mode) offset */
#define I2C_CR1_START       8   /**< @brief Start generation offset */
#define I2C_CR1_STOP        9   /**< @brief Stop generation offset */
#define I2C_CR1_ACK         10  /**< @brief Acknowledge enable offset */
#define I2C_CR1_POS         11  /**< @brief Acknowledge/PEC Position (for data reception) offset */
#define I2C_CR1_PEC         12  /**< @brief Packet error checking offset */
#define I2C_CR1_ALERT       13  /**< @brief SMBus alert offset */
#define I2C_CR1_SWRST       15  /**< @brief Software reset offset */
/** @} */

/**
 * @name Bit position definition I2C control register 2.
 * @{
 */
#define I2C_CR2_FREQ        0   /**< @brief Peripheral clock frequency offset */
#define I2C_CR2_ITERREN     8   /**< @brief Error interrupt enable offset */
#define I2C_CR2_ITEVTEN     9   /**< @brief Event interrupt enable offset */
#define I2C_CR2_ITBUFEN     10  /**< @brief Buffer interrupt enable offset */
#define I2C_CR2_DMAEN       11  /**< @brief DMA requests enable offset */
#define I2C_CR2_LAST        12  /**< @brief DMA last transfer offset */
/** @} */

/**
 * @name Bit position definition I2C status register 1.
 * @{
 */
#define I2C_SR1_SB          0  /**< @brief Start bit (Master mode) offset */
#define I2C_SR1_ADDR        1  /**< @brief Address sent (master mode)/matched (slave mode) offset */
#define I2C_SR1_BTF         2  /**< @brief Byte transfer finished offset */
#define I2C_SR1_ADD10       3  /**< @brief 10-bit header sent (Master mode) offset */
#define I2C_SR1_STOPF       4  /**< @brief Stop detection (slave mode) offset */
#define I2C_SR1_RXNE        6  /**< @brief Data register not empty (receivers) offset */
#define I2C_SR1_TXE         7  /**< @brief Data register empty (transmitters) offset */
#define I2C_SR1_BERR        8  /**< @brief Bus error offset */
#define I2C_SR1_ARLO        9  /**< @brief Arbitration lost (master mode) offset */
#define I2C_SR1_AF          10 /**< @brief Acknowledge failure offset */
#define I2C_SR1_OVR         11 /**< @brief Overrun/Underrun offset*/
#define I2C_SR1_PECERR      12 /**< @brief PEC Error in reception offset */
#define I2C_SR1_TIMEOUT     14 /**< @brief Timeout or Tlow error offset */
#define I2C_SR1_SMBALERT    15 /**< @brief SMBus alert offset */
/** @} */

/**
 * @name Bit position definition I2C status register 2.
 * @{
 */
#define I2C_SR2_MSL         0   /**< @brief Master/slave offset */
#define I2C_SR2_BUSY        1   /**< @brief Bus busy offset */
#define I2C_SR2_TRA         2   /**< @brief Transmitter/receiver offset */
#define I2C_SR2_GENCALL     4   /**< @brief General call address (Slave mode) offset */
#define I2C_SR2_SMBDEFAULT  5   /**< @brief SMBus device default address (Slave mode) offset */
#define I2C_SR2_SMBHOST     6   /**< @brief SMBus host header (Slave mode) offset */
#define I2C_SR2_DUALF       7   /**< @brief Dual flag (Slave mode) offset */
#define I2C_SR2_PEC         8   /**< @brief Packet error checking register offset */
/** @} */

/**
 * @name Bit position definition I2C clock control register.
 * @{
 */
#define I2C_CCR_CCR         0  /**< @brief Clock control register in Fm/Sm mode (Master mode) offset */
#define I2C_CCR_DUTY        14 /**< @brief Fm mode duty cycle offset */
#define I2C_CCR_FS          15 /**< @brief I2C master mode selection offset */
/** @} */

/**
 * @name Bit position definition USART control register 1.
 * @{
 */
#define USART_CR1_SBK       0   /**< @brief Send break offset */
#define USART_CR1_RWU       1   /**< @brief Receiver wakeup offset */
#define USART_CR1_RE        2   /**< @brief Receiver enable offset */
#define USART_CR1_TE        3   /**< @brief Transmitter enable offset */
#define USART_CR1_IDLEIE    4   /**< @brief IDLE interrupt enable offset */
#define USART_CR1_RXNEIE    5   /**< @brief RXNE interrupt enable offset */
#define USART_CR1_TCIE      6   /**< @brief Transmission complete interrupt enable offset */
#define USART_CR1_TXEIE     7   /**< @brief TXE interrupt enable offset */
#define USART_CR1_PEIE      8   /**< @brief PE interrupt enable offset */
#define USART_CR1_PS        9   /**< @brief Parity selection offset */
#define USART_CR1_PCE       10  /**< @brief Parity control enable offset */
#define USART_CR1_WAKE      11  /**< @brief Wakeup method offset */
#define USART_CR1_M         12  /**< @brief Word length offset */
#define USART_CR1_UE        13  /**< @brief USART enable offset */
#define USART_CR1_OVER8     15  /**< @brief Oversampling mode offset */
/** @} */

/**
 * @name Bit position definition USART control register 2.
 * @{
 */
#define USART_CR2_ADD       0  /**< @brief Address of the USART node offset */
#define USART_CR2_LBDL      5  /**< @brief LIN break detection length offset */
#define USART_CR2_LBDIE     6  /**< @brief LIN break detection interrupt enable offset */
#define USART_CR2_LBCL      8  /**< @brief Last bit clock pulse offset */
#define USART_CR2_CPHA      9  /**< @brief Clock phase offset */
#define USART_CR2_CPOL      10 /**< @brief Clock polarity offset */
#define USART_CR2_CLKEN     11 /**< @brief Clock enable offset */
#define USART_CR2_STOP      12 /**< @brief STOP bits offset */
#define USART_CR2_LINEN     14 /**< @brief LIN mode enable offset */
/** @} */

/**
 * @name Bit position definition USART control register 3.
 * @{
 */
#define USART_CR3_EIE       0  /**< @brief Error interrupt enable offset */
#define USART_CR3_IREN      1  /**< @brief IrDA mode enable offset */
#define USART_CR3_IRLP      2  /**< @brief IrDA low-power offset */
#define USART_CR3_HDSEL     3  /**< @brief Half-duplex selection offset */
#define USART_CR3_NACK      4  /**< @brief Smartcard NACK enable offset */
#define USART_CR3_SCEN      5  /**< @brief Smartcard mode enable offset */
#define USART_CR3_DMAR      6  /**< @brief DMA enable receiver offset */
#define USART_CR3_DMAT      7  /**< @brief DMA enable transmitter offset */
#define USART_CR3_RTSE      8  /**< @brief RTS enable offset */
#define USART_CR3_CTSE      9  /**< @brief CTS enable offset */
#define USART_CR3_CTSIE     10 /**< @brief CTS interrupt enable offset */
#define USART_CR3_ONEBIT    11 /**< @brief One sample bit method enable offset */
/** @} */

/**
 * @name Bit position definition USART status register.
 * @{
 */
#define USART_SR_PE         0   /**< @brief Parity error offset */
#define USART_SR_FE         1   /**< @brief Framing error offset */
#define USART_SR_NF         2   /**< @brief Noise detected flag offset */
#define USART_SR_ORE        3   /**< @brief Overrun error offset */
#define USART_SR_IDLE       4   /**< @brief IDLE line detected offset */
#define USART_SR_RXNE       5   /**< @brief Read data register not empty offset */
#define USART_SR_TC         6   /**< @brief Transmission complete offset */
#define USART_SR_TXE        7   /**< @brief Transmit data register empty offset */
#define USART_SR_LBD        8   /**< @brief LIN break detection flag offset */
#define USART_SR_CTS        9   /**< @brief CTS flag offset */
/** @} */

/**
 * @name Bit position definition CRC control register.
 * @{
 */
#define CRC_CR_RESET        0   /**< @brief Reset bit offset */
/** @} */

/**
 * @name Bit position definition FLASH access control register.
 * @{
 */
#define FLASH_ACR_LATENCY   0  /**< @brief Latency offset */
#define FLASH_ACR_PRFTEN    8  /**< @brief Prefetch enable offset */
#define FLASH_ACR_ICEN      9  /**< @brief Instruction cache enable offset */
#define FLASH_ACR_DCEN      10 /**< @brief Data cache enable offset */
#define FLASH_ACR_ICRST     11 /**< @brief Instruction cache reset offset */
#define FLASH_ACR_DCRST     12 /**< @brief Data cache reset offset */
/** @} */

/**
 * @name Bit position definition FLASH_KEYR.
 * @{
 */
#define FLASH_KEYR          0   /**< @brief FPEC key offset */
/** @} */

/**
 * @name Bit position definition FLASH_OPTKEYR.
 * @{
 */
#define FLASH_OPTKEYR       0   /**< @brief Option byte key offset */
/** @} */

/**
 * @name Bit position definition FLASH status register.
 * @{
 */
#define FLASH_SR_EOP        0  /**< @brief End of operation offset */
#define FLASH_SR_OPERR      1  /**< @brief Operation error offset */
#define FLASH_SR_WRPERR     4  /**< @brief Write protection error offset */
#define FLASH_SR_PGAERR     5  /**< @brief Programming alignment error offset */
#define FLASH_SR_PGPERR     6  /**< @brief Programming parallelism error offset */
#define FLASH_SR_PGSERR     7  /**< @brief Programming sequence error offset */
#define FLASH_SR_RDERR      8  /**< @brief Read Protection Error (pcrop) offset */
#define FLASH_SR_BSY        16 /**< @brief Busy offset */
/** @} */

/**
 * @name Bit position definition FLASH control register.
 * @{
 */
#define FLASH_CR_PG         0  /**< @brief Programming offset */
#define FLASH_CR_SER        1  /**< @brief Sector erase offset */
#define FLASH_CR_MER        2  /**< @brief Mass erase offset */
#define FLASH_CR_SNB        3  /**< @brief Sector number offset */
#define FLASH_CR_PSIZE      8  /**< @brief Program size offset */
#define FLASH_CR_STRT       16 /**< @brief Start offset */
#define FLASH_CR_EOPIE      24 /**< @brief End of operation interrupt enable offset */
#define FLASH_CR_ERRIE      25 /**< @brief Error interrupt enable offset */
#define FLASH_CR_LOCK       31 /**< @brief Lock offset */
/** @} */

/**
 * @name Bit position definition FLASH option control register.
 * @{
 */
#define FLASH_OPTCR_OPTLOCK     0  /**< @brief Option lock offset */
#define FLASH_OPTCR_OPTSTRT     1  /**< @brief Option start offset */
#define FLASH_OPTCR_BOR_LEV     2  /**< @brief BOR reset level offset */
#define FLASH_OPTCR_WDG_SW      5  /**< @brief User option byte (WDG_SW) offset */
#define FLASH_OPTCR_NRST_STOP   6  /**< @brief User option byte (nRST_STOP) offset */
#define FLASH_OPTCR_NRST_STDBY  7  /**< @brief User option byte (nRST_STDBY) offset */
#define FLASH_OPTCR_RDP         8  /**< @brief Read protect offset */
#define FLASH_OPTCR_NWRP        16 /**< @brief Not write protect offset */
#define FLASH_OPTCR_SPRMOD      31 /**< @brief Selection of Protection Mode of nWRPi bits offset */
/** @} */

/***********************************************************************************************************/
/*          Peripheral definitions (peripheral base addresses typecasted to xxx_RegDef_t)                  */
/***********************************************************************************************************/

/**
 * @name Peripheral Base Addresses Typecasted to xxx_RegDef_t.
 * @{
 */
#define GPIOA       ((GPIO_RegDef_t*)GPIOA_BASEADDR)        /**< @brief GPIOA base addr reg definition */
#define GPIOB       ((GPIO_RegDef_t*)GPIOB_BASEADDR)        /**< @brief GPIOB base addr reg definition */
#define GPIOC       ((GPIO_RegDef_t*)GPIOC_BASEADDR)        /**< @brief GPIOC base addr reg definition */
#define GPIOD       ((GPIO_RegDef_t*)GPIOD_BASEADDR)        /**< @brief GPIOD base addr reg definition */
#define GPIOE       ((GPIO_RegDef_t*)GPIOE_BASEADDR)        /**< @brief GPIOE base addr reg definition */
#define GPIOF       ((GPIO_RegDef_t*)GPIOF_BASEADDR)        /**< @brief GPIOF base addr reg definition */
#define GPIOG       ((GPIO_RegDef_t*)GPIOG_BASEADDR)        /**< @brief GPIOG base addr reg definition */
#define GPIOH       ((GPIO_RegDef_t*)GPIOH_BASEADDR)        /**< @brief GPIOH base addr reg definition */
#define GPIOI       ((GPIO_RegDef_t*)GPIOI_BASEADDR)        /**< @brief GPIOI base addr reg definition */

#define RCC         ((RCC_RegDef_t*)RCC_BASEADDR)           /**< @brief RCC base addr reg definition */

#define EXTI        ((EXTI_RegDef_t*)EXTI_BASEADDR)         /**< @brief EXTI base addr reg definition */

#define SYSCFG      ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)     /**< @brief SYSCFG base addr reg definition */

#define SPI1        ((SPI_RegDef_t*)SPI1_BASEADDR)          /**< @brief SPII base addr reg definition */
#define SPI2        ((SPI_RegDef_t*)SPI2_I2S2_BASEADDR)     /**< @brief SPI2 base addr reg definition */
#define SPI3        ((SPI_RegDef_t*)SPI3_I2S3_BASEADDR)     /**< @brief SPI3 base addr reg definition */
#define SPI4        ((SPI_RegDef_t*)SPI4_BASEADDR)          /**< @brief SPI4 base addr reg definition */

#define I2C1        ((I2C_RegDef_t*)I2C1_BASEADDR)          /**< @brief I2C1 base addr reg definition */
#define I2C2        ((I2C_RegDef_t*)I2C2_BASEADDR)          /**< @brief I2C2 base addr reg definition */
#define I2C3        ((I2C_RegDef_t*)I2C3_BASEADDR)          /**< @brief I2C3 base addr reg definition */

#define USART1      ((USART_RegDef_t*)USART1_BASEADDR)      /**< @brief USART1 base addr reg definition */
#define USART2      ((USART_RegDef_t*)USART2_BASEADDR)      /**< @brief USART2 base addr reg definition */
#define USART3      ((USART_RegDef_t*)USART3_BASEADDR)      /**< @brief USART3 base addr reg definition */
#define UART4       ((USART_RegDef_t*)UART4_BASEADDR)       /**< @brief USART4 base addr reg definition */
#define UART5       ((USART_RegDef_t*)UART5_BASEADDR)       /**< @brief USART5 base addr reg definition */
#define USART6      ((USART_RegDef_t*)USART6_BASEADDR)      /**< @brief USART6 base addr reg definition */

#define CRC         ((CRC_RegDef_t*)CRC_BASEADDR)           /**< @brief CRC base addr reg definition */

#define DBGMCU      ((DBG_RegDef_t*)DBGMCU_BASEADDR)        /**< @brief DBGMCU base addr reg definition */

#define FLASHINTR   ((FLASHINTR_RegDef_t*)FLASHINTR_BASEADDR)   /**< @brief FLASHINTR base addr reg defini */
/** @} */

/***********************************************************************************************************/
/*                          Peripheral macros                                                              */
/***********************************************************************************************************/

/**
 * @name Clock enable macros for GPIOx peripheral.
 * @{
 */
#define GPIOA_PCLK_EN()     (RCC->AHB1ENR |= (1 << 0))  /**< @brief Clock enable for GPIOA */
#define GPIOB_PCLK_EN()     (RCC->AHB1ENR |= (1 << 1))  /**< @brief Clock enable for GPIOB */
#define GPIOC_PCLK_EN()     (RCC->AHB1ENR |= (1 << 2))  /**< @brief Clock enable for GPIOC */
#define GPIOD_PCLK_EN()     (RCC->AHB1ENR |= (1 << 3))  /**< @brief Clock enable for GPIOD */
#define GPIOE_PCLK_EN()     (RCC->AHB1ENR |= (1 << 4))  /**< @brief Clock enable for GPIOE */
#define GPIOF_PCLK_EN()     (RCC->AHB1ENR |= (1 << 5))  /**< @brief Clock enable for GPIOF */
#define GPIOG_PCLK_EN()     (RCC->AHB1ENR |= (1 << 6))  /**< @brief Clock enable for GPIOG */
#define GPIOH_PCLK_EN()     (RCC->AHB1ENR |= (1 << 7))  /**< @brief Clock enable for GPIOH */
/** @} */

/**
 * @name Clock enable macros for I2Cx peripheral.
 * @{
 */
#define I2C1_PCLK_EN()      (RCC->APB1ENR |= (1 << 21)) /**< @brief Clock enable for I2C1 */
#define I2C2_PCLK_EN()      (RCC->APB1ENR |= (1 << 22)) /**< @brief Clock enable for I2C2 */
#define I2C3_PCLK_EN()      (RCC->APB1ENR |= (1 << 23)) /**< @brief Clock enable for I2C3 */
/** @} */

/**
 * @name Clock enable macros for SPIx peripheral.
 * @{
 */
#define SPI1_PCLK_EN()      (RCC->APB2ENR |= (1 << 12)) /**< @brief Clock enable for SPI1 */
#define SPI2_PCLK_EN()      (RCC->APB1ENR |= (1 << 14)) /**< @brief Clock enable for SPI2 */
#define SPI3_PCLK_EN()      (RCC->APB1ENR |= (1 << 15)) /**< @brief Clock enable for SPI3 */
#define SPI4_PCLK_EN()      (RCC->APB2ENR |= (1 << 13)) /**< @brief Clock enable for SPI4 */
/** @} */

/**
 * @name Clock enable macros for USARTx / UARTx peripheral.
 * @{
 */
#define USART1_PCLK_EN()    (RCC->APB2ENR |= (1 << 4))  /**< @brief Clock enable for USART1 */
#define USART2_PCLK_EN()    (RCC->APB1ENR |= (1 << 17)) /**< @brief Clock enable for USART2 */
#define USART3_PCLK_EN()    (RCC->APB1ENR |= (1 << 18)) /**< @brief Clock enable for USART3 */
#define UART4_PCLK_EN()     (RCC->APB1ENR |= (1 << 19)) /**< @brief Clock enable for UART4 */
#define UART5_PCLK_EN()     (RCC->APB1ENR |= (1 << 20)) /**< @brief Clock enable for UART5 */
#define USART6_PCLK_EN()    (RCC->APB2ENR |= (1 << 5))  /**< @brief Clock enable for USART6 */
/** @} */

/**
 * @name Clock enable macros for SYSCFG peripheral.
 * @{
 */
#define SYSCFG_PCLK_EN()    (RCC->APB2ENR |= (1 << 14)) /**< @brief Clock enable for SYSCFG */
/** @} */

/**
 * @name Clock enable macros for CRC peripheral.
 * @{
 */
#define CRC_PCLK_EN()       (RCC->AHB1ENR |= (1 << 12)) /**< @brief Clock enable for CRC */
/** @} */

/**
 * @name Clock disable macros for GPIOx peripheral.
 * @{
 */
#define GPIOA_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 0)) /**< @brief Clock disable for GPIOA */
#define GPIOB_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 1)) /**< @brief Clock disable for GPIOB */
#define GPIOC_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 2)) /**< @brief Clock disable for GPIOC */
#define GPIOD_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 3)) /**< @brief Clock disable for GPIOD */
#define GPIOE_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 4)) /**< @brief Clock disable for GPIOE */
#define GPIOF_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 5)) /**< @brief Clock disable for GPIOF */
#define GPIOG_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 6)) /**< @brief Clock disable for GPIOG */
#define GPIOH_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 7)) /**< @brief Clock disable for GPIOH */
/** @} */

/**
 * @name Clock disable macros for I2Cx peripheral.
 * @{
 */
#define I2C1_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 21)) /**< @brief Clock disable for I2C1 */
#define I2C2_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 22)) /**< @brief Clock disable for I2C2 */
#define I2C3_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 23)) /**< @brief Clock disable for I2C3 */
/** @} */

/**
 * @name Clock disable macros for SPIx peripheral.
 * @{
 */
#define SPI1_PCLK_DI()      (RCC->APB2ENR &= ~(1 << 12)) /**< @brief Clock disable for SPI1 */
#define SPI2_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 14)) /**< @brief Clock disable for SPI2 */
#define SPI3_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 15)) /**< @brief Clock disable for SPI3 */
#define SPI4_PCLK_DI()      (RCC->APB2ENR &= ~(1 << 13)) /**< @brief Clock disable for SPI4 */
/** @} */

/**
 * @name Clock disable macros for USARTx / UARTx peripheral.
 * @{
 */
#define USART1_PCLK_DI()    (RCC->APB2ENR &= ~(1 << 4))     /**< @brief Clock disable for USART1 */
#define USART2_PCLK_DI()    (RCC->APB1ENR &= ~(1 << 17))    /**< @brief Clock disable for USART2 */
#define USART3_PCLK_DI()    (RCC->APB1ENR &= ~(1 << 18))    /**< @brief Clock disable for USART3 */
#define UART4_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 19))    /**< @brief Clock disable for UART4 */
#define UART5_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 20))    /**< @brief Clock disable for UART5 */
#define USART6_PCLK_DI()    (RCC->APB2ENR &= ~(1 << 5))     /**< @brief Clock disable for USART6 */
/** @} */

/**
 * @name Clock disable macros for SYSCFG peripheral.
 * @{
 */
#define SYSCFG_PCLK_DI()    (RCC->APB2ENR &= ~(1 << 14))    /**< @brief Clock disable for SYSCFG */
/** @} */

/**
 * @name Clock disable macros for CRC peripheral.
 * @{
 */
#define CRC_PCLK_DI()       (RCC->AHB1ENR &= ~(1 << 12))    /**< @brief Clock disable for CRC */
/** @} */

/**
 * @name Reset macros GPIOx peripheral.
 * @{
 */
/** @brief Reset macro for GPIOA */
#define GPIOA_REG_RESET()   do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
/** @brief Reset macro for GPIOB */
#define GPIOB_REG_RESET()   do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
/** @brief Reset macro for GPIOC */
#define GPIOC_REG_RESET()   do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
/** @brief Reset macro for GPIOD */
#define GPIOD_REG_RESET()   do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
/** @brief Reset macro for GPIOE */
#define GPIOE_REG_RESET()   do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
/** @brief Reset macro for GPIOF */
#define GPIOF_REG_RESET()   do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
/** @brief Reset macro for GPIOG */
#define GPIOG_REG_RESET()   do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
/** @brief Reset macro for GPIOH */
#define GPIOH_REG_RESET()   do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)
/** @} */

/**
 * @brief This macro returns a code between 0 to 7 for a given GPIO base address(x).
 */
#define GPIO_BASEADDR_TO_CODE(x)    ((x == GPIOA) ? 0 :\
                                    (x == GPIOB) ? 1 :\
                                    (x == GPIOC) ? 2 :\
                                    (x == GPIOD) ? 3 :\
                                    (x == GPIOE) ? 4 :\
                                    (x == GPIOF) ? 5 :\
                                    (x == GPIOG) ? 6 :\
                                    (x == GPIOH) ? 7 : 0)

/**
 * @name Reset macros SPIx peripheral.
 * @{
 */
/** @brief Reset macro for SPI1 */
#define SPI1_REG_RESET()    do{(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12));}while(0)
/** @brief Reset macro for SPI2 */
#define SPI2_REG_RESET()    do{(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14));}while(0)
/** @brief Reset macro for SPI3 */
#define SPI3_REG_RESET()    do{(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15));}while(0)
/** @brief Reset macro for SPI4 */
#define SPI4_REG_RESET()    do{(RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13));}while(0)
/** @} */

/**
 * @name Reset macros I2Cx peripheral.
 * @{
 */
/** @brief Reset macro for I2C1 */
#define I2C1_REG_RESET()    do{(RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21));}while(0)
/** @brief Reset macro for I2C2 */
#define I2C2_REG_RESET()    do{(RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22));}while(0)
/** @brief Reset macro for I2C3 */
#define I2C3_REG_RESET()    do{(RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23));}while(0)
/** @} */

/**
 * @name Reset macros USARTx peripheral.
 * @{
 */
/** @brief Reset macro for USART1 */
#define USART1_REG_RESET()  do{(RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4));}while(0)
/** @brief Reset macro for USART2 */
#define USART2_REG_RESET()  do{(RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17));}while(0)
/** @brief Reset macro for USART3 */
#define USART3_REG_RESET()  do{(RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 18));}while(0)
/** @brief Reset macro for UART4 */
#define UART4_REG_RESET()   do{(RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 19));}while(0)
/** @brief Reset macro for UART5 */
#define UART5_REG_RESET()   do{(RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 20));}while(0)
/** @brief Reset macro for USART6 */
#define USART6_REG_RESET()  do{(RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5));}while(0)
/** @} */

/**
 * @name Reset macros CRC peripheral.
 * @{
 */
/** @brief Reset macro for CRC */
#define CRC_REG_RESET()     do{(RCC->AHB1RSTR |= (1 << 12)); (RCC->AHB1RSTR &= ~(1 << 12));}while(0)
/** @} */

/***********************************************************************************************************/
/*                          IRQ definitions                                                                */
/***********************************************************************************************************/

/**
 * @name IRQ (Interrupt Request) number.
 * @{
 */
#define IRQ_NO_EXTI0        6   /**< @brief Interrupt Number for EXTI0 */
#define IRQ_NO_EXTI1        7   /**< @brief Interrupt Number for EXTI1 */
#define IRQ_NO_EXTI2        8   /**< @brief Interrupt Number for EXTI2 */
#define IRQ_NO_EXTI3        9   /**< @brief Interrupt Number for EXTI3 */
#define IRQ_NO_EXTI4        10  /**< @brief Interrupt Number for EXTI4 */
#define IRQ_NO_EXTI9_5      23  /**< @brief Interrupt Number for EXTI5 to EXTI9 */
#define IRQ_NO_EXTI15_10    40  /**< @brief Interrupt Number for EXTI10 to EXTI15 */
#define IRQ_NO_SPI1         35  /**< @brief Interrupt Number for SPI1 */
#define IRQ_NO_SPI2         36  /**< @brief Interrupt Number for SPI2 */
#define IRQ_NO_SPI3         51  /**< @brief Interrupt Number for SPI3 */
#define IRQ_NO_SPI4         84  /**< @brief Interrupt Number for SPI4 */
#define IRQ_NO_I2C1_EV      31  /**< @brief Interrupt Number for I2C1 EV */
#define IRQ_NO_I2C1_ER      32  /**< @brief Interrupt Number for I2C1 ER */
#define IRQ_NO_I2C2_EV      33  /**< @brief Interrupt Number for I2C2 EV */
#define IRQ_NO_I2C2_ER      34  /**< @brief Interrupt Number for I2C2 ER */
#define IRQ_NO_I2C3_EV      72  /**< @brief Interrupt Number for I2C3 EV */
#define IRQ_NO_I2C3_ER      73  /**< @brief Interrupt Number for I2C3 ER */
#define IRQ_NO_USART1       37  /**< @brief Interrupt Number for USART1 */
#define IRQ_NO_USART2       38  /**< @brief Interrupt Number for USART2 */
#define IRQ_NO_USART3       39  /**< @brief Interrupt Number for USART3 */
#define IRQ_NO_UART4        52  /**< @brief Interrupt Number for UART4 */
#define IRQ_NO_UART5        53  /**< @brief Interrupt Number for UART5 */
#define IRQ_NO_USART6       71  /**< @brief Interrupt Number for USART6 */
/** @} */

/**
 * @name IRQ priority.
 * @{
 */
#define NVIC_IRQ_PRIORITY0      0   /**< @brief Interrupt Priority 0 */
#define NVIC_IRQ_PRIORITY1      1   /**< @brief Interrupt Priority 1 */
#define NVIC_IRQ_PRIORITY2      2   /**< @brief Interrupt Priority 2 */
#define NVIC_IRQ_PRIORITY3      3   /**< @brief Interrupt Priority 3 */
#define NVIC_IRQ_PRIORITY4      4   /**< @brief Interrupt Priority 4 */
#define NVIC_IRQ_PRIORITY5      5   /**< @brief Interrupt Priority 5 */
#define NVIC_IRQ_PRIORITY6      6   /**< @brief Interrupt Priority 6 */
#define NVIC_IRQ_PRIORITY7      7   /**< @brief Interrupt Priority 7 */
#define NVIC_IRQ_PRIORITY8      8   /**< @brief Interrupt Priority 8 */
#define NVIC_IRQ_PRIORITY9      9   /**< @brief Interrupt Priority 9 */
#define NVIC_IRQ_PRIORITY10     10  /**< @brief Interrupt Priority 10 */
#define NVIC_IRQ_PRIORITY11     11  /**< @brief Interrupt Priority 11 */
#define NVIC_IRQ_PRIORITY12     12  /**< @brief Interrupt Priority 12 */
#define NVIC_IRQ_PRIORITY13     13  /**< @brief Interrupt Priority 13 */
#define NVIC_IRQ_PRIORITY14     14  /**< @brief Interrupt Priority 14 */
#define NVIC_IRQ_PRIORITY15     15  /**< @brief Interrupt Priority 15 */
/** @} */

#endif /* STM32F446XX_H */
