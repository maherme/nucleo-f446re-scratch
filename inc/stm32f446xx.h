/*****************************************************************************************************
* FILENAME :        stm32f446xx.h
*
* DESCRIPTION :
*       File containing macros definition and structures for configuring the microcontroller.
*
* NOTES :
*       These file is for the specific STM32F446xx microcontroller.
*
**/

#ifndef STM32F446XX_H
#define STM32F446XX_H

#include <stdint.h>

/*****************************************************************************************************/
/*                          Generic macros                                                           */
/*****************************************************************************************************/
#define ENABLE              1
#define DISABLE             0
#define SET                 ENABLE
#define RESET               DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_SET            SET
#define FLAG_RESET          RESET

/*****************************************************************************************************/
/*                          ARM Cortex M4 Processor Specific Setails                                 */
/*****************************************************************************************************/

/**
 * ARM Cortex M4 processor NVIC ISERx register addresses.
 */
#define NVIC_ISER0      ((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1      ((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2      ((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3      ((volatile uint32_t*)0xE000E10C)

/**
 * ARM Cortex M4 processor NVIC ICERx register addresses.
 */
#define NVIC_ICER0      ((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1      ((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2      ((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3      ((volatile uint32_t*)0xE000E18C)

/**
 * ARM Cortex M4 processor priority register address calculation
 */
#define NVIC_PR_BASEADDR    ((volatile uint32_t*)0xE000E400)

/**
 * ARM Cortex M4 processor number of priority bits implemented in priority register.
 */
#define NO_PR_BITS_IMPLEMENTED      4

/*****************************************************************************************************/
/*                          Memory and Bus Base Address Definition                                   */
/*****************************************************************************************************/

/**
 * Base addresses of memories.
 */
#define FLASH_BASEADDR      0x08000000U         /* Base address of flash memory, size 512KB */
#define SRAM1_BASEADDR      0x20000000U         /* Base address of system RAM first block, size 116KB */
#define SRAM2_BASEADDR      0x2001C000U         /* Base address of system RAM second block, size 16KB */
#define SRAM                SRAM1_BASEADDR
#define ROM                 0x1FFF0000U         /* Base address of system memory or ROM, size 30KB */

/**
 * Base addresses of AHBx and APBx Bus Peripheral.
 */
#define PERIPH_BASEADDR     0x40000000U         /* Base address of peripheral */
#define APB1_BASEADDR       PERIPH_BASEADDR     /* Base address of peripheral connected to APB1 bus */
#define APB2_BASEADDR       0x40010000U         /* Base address of peripheral connected to APB2 bus */
#define AHB1_BASEADDR       0x40020000U         /* Base address of peripheral connected to AHB1 bus */
#define AHB2_BASEADDR       0x50000000U         /* Base address of peripheral connected to AHB2 bus */
#define AHB3_BASEADDR       0xA0000000U         /* Base address of peripheral connected to AHB3 bus */

/*****************************************************************************************************/
/*                          Peripheral Base Address Definition                                       */
/*****************************************************************************************************/

/**
 * Base addresses of peripheral connected to AHB1 bus.
*/
#define GPIOA_BASEADDR      (AHB1_BASEADDR + 0x0000)    /* Base address of GPIOA */
#define GPIOB_BASEADDR      (AHB1_BASEADDR + 0x0400)    /* Base address of GPIOB */
#define GPIOC_BASEADDR      (AHB1_BASEADDR + 0x0800)    /* Base address of GPIOC */
#define GPIOD_BASEADDR      (AHB1_BASEADDR + 0x0C00)    /* Base address of GPIOD */
#define GPIOE_BASEADDR      (AHB1_BASEADDR + 0x1000)    /* Base address of GPIOE */
#define GPIOF_BASEADDR      (AHB1_BASEADDR + 0x1400)    /* Base address of GPIOF */
#define GPIOG_BASEADDR      (AHB1_BASEADDR + 0x1800)    /* Base address of GPIOG */
#define GPIOH_BASEADDR      (AHB1_BASEADDR + 0x1C00)    /* Base address of GPIOH */
#define CRC_BASEADDR        (AHB1_BASEADDR + 0x3000)    /* Base address of CRC */
#define RCC_BASEADDR        (AHB1_BASEADDR + 0x3800)    /* Base address of RCC */
#define FLASHINTR_BASEADDR  (AHB1_BASEADDR + 0x3C00)    /* Base address of flash interface register */
#define BKPSRAM_BASEADDR    (AHB1_BASEADDR + 0x4000)    /* Base address of BKPSRAM */
#define DMA1_BASEADDR       (AHB1_BASEADDR + 0x6000)    /* Base address of DMA1 */
#define DMA2_BASEADDR       (AHB1_BASEADDR + 0x6400)    /* Base address of DMA2 */
#define USBOTGHS_BASEADDR   (AHB1_BASEADDR + 0x40000)   /* Base address of USB OTG HS */

/**
 * Base addresses of peripheral connected to APB1 bus.
 */
#define TIM2_BASEADDR       (APB1_BASEADDR + 0x0000)    /* Base address of TIM2 */
#define TIM3_BASEADDR       (APB1_BASEADDR + 0x0400)    /* Base address of TIM3 */
#define TIM4_BASEADDR       (APB1_BASEADDR + 0x0800)    /* Base address of TIM4 */
#define TIM5_BASEADDR       (APB1_BASEADDR + 0x0C00)    /* Base address of TIM5 */
#define TIM6_BASEADDR       (APB1_BASEADDR + 0x1000)    /* Base address of TIM6 */
#define TIM7_BASEADDR       (APB1_BASEADDR + 0x1400)    /* Base address of TIM7 */
#define TIM12_BASEADDR      (APB1_BASEADDR + 0x1800)    /* Base address of TIM12 */
#define TIM13_BASEADDR      (APB1_BASEADDR + 0x1C00)    /* Base address of TIM13 */
#define TIM14_BASEADDR      (APB1_BASEADDR + 0x2000)    /* Base address of TIM14 */
#define RTCBKP_BASEADDR     (APB1_BASEADDR + 0x2800)    /* Base address of RTC and BKP registers */
#define WWDG_BASEADDR       (APB1_BASEADDR + 0x2C00)    /* Base address of WWDG */
#define IWDG_BASEADDR       (APB1_BASEADDR + 0x3000)    /* Base address of IWDG */
#define SPI2_I2S2_BASEADDR  (APB1_BASEADDR + 0x3800)    /* Base address of SPI2 / I2S2 */
#define SPI3_I2S3_BASEADDR  (APB1_BASEADDR + 0x3C00)    /* Base address of SPI3 / I2S3 */
#define SPDIF_RX_BASEADDR   (APB1_BASEADDR + 0x4000)    /* Base address of SPDIF-RX */
#define USART2_BASEADDR     (APB1_BASEADDR + 0x4400)    /* Base address of USART2 */
#define USART3_BASEADDR     (APB1_BASEADDR + 0x4800)    /* Base address of USART3 */
#define UART4_BASEADDR      (APB1_BASEADDR + 0x4C00)    /* Base address of UART4 */
#define UART5_BASEADDR      (APB1_BASEADDR + 0x5000)    /* Base address of UART5 */
#define I2C1_BASEADDR       (APB1_BASEADDR + 0x5400)    /* Base address of I2C1 */
#define I2C2_BASEADDR       (APB1_BASEADDR + 0x5800)    /* Base address of I2C2 */
#define I2C3_BASEADDR       (APB1_BASEADDR + 0x5C00)    /* Base address of I2C3 */
#define CAN1_BASEADDR       (APB1_BASEADDR + 0x6400)    /* Base address of CAN1 */
#define CAN2_BASEADDR       (APB1_BASEADDR + 0x6800)    /* Base address of CAN2 */
#define HDMI_CEC_BASEADDR   (APB1_BASEADDR + 0x6C00)    /* Base address of HDMI-CEC */
#define PWR_BASEADDR        (APB1_BASEADDR + 0x7000)    /* Base address of PWR */
#define DAC_BASEADDR        (APB1_BASEADDR + 0x7400)    /* Base address of DAC */

/**
 * Base addresses of peripheral connected to AHB2 bus.
 */
#define USBOTGFS_BASEADDR   (AHB2_BASEADDR + 0x0000)    /* Base address of USB OTG FS */
#define DCMI_BASEADDR       (AHB2_BASEADDR + 0x50000)   /* Base address of DCMI */

/**
 * Base addresses of peripheral connected to APB2 bus.
 */
#define TIM1_BASEADDR       (APB2_BASEADDR + 0x0000)    /* Base address of TIM1 */
#define TIM8_BASEADDR       (APB2_BASEADDR + 0x0400)    /* Base address of TIM8 */
#define USART1_BASEADDR     (APB2_BASEADDR + 0x1000)    /* Base address of USART1 */
#define USART6_BASEADDR     (APB2_BASEADDR + 0x1400)    /* Base address of USART6 */
#define ADC1_2_3_BASEADDR   (APB2_BASEADDR + 0x2000)    /* Base address of ADC1 - ADC2 - ADC3 */
#define SDMMC_BASEADDR      (APB2_BASEADDR + 0x2C00)    /* Base address of SDMMC */
#define SPI1_BASEADDR       (APB2_BASEADDR + 0x3000)    /* Base address of SPI1 */
#define SPI4_BASEADDR       (APB2_BASEADDR + 0x3400)    /* Base address of SPI4 */
#define SYSCFG_BASEADDR     (APB2_BASEADDR + 0x3800)    /* Base address of SYSCFG */
#define EXTI_BASEADDR       (APB2_BASEADDR + 0x3C00)    /* Base address of EXTI */
#define TIM9_BASEADDR       (APB2_BASEADDR + 0x4000)    /* Base address of TIM9 */
#define TIM10_BASEADDR      (APB2_BASEADDR + 0x4400)    /* Base address of TIM10 */
#define TIM11_BASEADDR      (APB2_BASEADDR + 0x4800)    /* Base address of TIM11 */
#define SAI1_BASEADDR       (APB2_BASEADDR + 0x5800)    /* Base address of SAI1 */
#define SAI2_BASEADDR       (APB2_BASEADDR + 0x5C00)    /* Base address of SAI2 */

/**
 * Base addresses of peripheral connected to AHB3 bus.
 */
#define FMC_BASEADDR        (AHB3_BASEADDR + 0x0000)    /* Base address of FMC control register */
#define QUADSPI_BASEADDR    (AHB3_BASEADDR + 0x1000)    /* Base address of QUADSPI register */

/*****************************************************************************************************/
/*                          Peripheral Register Definition Structures                                */
/*****************************************************************************************************/

/**
 * Peripheral register definition structure for GPIO.
 */
typedef struct
{
    volatile uint32_t MODER;        /* GPIO port mode register                      Address offset 0x00 */
    volatile uint32_t OTYPER;       /* GPIO port output type register               Address offset 0x04 */
    volatile uint32_t OSPEEDER;     /* GPIO port output speed register              Address offset 0x08 */
    volatile uint32_t PUPDR;        /* GPIO port pull-up/pull-down register         Address offset 0x0C */
    volatile uint32_t IDR;          /* GPIO port input data register                Address offset 0x10 */
    volatile uint32_t ODR;          /* GPIO port output data register               Address offset 0x14 */
    volatile uint32_t BSRR;         /* GPIO port bit set/reset register             Address offset 0x18 */
    volatile uint32_t LCKR;         /* GPIO port configuration lock register        Address offset 0x1C */
    volatile uint32_t AFR[2];       /* GPIO alternate function low register AFR[0]  Address offset 0x20 */
                                    /* GPIO alternate function high register AFR[1] Address offset 0x24 */
} GPIO_RegDef_t;

/**
 * Peripheral register definition structure for RCC.
 */
typedef struct
{
    volatile uint32_t CR;           /* RCC clock control register                   Address offset 0x00 */
    volatile uint32_t PLLCFGR;      /* RCC PLL configuration register               Address offset 0x04 */
    volatile uint32_t CFGR;         /* RCC clock configuration register             Address offset 0x08 */
    volatile uint32_t CIR;          /* RCC clock interrupt register                 Address offset 0x0C */
    volatile uint32_t AHB1RSTR;     /* RCC AHB1 peripheral reset register           Address offset 0x10 */
    volatile uint32_t AHB2RSTR;     /* RCC AHB2 peripheral reset register           Address offset 0x14 */
    volatile uint32_t AHB3RSTR;     /* RCC AHB3 peripheral reset register           Address offset 0x18 */
    uint32_t RESERVED0;             /* Reserved                                     Address offset 0x1C */
    volatile uint32_t APB1RSTR;     /* RCC APB1 peripheral reset register           Address offset 0x20 */
    volatile uint32_t APB2RSTR;     /* RCC APB2 peripheral reset register           Address offset 0x24 */
    uint32_t RESERVED1;             /* Reserved                                     Address offset 0x28 */
    uint32_t RESERVED2;             /* Reserved                                     Address offset 0x2C */
    volatile uint32_t AHB1ENR;      /* RCC AHB1 peripheral clock enable register    Address offset 0x30 */
    volatile uint32_t AHB2ENR;      /* RCC AHB2 peripheral clock enable register    Address offset 0x34 */
    volatile uint32_t AHB3ENR;      /* RCC AHB3 peripheral clock enable register    Address offset 0x38 */
    uint32_t RESERVED3;             /* Reserved                                     Address offset 0x3C */
    volatile uint32_t APB1ENR;      /* RCC APB1 peripheral clock enable register    Address offset 0x40 */
    volatile uint32_t APB2ENR;      /* RCC APB2 peripheral clock enable register    Address offset 0x44 */
    uint32_t RESERVED4;             /* Reserved                                     Address offset 0x48 */
    uint32_t RESERVED5;             /* Reserved                                     Address offset 0x4C */
    volatile uint32_t AHB1LPENR;    /* RCC AHB1 peri clk en in low power mode reg   Address offset 0x50 */
    volatile uint32_t AHB2LPENR;    /* RCC AHB2 peri clk en in low power mode reg   Address offset 0x54 */
    volatile uint32_t AHB3LPENR;    /* RCC AHB3 peri clk en in low power mode reg   Address offset 0x58 */
    uint32_t RESERVED6;             /* Reserved                                     Address offset 0x5C */
    volatile uint32_t APB1LPENR;    /* RCC APB1 peri clk en in low power mode reg   Address offset 0x60 */
    volatile uint32_t APB2LPENR;    /* RCC APB2 peri clk en in low power mode reg   Address offset 0x64 */
    uint32_t RESERVED7;             /* Reserved                                     Address offset 0x68 */
    uint32_t RESERVED8;             /* Reserved                                     Address offset 0x6C */
    volatile uint32_t BDCR;         /* RCC Backup domain control register           Address offset 0x70 */
    volatile uint32_t CSR;          /* RCC clock control & status register          Address offset 0x74 */
    uint32_t RESERVED9;             /* Reserved                                     Address offset 0x78 */
    uint32_t RESERVED10;            /* Reserved                                     Address offset 0x7C */
    volatile uint32_t SSCGR;        /* RCC spread spectrum clk generation reg       Address offset 0x80 */
    volatile uint32_t PLLI2SCFGR;   /* RCC PLLI2S configuration register            Address offset 0x84 */
    volatile uint32_t PLLSAICFGR;   /* RCC PLL configuration register               Address offset 0x88 */
    volatile uint32_t DCKCFGR;      /* RCC dedicated clock configuration register   Address offset 0x8C */
    volatile uint32_t CKGATENR;     /* RCC clocks gated enable register             Address offset 0x90 */
    volatile uint32_t DCKCFGR2;     /* RCC dedicated clocks configuration reg 2     Address offset 0x94 */
} RCC_RegDef_t;

/**
 * Peripheral register definition structure for EXTI.
 */
typedef struct
{
    volatile uint32_t IMR;      /* Interrupt mask register              Address offset 0x00 */
    volatile uint32_t EMR;      /* Event mask register                  Address offset 0x04 */
    volatile uint32_t RTSR;     /* Rising trigger selection register    Address offset 0x08 */
    volatile uint32_t FTSR;     /* Falling trigger selection register   Address offset 0x0C */
    volatile uint32_t SWIER;    /* Software interrupt event register    Address offset 0x10 */
    volatile uint32_t PR;       /* Pending register                     Address offset 0x14 */
}EXTI_RegDef_t;

/**
 * Peripheral register definition structure for SYSCFG.
 */
typedef struct
{
    volatile uint32_t MEMRMP;       /* SYSCFG memory remap register                 Address offset 0x00 */
    volatile uint32_t PMC;          /* SYSCFG peripheral mode config register       Address offset 0x04 */
    volatile uint32_t EXTICR[4];    /* SYSCFG ext interrupt cfg reg 1 EXTICR[0]     Address offset 0x08 */
                                    /* SYSCFG ext interrupt cfg reg 2 EXTICR[1]     Address offset 0x0C */
                                    /* SYSCFG ext interrupt cfg reg 3 EXTICR[2]     Address offset 0x10 */
                                    /* SYSCFG ext interrupt cfg reg 4 EXTICR[3]     Address offset 0x14 */
    uint32_t RESERVED0;             /* Reserved                                     Address offset 0x18 */
    uint32_t RESERVED1;             /* Reserved                                     Address offset 0x1C */
    volatile uint32_t CMPCR;        /* Compensation cell control register           Address offset 0x20 */
    uint32_t RESERVED2;             /* Reserved                                     Address offset 0x24 */
    uint32_t RESERVED3;             /* Reserved                                     Address offset 0x28 */
    volatile uint32_t CFGR;         /* SYSCFG configuration register                Address offset 0x2C */
}SYSCFG_RegDef_t;

/**
 * Peripheral register definition structure for SPI.
 */
typedef struct
{
    volatile uint32_t CR1;          /* SPI control register 1               Address offset 0x00 */
    volatile uint32_t CR2;          /* SPI control register 2               Address offset 0x04 */
    volatile uint32_t SR;           /* SPI status register                  Address offset 0x08 */
    volatile uint32_t DR;           /* SPI data register                    Address offset 0x0C */
    volatile uint32_t CRCPR;        /* SPI CRC polynomial register          Address offset 0x10 */
    volatile uint32_t RXCRCR;       /* SPI RX CRC register                  Address offset 0x14 */
    volatile uint32_t TXCRCR;       /* SPI TX CRC register                  Address offset 0x18 */
    volatile uint32_t I2SCFGR;      /* SPI_I2S configuration register       Address offset 0x1C */
    volatile uint32_t I2SPR;        /* SPI_I2S prescaler register           Address offset 0x20 */
}SPI_RegDef_t;

/**
 * Peripheral register definition structure for I2C.
 */
typedef struct
{
    volatile uint32_t CR1;          /* I2C control register 1               Address offset 0x00 */
    volatile uint32_t CR2;          /* I2C control register 2               Address offset 0x04 */
    volatile uint32_t OAR1;         /* I2C own address register 1           Address offset 0x08 */
    volatile uint32_t OAR2;         /* I2C own address register 2           Address offset 0x0C */
    volatile uint32_t DR;           /* I2C data register                    Address offset 0x10 */
    volatile uint32_t SR1;          /* I2C status register 1                Address offset 0x14 */
    volatile uint32_t SR2;          /* I2C status register 2                Address offset 0x18 */
    volatile uint32_t CCR;          /* I2C clock control register           Address offset 0x1C */
    volatile uint32_t TRISE;        /* I2C TRISE register                   Address offset 0x20 */
    volatile uint32_t FLTR;         /* I2C FLTR register                    Address offset 0x24 */
}I2C_RegDef_t;

/**
 * Peripheral register definition structure for USART.
 */
typedef struct
{
    volatile uint32_t SR;           /* USART status register                Address offset 0x00 */
    volatile uint32_t DR;           /* USART data register                  Address offset 0x04 */
    volatile uint32_t BRR;          /* USART baud rate register             Address offset 0x08 */
    volatile uint32_t CR1;          /* USART control register 1             Address offset 0x0C */
    volatile uint32_t CR2;          /* USART control register 2             Address offset 0x10 */
    volatile uint32_t CR3;          /* USART control register 3             Address offset 0x14 */
    volatile uint32_t GTPR;         /* USART guard time and prescaler reg   Address offset 0x18 */
}USART_RegDef_t;

/*****************************************************************************************************/
/*                          Bit Position Definition of Peripheral Register                           */
/*****************************************************************************************************/

/**
 * Bit position definitions SPI_CR1.
 */
#define SPI_CR1_CPHA        0
#define SPI_CR1_CPOL        1
#define SPI_CR1_MSTR        2
#define SPI_CR1_BR          3
#define SPI_CR1_SPE         6
#define SPI_CR1_LSB_FIRST   7
#define SPI_CR1_SSI         8
#define SPI_CR1_SSM         9
#define SPI_CR1_RXONLY      10
#define SPI_CR1_DFF         11
#define SPI_CR1_CRC_NEXT    12
#define SPI_CR1_CRC_EN      13
#define SPI_CR1_BIDI_OE     14
#define SPI_CR1_BIDI_MODE   15

/**
 * Bit position definition SPI_CR2.
 */
#define SPI_CR2_RXDMAEN     0
#define SPI_CR2_TXDMAEN     1
#define SPI_CR2_SSOE        2
#define SPI_CR2_FRF         4
#define SPI_CR2_ERRIE       5
#define SPI_CR2_RXNEIE      6
#define SPI_CR2_TXEIE       7

/**
 * Bit position definition SPI_SR.
 */
#define SPI_SR_RXNE         0
#define SPI_SR_TXE          1
#define SPI_SR_CHSIDE       2
#define SPI_SR_UDR          3
#define SPI_SR_CRCERR       4
#define SPI_SR_MODF         5
#define SPI_SR_OVR          6
#define SPI_SR_BSY          7
#define SPI_SR_FRE          8

/**
 * Bit position definition I2C_CR1.
 */
#define I2C_CR1_PE          0
#define I2C_CR1_SMBUS       1
#define I2C_CR1_SMBTYPE     3
#define I2C_CR1_ENARP       4
#define I2C_CR1_ENPEC       5
#define I2C_CR1_ENGC        6
#define I2C_CR1_NOSTRETCH   7
#define I2C_CR1_START       8
#define I2C_CR1_STOP        9
#define I2C_CR1_ACK         10
#define I2C_CR1_POS         11
#define I2C_CR1_PEC         12
#define I2C_CR1_ALERT       13
#define I2C_CR1_SWRST       15

/**
 * Bit position definition I2C_CR2.
 */
#define I2C_CR2_FREQ        0
#define I2C_CR2_ITERREN     8
#define I2C_CR2_ITEVTEN     9
#define I2C_CR2_ITBUFEN     10
#define I2C_CR2_DMAEN       11
#define I2C_CR2_LAST        12

/**
 * Bit position definition I2C_SR1.
 */
#define I2C_SR1_SB          0
#define I2C_SR1_ADDR        1
#define I2C_SR1_BTF         2
#define I2C_SR1_ADD10       3
#define I2C_SR1_STOPF       4
#define I2C_SR1_RXNE        6
#define I2C_SR1_TXE         7
#define I2C_SR1_BERR        8
#define I2C_SR1_ARLO        9
#define I2C_SR1_AF          10
#define I2C_SR1_OVR         11
#define I2C_SR1_PECERR      12
#define I2C_SR1_TIMEOUT     14
#define I2C_SR1_SMBALERT    15

/**
 * Bit position definition I2C_SR2.
 */
#define I2C_SR2_MSL         0
#define I2C_SR2_BUSY        1
#define I2C_SR2_TRA         2
#define I2C_SR2_GENCALL     4
#define I2C_SR2_SMBDEFAULT  5
#define I2C_SR2_SMBHOST     6
#define I2C_SR2_DUALF       7
#define I2C_SR2_PEC         8

/**
 * Bit position definition I2C_CCR.
 */
#define I2C_CCR_CCR         0
#define I2C_CCR_DUTY        14
#define I2C_CCR_FS          15

/**
 * Bit position definition USART_CR1.
 */
#define USART_CR1_SBK       0
#define USART_CR1_RWU       1
#define USART_CR1_RE        2
#define USART_CR1_TE        3
#define USART_CR1_IDLEIE    4
#define USART_CR1_RXNEIE    5
#define USART_CR1_TCIE      6
#define USART_CR1_TXEIE     7
#define USART_CR1_PEIE      8
#define USART_CR1_PS        9
#define USART_CR1_PCE       10
#define USART_CR1_WAKE      11
#define USART_CR1_M         12
#define USART_CR1_UE        13
#define USART_CR1_OVER8     15

/**
 * Bit position definition USART_CR2.
 */
#define USART_CR2_ADD       0
#define USART_CR2_LBDL      5
#define USART_CR2_LBDIE     6
#define USART_CR2_LBCL      8
#define USART_CR2_CPHA      9
#define USART_CR2_CPOL      10
#define USART_CR2_CLKEN     11
#define USART_CR2_STOP      12
#define USART_CR2_LINEN     14

/**
 * Bit position definition USART_CR3.
 */
#define USART_CR3_EIE       0
#define USART_CR3_IREN      1
#define USART_CR3_IRLP      2
#define USART_CR3_HDSEL     3
#define USART_CR3_NACK      4
#define USART_CR3_SCEN      5
#define USART_CR3_DMAR      6
#define USART_CR3_DMAT      7
#define USART_CR3_RTSE      8
#define USART_CR3_CTSE      9
#define USART_CR3_CTSIE     10
#define USART_CR3_ONEBIT    11

/**
 * Bit position definition USART_SR.
 */
#define USART_SR_PE         0
#define USART_SR_FE         1
#define USART_SR_NF         2
#define USART_SR_ORE        3
#define USART_SR_IDLE       4
#define USART_SR_RXNE       5
#define USART_SR_TC         6
#define USART_SR_TXE        7
#define USART_SR_LBD        8
#define USART_SR_CTS        9

/*****************************************************************************************************/
/*          Peripheral definitions (peripheral base addresses typecasted to xxx_RegDef_t)            */
/*****************************************************************************************************/

#define GPIOA   ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB   ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC   ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD   ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE   ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF   ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG   ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH   ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI   ((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC     ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI    ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG  ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1    ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2    ((SPI_RegDef_t*)SPI2_I2S2_BASEADDR)
#define SPI3    ((SPI_RegDef_t*)SPI3_I2S3_BASEADDR)
#define SPI4    ((SPI_RegDef_t*)SPI4_BASEADDR)

#define I2C1    ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2    ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3    ((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1  ((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  ((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  ((USART_RegDef_t*)USART3_BASEADDR)
#define UART4   ((USART_RegDef_t*)UART4_BASEADDR)
#define UART5   ((USART_RegDef_t*)UART5_BASEADDR)
#define USART6  ((USART_RegDef_t*)USART6_BASEADDR)

/*****************************************************************************************************/
/*                          Peripheral macros                                                        */
/*****************************************************************************************************/

/**
 * Clock enable macros for GPIOx peripheral.
 */
#define GPIOA_PCLK_EN()     (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()     (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()     (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()     (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()     (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()     (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()     (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()     (RCC->AHB1ENR |= (1 << 7))

/**
 * Clock enable macros for I2Cx peripheral.
 */
#define I2C1_PCLK_EN()      (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()      (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()      (RCC->APB1ENR |= (1 << 23))

/**
 * Clock enable macros for SPIx peripheral.
 */
#define SPI1_PCLK_EN()      (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()      (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()      (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()      (RCC->APB2ENR |= (1 << 13))

/**
 * Clock enable macros for USARTx / UARTx peripheral.
 */
#define USART1_PCLK_EN()    (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()    (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()    (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()     (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()     (RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()    (RCC->APB2ENR |= (1 << 5))

/**
 * Clock enable macros for SYSCFG peripheral.
 */
#define SYSCFG_PCLK_EN()    (RCC->APB2ENR |= (1 << 14))

/**
 * Clock disable macros for GPIOx peripheral.
 */
#define GPIOA_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 7))

/**
 * Clock disable macros for I2Cx peripheral.
 */
#define I2C1_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 23))

/**
 * Clock disable macros for SPIx peripheral.
 */
#define SPI1_PCLK_DI()      (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()      (RCC->APB2ENR &= ~(1 << 13))

/**
 * Clock disable macros for USARTx / UARTx peripheral.
 */
#define USART1_PCLK_DI()    (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()    (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()    (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()    (RCC->APB2ENR &= ~(1 << 5))

/**
 * Clock disable macros for SYSCFG peripheral.
 */
#define SYSCFG_PCLK_DI()    (RCC->APB2ENR &= ~(1 << 14))

/**
 * Reset macros GPIOx peripheral.
 */
#define GPIOA_REG_RESET()   do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()   do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()   do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()   do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()   do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()   do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()   do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()   do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)

/**
 * This macro returns a code between 0 to 7 for a given GPIO base address(x).
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
 * Reset macros SPIx peripheral.
 */
#define SPI1_REG_RESET()    do{(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12));}while(0)
#define SPI2_REG_RESET()    do{(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14));}while(0)
#define SPI3_REG_RESET()    do{(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15));}while(0)
#define SPI4_REG_RESET()    do{(RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13));}while(0)

/**
 * Reset macros I2Cx peripheral.
 */
#define I2C1_REG_RESET()    do{(RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21));}while(0)
#define I2C2_REG_RESET()    do{(RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22));}while(0)
#define I2C3_REG_RESET()    do{(RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23));}while(0)

/**
 * Reset macros USARTx peripheral.
 */
#define USART1_REG_RESET()  do{(RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4));}while(0)
#define USART2_REG_RESET()  do{(RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17));}while(0)
#define USART3_REG_RESET()  do{(RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 18));}while(0)
#define UART4_REG_RESET()   do{(RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 19));}while(0)
#define UART5_REG_RESET()   do{(RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 20));}while(0)
#define USART6_REG_RESET()  do{(RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5));}while(0)

/*****************************************************************************************************/
/*                          IRQ definitions                                                          */
/*****************************************************************************************************/

/**
 * IRQ (Interrupt Request) number.
 */
#define IRQ_NO_EXTI0        6
#define IRQ_NO_EXTI1        7
#define IRQ_NO_EXTI2        8
#define IRQ_NO_EXTI3        9
#define IRQ_NO_EXTI4        10
#define IRQ_NO_EXTI9_5      23
#define IRQ_NO_EXTI15_10    40
#define IRQ_NO_SPI1         35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4         84
#define IRQ_NO_I2C1_EV      31
#define IRQ_NO_I2C1_ER      32
#define IRQ_NO_I2C2_EV      33
#define IRQ_NO_I2C2_ER      34
#define IRQ_NO_I2C3_EV      72
#define IRQ_NO_I2C3_ER      73
#define IRQ_NO_USART1       37
#define IRQ_NO_USART2       38
#define IRQ_NO_USART3       39
#define IRQ_NO_UART4        52
#define IRQ_NO_UART5        53
#define IRQ_NO_USART6       71

/**
 * IRQ priority.
 */
#define NVIC_IRQ_PRIORITY0      0
#define NVIC_IRQ_PRIORITY1      1
#define NVIC_IRQ_PRIORITY2      2
#define NVIC_IRQ_PRIORITY3      3
#define NVIC_IRQ_PRIORITY4      4
#define NVIC_IRQ_PRIORITY5      5
#define NVIC_IRQ_PRIORITY6      6
#define NVIC_IRQ_PRIORITY7      7
#define NVIC_IRQ_PRIORITY8      8
#define NVIC_IRQ_PRIORITY9      9
#define NVIC_IRQ_PRIORITY10     10
#define NVIC_IRQ_PRIORITY11     11
#define NVIC_IRQ_PRIORITY12     12
#define NVIC_IRQ_PRIORITY13     13
#define NVIC_IRQ_PRIORITY14     14
#define NVIC_IRQ_PRIORITY15     15

#endif /* STM32F446XX_H */
