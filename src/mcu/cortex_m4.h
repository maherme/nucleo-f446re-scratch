/********************************************************************************************************//**
* @file cortex_m4.h
*
* @brief Header file containing the prototypes of the APIs for configuring the Cortex M4.
*
* Public Functions:
*       - void IRQConfig(uint8_t IRQNumber, uint8_t en_or_di)
*       - void IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
*       - void IRQClearPending(uint8_t IRQNumber)
*       - void Enter_WFI(void)
*       - void Enter_WFE(void)
*       - void EnableSleepOnExit(void)
*       - void DisableSleepOnExit(void)
*       - void EnableSEVONPEND(void)
*       - void DisableSEVONPEND(void)
*       - void EnableSleepDeep(void)
*       - void DisableSleepDeep(void)
*/

#ifndef CORTEX_M4_H
#define CORTEX_M4_H

#include <stdint.h>

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

#define NVIC_ICPR0      ((volatile uint32_t*)0xE000E280)        /**< @brief NVIC ICPR0 Register Addr */
#define NVIC_ICPR1      ((volatile uint32_t*)0xE000E284)        /**< @brief NVIC ICPR1 Register Addr */
#define NVIC_ICPR2      ((volatile uint32_t*)0xE000E288)        /**< @brief NVIC ICPR2 Register Addr */
#define NVIC_ICPR3      ((volatile uint32_t*)0xE000E28C)        /**< @brief NVIC ICPR3 Register Addr */

#define NVIC_PR_BASEADDR    ((volatile uint32_t*)0xE000E400)    /**< @brief NVIC Priority Register Addr */
#define SCB_BASEADDR        ((volatile uint32_t*)0xE000E008)    /**< @brief System control block Reg Addr */
#define DBGMCU_BASEADDR     0xE0042000                          /**< @brief Debug Peripheral Base Addr */

#define NO_PR_BITS_IMPLEMENTED  4 /**< @brief Numb of priority bits implemented in the Priority Register */
/** @} */

/**
 * @brief Core register definition structure for SCB.
 */
typedef struct
{
    volatile uint32_t ACTLR;            /**< @brief Auxiliary control register          Addr offset 0x00 */
    volatile uint32_t RESERVED0;        /**< @brief Reserved                            Addr offset 0x04 */
    volatile uint32_t STCSR;            /**< @brief SysTick control and status register Addr offset 0x08 */
    volatile uint32_t STRVR;            /**< @brief SysTick reload value register       Addr offset 0x0C */
    volatile uint32_t STCVR;            /**< @brief SysTick current value register      Addr offset 0x10 */
    volatile uint32_t STCR;             /**< @brief SysTick calibration value register  Addr offset 0x14 */
    volatile uint32_t RESERVED1[824];   /**< @brief Reserved                            Addr offset 0x18 */
    volatile uint32_t CPUID;            /**< @brief CPUID base register                 Addr offset 0xCF8 */
    volatile uint32_t ICSR;             /**< @brief Interrupt ctrl and state register   Addr offset 0xCFC */
    volatile uint32_t VTOR;             /**< @brief Vector table offset register        Addr offset 0xD00 */
    volatile uint32_t AIRCR;            /**< @brief App interrupt and reset ctrl reg    Addr offset 0xD04 */
    volatile uint32_t SCR;              /**< @brief System control register             Addr offset 0xD08 */
    volatile uint32_t CCR;              /**< @brief Configuration and control register  Addr offset 0xD0C */
    volatile uint32_t SHPR1;            /**< @brief System handler priority register 1  Addr offset 0xD10 */
    volatile uint32_t SHPR2;            /**< @brief System handler priority register 2  Addr offset 0xD14 */
    volatile uint32_t SHPR3;            /**< @brief System handler priority register 3  Addr offset 0xD18 */
    volatile uint32_t SHCSR;            /**< @brief System haldler ctrl and status reg  Addr offset 0xD1C */
    volatile uint32_t CFSR;             /**< @brief Configurable fault status register  Addr offset 0xD20 */
    volatile uint32_t HFSR;             /**< @brief HardFault status register           Addr offset 0xD24 */
    volatile uint32_t DFSR;             /**< @brief Debug fault status register         Addr offset 0xD28 */
    volatile uint32_t MMFAR;            /**< @brief MemManage address register          Addr offset 0xD2C */
    volatile uint32_t BFAR;             /**< @brief BusFault address register           Addr offset 0xD30 */
    volatile uint32_t AFSR;             /**< @brief Auxiliary fault status register     Addr offset 0xD34 */
    volatile uint32_t ID_PFR0;          /**< @brief Processor feature register 0        Addr offset 0xD38 */
    volatile uint32_t ID_PFR1;          /**< @brief Processor feature register 1        Addr offset 0xD3C */
    volatile uint32_t ID_DRF0;          /**< @brief Debug features register 0           Addr offset 0xD40 */
    volatile uint32_t ID_AFR0;          /**< @brief Auxiliary features register 0       Addr offset 0xD44 */
    volatile uint32_t ID_MMFR0;         /**< @brief Memory model feature register 0     Addr offset 0xD48 */
    volatile uint32_t ID_MMFR1;         /**< @brief Memory model feature register 1     Addr offset 0xD4C */
    volatile uint32_t ID_MMFR2;         /**< @brief Memory model feature register 2     Addr offset 0xD50 */
    volatile uint32_t ID_MMFR3;         /**< @brief Memory model feature register 3     Addr offset 0xD54 */
    volatile uint32_t ID_ISAR0;         /**< @brief Instruction set attributes reg 0    Addr offset 0xD58 */
    volatile uint32_t ID_ISAR1;         /**< @brief Instruction set attributes reg 1    Addr offset 0xD5C */
    volatile uint32_t ID_ISAR2;         /**< @brief Instruction set attributes reg 2    Addr offset 0xD60 */
    volatile uint32_t ID_ISAR3;         /**< @brief Instruction set attributes reg 3    Addr offset 0xD64 */
    volatile uint32_t ID_ISAR4;         /**< @brief Instruction set attributes reg 4    Addr offset 0xD68 */
    volatile uint32_t RESERVED2[5];     /**< @brief Reserved                            Addr offset 0xD6C */
    volatile uint32_t CPACR;            /**< @brief Coprocessor access control register Addr offset 0xD80 */
    volatile uint32_t RESERVED3[93];    /**< @brief Reserverd                           Addr offset 0xD84 */
    volatile uint32_t STIR;             /**< @brief Software triggered interrupt reg    Addr offset 0xEF8 */
}SCB_RegDef_t;

/**
 * @name Bit position definitions SCR
 * @{
 */
#define SCB_SCR_SLEEPONEXIT     1   /**< @brief Sleep on exit when returning from handler to thread mode  */
#define SCB_SCR_SLEEPDEPP       2   /**< @brief Sleep or deep sleep in low power mode */
#define SCB_SCR_SEVONPEND       4   /**< @brief Send event on pending bit */
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

/**
 * @name Base Addresses Typecasted to xxx_RegDef_t.
 * @{
 */
#define SCB     ((SCB_RegDef_t*)SCB_BASEADDR)       /**< @brief SCB base addr reg definition */
/** @} */
/***********************************************************************************************************/
/*                                       APIs Supported                                                    */
/***********************************************************************************************************/

/**
 * @brief Function to configure the IRQ number of the Cortex M4.
 * @param[in] IRQNumber number of the interrupt.
 * @param[in] en_or_di for enable or disable.
 * @return void.
 */
void IRQConfig(uint8_t IRQNumber, uint8_t en_or_di);

/**
 * @brief Function to configure the IRQ number of the Cortex M4.
 * @param[in] IRQNumber number of the interrupt.
 * @param[in] IRQPriority priority of the interrupt.
 * @return void.
 */
void IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * @brief Function to clear the pending bit of the IRQ number.
 * @param[in] IRQNumber number of the interrupt.
 * @return void.
 */
void IRQClearPending(uint8_t IRQNumber);

/**
 * @brief Function to execute the Wait For Interrupt instruction.
 * @return void.
 */
__attribute__((always_inline)) static inline void Enter_WFI(void){

    __asm volatile ("WFI");
}

/**
 * @brief Function to execute the Wait For Event instruction.
 * @return void.
 */
__attribute__((always_inline)) static inline void Enter_WFE(void){

    __asm volatile ("WFE");
}

/**
 * @brief Function to enable sleep on exit.
 * @return void.
 */
void EnableSleepOnExit(void);

/**
 * @brief Function to disable sleep on exit.
 * @return void.
 */
void DisableSleepOnExit(void);

/**
 * @brief Function to set SEVONPEND bit in SCR register.
 * @return void.
 */
void EnableSEVONPEND(void);

/**
 * @brief Function to clear SEVONPEND bit in SCR register.
 * @return void.
 */
void DisableSEVONPEND(void);

/**
 * @brief Function to enable deep sleep.
 * @return void.
 */
void EnableSleepDeep(void);

/**
 * @brief Function to disable deep sleep.
 * @return void.
 */
void DisableSleepDeep(void);

#endif /* CORTEX_M4_H */