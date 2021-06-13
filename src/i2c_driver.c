/*****************************************************************************************************
* FILENAME :        i2c_driver.c
*
* DESCRIPTION :
*       File containing the APIs for configuring the I2C peripheral.
*
* PUBLIC FUNCTIONS :
*       void    I2C_Init(I2C_Handle_t* pI2C_Handle)
*       void    I2C_DeInit(I2C_RegDef_t* pI2Cx)
*       void    I2C_PerClkCtrl(I2C_RegDef_t* pI2Cx, uint8_t en_or_di)
*       void    I2C_MasterSendData(I2C_Handle_t* pI2C_Handle, uint8_t* pTxBuffer, uint32_t len, uint8_t slave_addr, sr_t sr)
*       void    I2C_MasterReceiveData(I2C_Handle_t* pI2C_Handle, uint8_t* pRxBuffer, uint8_t len, uint8_t slave_addr, sr_t sr)
*       uint8_t I2C_MasterSendDataIT(I2C_Handle_t* pI2C_Handle, uint8_t* pTxBuffer, uint32_t len, uint8_t slave_addr, sr_t sr)
*       uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t* pI2C_Handle, uint8_t* pRxBuffer, uint8_t len, uint8_t slave_addr, sr_t sr)
*       void    I2C_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di)
*       void    I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
*       void    I2C_EV_IRQHandling(I2C_Handle_t* pI2C_Handle)
*       void    I2C_ER_IRQHandling(I2C_Handle_t* pI2C_Handle)
*       void    I2C_Enable(I2C_RegDef_t* pI2Cx, uint8_t en_or_di)
*       uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint32_t flagname)
*       void    I2C_CloseReceiveData(I2C_Handle_t* pI2C_Handle)
*       void    I2C_CloseSendData(I2C_Handle_t* pI2C_Handle)
*       void    I2C_ApplicationEventCallback(I2C_Handle_t* pI2C_Handle, uint8_t app_event)
*
* NOTES :
*       For further information about functions refer to the corresponding header file.
*
**/

#include <stdint.h>
#include <stddef.h>
#include "stm32f446xx.h"
#include "i2c_driver.h"

#define FREQ_8MHZ    8000000
#define FREQ_16MHZ   16000000

static uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
static uint8_t APB1_PreScaler[4] = {2, 4, 8, 16};

/**
 * @fn RCC_GetPCLK1Value
 *
 * @brief function to calculate the PCLK1 clock value.
 *
 * @return PCLK1 clock value.
 */
static uint32_t RCC_GetPCLK1Value(void);

/**
 * @fn RCC_GetPLLOutputClock
 *
 * @brief function to calculate the PLL clock value.
 *
 * @return PLL clock value.
 */
static uint32_t RCC_GetPLLOutputClock(void);

/**
 * @fn I2C_GenerateStartCondition
 *
 * @brief function to generate the start condition for transmission.
 *
 * @param[in] pI2Cx the base address of the I2Cx peripheral.
 *
 * @return void.
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx);

/**
 * @fn I2C_GenerateStartCondition
 *
 * @brief function to set address of the slave for transmission.
 *
 * @param[in] pI2Cx the base address of the I2Cx peripheral.
 * @param[in] slave_addr address of the slave.
 * @param[in] rw selection to read or write, @I2C_RW.
 *
 * @return void.
 */
static void I2C_ExecuteAddressPhase(I2C_RegDef_t* pI2Cx, uint8_t slave_addr, rw_t rw);

/**
 * @fn I2C_GenerateStartCondition
 *
 * @brief function to clear address flag.
 *
 * @param[in] pI2C_Handle handle structure for the I2C peripheral.
 *
 * @return void.
 */
static void I2C_ClearADDRFlag(I2C_Handle_t* pI2C_Handle);

/**
 * @fn I2C_GenerateStopCondition
 *
 * @brief function to generate stop condition.
 *
 * @param[in] pI2Cx the base address of the I2Cx peripheral.
 *
 * @return void.
 */
static void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx);

/**
 * @fn I2C_ManageAcking
 *
 * @brief function to enable or disable the acking.
 *
 * @param[in] pI2Cx the base address of the I2Cx peripheral.
 * @param[in] en_or_di to enable or disable @I2C_ACKCONTROL@
 *
 * @return void.
 */
static void I2C_ManageAcking(I2C_RegDef_t* pI2Cx, uint8_t en_or_di);

/**
 * @fn I2C_MasterHandleTXEInterrupt
 *
 * @brief function to manage transmission in handle interrupt.
 *
 * @param[in] pI2C_Handle handle structure for the I2C peripheral.
 *
 * @return void.
 */
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t* pI2C_Handle);

/**
 * @fn I2C_MasterHandleRXNEInterrupt
 *
 * @brief function to manage reception in handle interrupt.
 *
 * @param[in] pI2C_Handle handle structure for the I2C peripheral.
 *
 * @return void.
 */
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t* pI2C_Handle);

void I2C_Init(I2C_Handle_t* pI2C_Handle){

    uint8_t temp = 0;
    uint16_t ccr_value = 0;

    /* Enable the peripheral clock */
    I2C_PerClkCtrl(pI2C_Handle->pI2Cx, ENABLE);

    /* ACK control bit */
    temp |= (pI2C_Handle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);
    pI2C_Handle->pI2Cx->CR1 = temp;

    /* FREQ field of CR2 */
    temp = 0;
    temp |= RCC_GetPCLK1Value()/1000000U;
    pI2C_Handle->pI2Cx->CR2 = temp & 0x3F;

    /* Device own address */
    temp = 0;
    temp |= (pI2C_Handle->I2C_Config.I2C_DeviceAddress << 1);
    temp |= (1 << 14);
    pI2C_Handle->pI2Cx->OAR1 = temp;

    /* CCR */
    temp = 0;
    if(pI2C_Handle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
        /* standard mode */
        ccr_value = RCC_GetPCLK1Value()/(2*pI2C_Handle->I2C_Config.I2C_SCLSpeed);
        temp |= (ccr_value & 0xFFF);
    }
    else{
        /* fast mode */
        temp |= (1 << I2C_CCR_FS);
        temp |= (pI2C_Handle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);
        if(pI2C_Handle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
            ccr_value = RCC_GetPCLK1Value()/(3*pI2C_Handle->I2C_Config.I2C_SCLSpeed);
        }
        else{
            ccr_value = RCC_GetPCLK1Value()/(25*pI2C_Handle->I2C_Config.I2C_SCLSpeed);
        }
        temp |= (ccr_value & 0xFFF);
    }
    pI2C_Handle->pI2Cx->CCR = temp;

    /* TRISE configuration */
    if(pI2C_Handle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
        /* standard mode */
        temp = (RCC_GetPCLK1Value()/1000000U) + 1;
    }
    else{
        /* fast mode */
        temp = ((RCC_GetPCLK1Value()*300)/1000000000U) + 1;
    }
    pI2C_Handle->pI2Cx->TRISE = (temp & 0x3F);
}

void I2C_DeInit(I2C_RegDef_t* pI2Cx){

    if(pI2Cx == I2C1){
        I2C1_REG_RESET();
    }
    else if(pI2Cx == I2C2){
        I2C2_REG_RESET();
    }
    else if(pI2Cx == I2C3){
        I2C3_REG_RESET();
    }
    else{
        /* do nothing */
    }
}

void I2C_PerClkCtrl(I2C_RegDef_t* pI2Cx, uint8_t en_or_di){

    if(en_or_di == ENABLE){
        if(pI2Cx == I2C1){
            I2C1_PCLK_EN();
        }
        else if(pI2Cx == I2C2){
            I2C2_PCLK_EN();
        }
        else if(pI2Cx == I2C3){
            I2C3_PCLK_EN();
        }
        else{
            /* do nothing */
        }
    }
    else{
        if(pI2Cx == I2C1){
            I2C1_PCLK_DI();
        }
        else if(pI2Cx == I2C2){
            I2C2_PCLK_DI();
        }
        else if(pI2Cx == I2C3){
            I2C3_PCLK_DI();
        }
        else{
            /* do nothing */
        }
    }
}

void I2C_MasterSendData(I2C_Handle_t* pI2C_Handle, uint8_t* pTxBuffer, uint32_t len, uint8_t slave_addr, sr_t sr){

    /* Generate start condition */
    I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

    /* Check SB flag in SR1 */
    /* Note: until SB is cleared SCL will be stretched (pulled to LOW) */
    while(!I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_SB));

    /* Send address of the slave with r/w bit set to w(0) (total 8 bits) */
    I2C_ExecuteAddressPhase(pI2C_Handle->pI2Cx, slave_addr, WRITE);

    /* Confirm address phase is completed by checking the ADDR flag in SR1 */
    while(!I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_ADDR));

    /* Clear the ADDR flag */
    /* Note: until ADDR is cleared SCL will be stretched (pulled in LOW) */
    I2C_ClearADDRFlag(pI2C_Handle);

    /* Send data until len becomes 0 */
    while(len > 0){
        while(!I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_TXE));
        pI2C_Handle->pI2Cx->DR = *pTxBuffer;
        pTxBuffer++;
        len--;
    }

    /* Wait for TXE=1 and BTF=1 before generating STOP condition */
    /* Note: TXE=1, BTF=1, means that both SR and DR are empty and next transmission should begin */
    /* when BTF=1 SCL will be stretched (pulled to LOW) */
    while(!I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_TXE));
    while(!I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_BTF));

    /* Generate STOP condition */
    /* Note: generating STOP, automatically clears the BTF */
    if(sr == I2C_DISABLE_SR){
        I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
    }
}

void I2C_MasterReceiveData(I2C_Handle_t* pI2C_Handle, uint8_t* pRxBuffer, uint8_t len, uint8_t slave_addr, sr_t sr){

    uint32_t i = 0;

    /* Generate start condition */
    I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

    /* Check SB flag in SR1 */
    /* Note: until SB is cleared SCL will be stretched (pulled to LOW) */
    while(!I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_SB));

    /* Send address of the slave with r/w bit set to R(1) (total 8 bits) */
    I2C_ExecuteAddressPhase(pI2C_Handle->pI2Cx, slave_addr, READ);

    /* Wait until address phase is completed by checking the ADDR flag in SR1 */
    while(!I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_ADDR));

    /* Procedure to read only 1 byte from slave */
    if(len == 1){
        /* Disable Acking */
        I2C_ManageAcking(pI2C_Handle->pI2Cx, I2C_ACK_DISABLE);

        /* Clear ADDR flag */
        I2C_ClearADDRFlag(pI2C_Handle);

        /* Wait until RXNE=1 */
        while(!I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_RXNE));

        /* Generate stop condition */
        if(sr == I2C_DISABLE_SR){
            I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
        }

        /* Read data into buffer */
        *pRxBuffer = pI2C_Handle->pI2Cx->DR;
    }

    /* Procedure to read data from slave when len > 1 */
    if(len > 1){
        /* Clear ADDR flag */
        I2C_ClearADDRFlag(pI2C_Handle);

        /* Read datas until len becomes zero */
        for(i = len; i > 0; i--){
            /* Wait until RXNE becomes 1 */
            while(!I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_RXNE));

            if(i == 2){ /* If last 2 bytes are remaining */
                /* Disable Acking */
                I2C_ManageAcking(pI2C_Handle->pI2Cx, I2C_ACK_DISABLE);

                /* Generate stop condition */
                if(sr == I2C_DISABLE_SR){
                    I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
                }
            }
            /* Read the data from data register into the buffer */
            *pRxBuffer = pI2C_Handle->pI2Cx->DR;

            /* Increment the buffer address */
            pRxBuffer++;
        }
    }

    /* Re-enable Acking */
    if(pI2C_Handle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE){
        I2C_ManageAcking(pI2C_Handle->pI2Cx, I2C_ACK_ENABLE);
    }
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t* pI2C_Handle, uint8_t* pTxBuffer, uint32_t len, uint8_t slave_addr, sr_t sr){

    uint8_t busystate = pI2C_Handle->TxRxState;

    if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)){
        pI2C_Handle->pTxBuffer = pTxBuffer;
        pI2C_Handle->TxLen = len;
        pI2C_Handle->TxRxState = I2C_BUSY_IN_TX;
        pI2C_Handle->DevAddr = slave_addr;
        pI2C_Handle->Sr = sr;

        /* Generate START condition */
        I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

        /* Enable ITBUFEN control bit */
        pI2C_Handle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

        /* Enable ITEVTEN control bit */
        pI2C_Handle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

        /* Enable ITERREN control bit */
        pI2C_Handle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }

    return busystate;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t* pI2C_Handle, uint8_t* pRxBuffer, uint8_t len, uint8_t slave_addr, sr_t sr){

    uint8_t busystate = pI2C_Handle->TxRxState;

    if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)){
        pI2C_Handle->pRxBuffer = pRxBuffer;
        pI2C_Handle->RxLen = len;
        pI2C_Handle->TxRxState = I2C_BUSY_IN_RX;
        pI2C_Handle->RxSize = len;
        pI2C_Handle->DevAddr = slave_addr;
        pI2C_Handle->Sr = sr;

        /* Generate START condition */
        I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

        /* Enable ITBUFEN control bit */
        pI2C_Handle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

        /* Enable ITEVTEN control bit */
        pI2C_Handle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

        /* Enable ITERREN control bit */
        pI2C_Handle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }

    return busystate;
}

void I2C_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di){

    if(en_or_di == ENABLE){
        if(IRQNumber <= 31){
            /* Program ISER0 register */
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64){
            /* Program ISER1 register */
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96){
            /* Program ISER2 register */
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
        else{
            /* do nothing */
        }
    }
    else{
        if(IRQNumber <= 31){
            /* Program ICER0 register */
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64){
            /* Program ICER1 register */
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96){
            /* Program ICER2 register */
            *NVIC_ICER2 |= (1 << (IRQNumber % 64));
        }
        else{
            /* do nothing */
        }
    }
}

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
    /* Find out the IPR register */
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;
    uint8_t shift = (8*iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

    *(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift);
}

void I2C_EV_IRQHandling(I2C_Handle_t* pI2C_Handle){

    /* Interrupt handling for both master and slave mode of a device */
    uint8_t temp1, temp2, temp3;

    temp1 = pI2C_Handle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
    temp2 = pI2C_Handle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
    temp3 = pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

    /* Handle for interrupt generated by SB event */
    /* Note: SB flag is only applicable in master mode */
    if(temp1 && temp3){
        /* SB flag is set */
        if(pI2C_Handle->TxRxState == I2C_BUSY_IN_TX){
            I2C_ExecuteAddressPhase(pI2C_Handle->pI2Cx, pI2C_Handle->DevAddr, WRITE);
        }
        else if(pI2C_Handle->TxRxState == I2C_BUSY_IN_RX){
            I2C_ExecuteAddressPhase(pI2C_Handle->pI2Cx, pI2C_Handle->DevAddr, READ);
        }
        else{
            /* do nothing */
        }
    }

    temp3 = pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
    /* Handle for interrupt generated by ADDR event */
    /* Note: when master mode address is sent */
    /*       when slave mode address matched with own address */
    if(temp1 && temp3){
        /* ADDR flag is set */
        I2C_ClearADDRFlag(pI2C_Handle);
    }

    temp3 = pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
    /* Handle for interrupt generated by BTF (Byte Transfer Finished) event */
    if(temp1 && temp3){
        /* BTF flag is set */
        if(pI2C_Handle->TxRxState == I2C_BUSY_IN_TX){
            /* Make sure that TXE is also set */
            if(pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)){
                /* BTF, TXE = 1 */
                if(pI2C_Handle->TxLen == 0){
                    /* Generate the STOP condition */
                    if(pI2C_Handle->Sr == I2C_DISABLE_SR)
                        I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);

                    /* Reset all the member elements of the handle structure */
                    I2C_CloseSendData(pI2C_Handle);

                    /* Notify the application about transmission complete */
                    I2C_ApplicationEventCallback(pI2C_Handle, I2C_EVENT_TX_CMPLT);
                }
            }
        }
        else if(pI2C_Handle->TxRxState == I2C_BUSY_IN_RX){
            /* do nothing */
        }
        else{
            /* do nothing */
        }
    }

    temp3 = pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
    /* Handle for interrupt generated by STOPF event */
    /* Note: stop detection flag is applicable only in slave mode */
    if(temp1 && temp3){
        /* STOPF flag is set */
        /* Clear the STOP: read SR1, write to CR1 */
        pI2C_Handle->pI2Cx->CR1 |= 0x0000;
        /* Notify the application that STOP is detected */
        I2C_ApplicationEventCallback(pI2C_Handle, I2C_EVENT_STOP);
    }

    temp3 = pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
    /* Handle for interrupt generated by TXE event */
    if(temp1 && temp2 && temp3){
        /* Check for device mode */
        if(pI2C_Handle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
            /* TXE flag is set */
            if(pI2C_Handle->TxRxState == I2C_BUSY_IN_TX){
                I2C_MasterHandleTXEInterrupt(pI2C_Handle);
            }
        }
    }

    temp3 = pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
    /* Handle for interrupt generated by RXNE event */
    if(temp1 && temp2 && temp3){
        /* Check for device mode */
        if(pI2C_Handle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
            /* The device is master */
            /* RXNE flag is set */
            if(pI2C_Handle->TxRxState == I2C_BUSY_IN_RX){
                I2C_MasterHandleRXNEInterrupt(pI2C_Handle);
            }
        }
    }
}

void I2C_ER_IRQHandling(I2C_Handle_t* pI2C_Handle){
}

void I2C_Enable(I2C_RegDef_t* pI2Cx, uint8_t en_or_di){

    if(en_or_di == ENABLE){
        pI2Cx->CR1 |= (1 << I2C_CR1_PE);
    }
    else{
        pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
    }
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint32_t flagname){

    if(pI2Cx->SR1 & flagname){
        return  FLAG_SET;
    }
    return FLAG_RESET;
}

void I2C_CloseReceiveData(I2C_Handle_t* pI2C_Handle){
    /* Disable ITBUFEN control bit */
    pI2C_Handle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

    /* Disable ITEVTEN control bit */
    pI2C_Handle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    pI2C_Handle->TxRxState = I2C_READY;
    pI2C_Handle->pRxBuffer = NULL;
    pI2C_Handle->RxLen = 0;
    pI2C_Handle->RxSize = 0;

    if(pI2C_Handle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE){
        I2C_ManageAcking(pI2C_Handle->pI2Cx, ENABLE);
    }
}

void I2C_CloseSendData(I2C_Handle_t* pI2C_Handle){
    /* Disable ITBUFEN control bit */
    pI2C_Handle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

    /* Disable ITEVTEN control bit */
    pI2C_Handle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    pI2C_Handle->TxRxState = I2C_READY;
    pI2C_Handle->pTxBuffer = NULL;
    pI2C_Handle->TxLen = 0;
}

__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t* pI2C_Handle, uint8_t app_event){

    /* This is a weak implementation. The application may override this function */
}

static uint32_t RCC_GetPCLK1Value(void){

    uint32_t pclk1, systemclk, clksrc;
    uint8_t temp;
    uint8_t ahbp, apb1p;

    /* Check for SWS */
    clksrc = ((RCC->CFGR >> 2) & 0x3);

    if(clksrc == 0){
        /* clk is HSI */
        systemclk = FREQ_16MHZ;
    }
    else if(clksrc == 1){
        /* clk is HSE */
        systemclk = FREQ_8MHZ;
    }
    else if(clksrc == 2){
        /* clk is PLL */
        systemclk = RCC_GetPLLOutputClock();
    }
    else{
        /* do nothing */
    }

    /* AHB prescaler */
    temp = ((RCC->CFGR >> 4) & 0xF);

    if(temp < 8){
        ahbp = 1;
    }
    else{
        ahbp = AHB_PreScaler[temp-8];
    }

    /* APB1 prescaler */
    temp = ((RCC->CFGR >> 10) & 0x7);

    if(temp < 4){
        apb1p = 1;
    }
    else{
        apb1p = APB1_PreScaler[temp-4];
    }

    pclk1 = (systemclk/ahbp)/apb1p;

    return pclk1;
}

static uint32_t RCC_GetPLLOutputClock(void){
    /* TO BE DONE */
    return 0;
}

static void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx){

    pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t* pI2Cx, uint8_t slave_addr, rw_t rw){

    slave_addr = slave_addr << 1;
    if(rw == WRITE){
        slave_addr &= ~(1); /* slave_addr is slave address + r/w bit = 0 */
    }
    else if(rw == READ){
        slave_addr |= 1; /* slave_addr is slave address + r/w bit = 1 */
    }
    else{
        /* do nothing */
    }
    pI2Cx->DR = slave_addr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t* pI2C_Handle){

    uint32_t dummy_read;

    /* Check for device mode */
    if(pI2C_Handle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
        /* Master mode */
        if(pI2C_Handle->TxRxState == I2C_BUSY_IN_RX){
            if(pI2C_Handle->RxSize == 1){
                /* Disable ACK */
                I2C_ManageAcking(pI2C_Handle->pI2Cx, DISABLE);

                /* Clear ADDR flag (read SR1, read SR2 */
                dummy_read = pI2C_Handle->pI2Cx->SR1;
                dummy_read = pI2C_Handle->pI2Cx->SR2;
                (void)dummy_read;
            }
        }
        else{
            /* Clear ADDR flag (read SR1, read SR2 */
            dummy_read = pI2C_Handle->pI2Cx->SR1;
            dummy_read = pI2C_Handle->pI2Cx->SR2;
            (void)dummy_read;
        }
    }
    else{
        /* Slave mode */
        /* Clear ADDR flag (read SR1, read SR2 */
        dummy_read = pI2C_Handle->pI2Cx->SR1;
        dummy_read = pI2C_Handle->pI2Cx->SR2;
        (void)dummy_read;
    }
}

static void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx){

    pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_ManageAcking(I2C_RegDef_t* pI2Cx, uint8_t en_or_di){

    if(en_or_di == I2C_ACK_ENABLE){
        /* enable the ack */
        pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
    }
    else{
        /* disable the ack */
        pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
    }
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t* pI2C_Handle){

    if(pI2C_Handle->TxLen > 0){
        /* Load the data into DR */
        pI2C_Handle->pI2Cx->DR = *(pI2C_Handle->pTxBuffer);

        /* Decrement the TxLen */
        pI2C_Handle->TxLen--;

        /* Increment the buffer address */
        pI2C_Handle->pTxBuffer++;
    }
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t* pI2C_Handle){

    if(pI2C_Handle->RxSize == 1){
        *pI2C_Handle->pRxBuffer = pI2C_Handle->pI2Cx->DR;
        pI2C_Handle->RxLen--;
    }

    if(pI2C_Handle->RxSize > 1){
        if(pI2C_Handle->RxLen == 2){
            /* Clear ACK */
            I2C_ManageAcking(pI2C_Handle->pI2Cx, DISABLE);
        }
        /* Read DR */
        *pI2C_Handle->pRxBuffer = pI2C_Handle->pI2Cx->DR;
        pI2C_Handle->pRxBuffer++;
        pI2C_Handle->RxLen--;
    }

    if(pI2C_Handle->RxLen == 0){
        /* Close I2C data reception and notify the application */
        /* Generate the STOP condition */
        if(pI2C_Handle->Sr == I2C_DISABLE_SR)
            I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);

        /* Close the I2C RX */
        I2C_CloseReceiveData(pI2C_Handle);

        /* Notify the application */
        I2C_ApplicationEventCallback(pI2C_Handle, I2C_EVENT_RX_CMPLT);
    }
}
