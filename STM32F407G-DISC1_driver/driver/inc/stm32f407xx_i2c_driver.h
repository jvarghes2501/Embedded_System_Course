/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: May 17, 2025
 *      Author: jesti
 */
#include "stm32f407xx.h"
#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

typedef struct {
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t	I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

typedef struct{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxRxState;
	uint8_t DevAddr;
	uint32_t RxSize ;
	uint8_t Sr;


}I2C_Handle_t;

// I2C clock speed
#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM	400000

// I2C ack control
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

//I2C duty cycle
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

//I2C states
# define I2C_READY			0
# define I2C_BUSY_IN_RX		1
# define I2C_BUSY_IN_TX		2

// I2C status flag
#define I2C_FLAG_TXE		(1<<I2C_SR1_TXE)
#define I2C_FLAG_RXNE		(1<<I2C_SR1_RXNE)
#define I2C_FLAG_SB			(1<<I2C_SR1_SB)
#define I2C_FLAG_OVR		(1<<I2C_SR1_OVR)
#define I2C_FLAG_AF			(1<<I2C_SR1_AF)
#define I2C_FLAG_ARLO		(1<<I2C_SR1_ARLO)
#define I2C_FLAG_BERR		(1<<I2C_SR1_BERR)
#define I2C_FLAG_STOPF		(1<<I2C_SR1_STOPF)
#define I2C_FLAG_ADD10		(1<<I2C_SR1_ADD10)
#define I2C_FLAG_BTF		(1<<I2C_SR1_BTF)
#define I2C_FLAG_ADDR		(1<<I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT	(1<<I2C_SR1_TIMEOUT)

#define I2C_NO_SR			RESET
#define I2C_SR				SET

//I2C events
#define I2C_EV_TX_COMPLT		0
#define I2C_EV_RX_COMPLT		1
#define I2C_EV_STOP				2
#define I2C_EV_DATA_RCV			3
#define I2C_EV_DATA_REQ			4

//I2C error event flags
#define I2C_ERROR_BERR			3
#define I2C_ERROR_ARLO			4
#define I2C_ERROR_AF			5
#define I2C_ERROR_OVR			6
#define I2C_ERROR_TIMEOUT		7

// I2C intialization and deinitialization functions
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DiInit(I2C_RegDef_t *pI2Cx);
//SPI clock enable function:
void I2C_ClockControl(I2C_RegDef_t *pI2Cx, uint8_t Enable_or_disable);

//Data send and receive functions:
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr,uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);
uint8_t I2C_MasterSendData_nonBlocking(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveData_nonBlocking(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr,uint8_t Sr);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

//I2C priority interrupt handling
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t Enable_or_disable);
void I2C_PriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);


//peripheral control APIs
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t Enable_or_disable);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_ControlAcking(I2C_RegDef_t *pI2Cx,uint8_t Enable_or_disable);

//application call back
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
