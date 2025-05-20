/*
 * spi_driver.h
 *
 *  Created on: May 10, 2025
 *      Author: jesti
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_
#include "stm32f407xx.h"

typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

typedef struct {
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
}SPI_Handle_t;

// SPI device mode
#define SPI_DEVICE_MODE_MASTER	 1
#define SPI_DEVICE_MODE_Slave	 0

// SPI bus config
#define SPI_BUS_CONFIG_FULL_DUP				1
#define SPI_BUS_CONFIG_HALF_DUP				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3

// SPI clock speed
# define SPI_SCLK_SPEED_DIV2				0
# define SPI_SCLK_SPEED_DIV4				1
# define SPI_SCLK_SPEED_DIV8				2
# define SPI_SCLK_SPEED_DIV16				3
# define SPI_SCLK_SPEED_DIV32				4
# define SPI_SCLK_SPEED_DIV64				5
# define SPI_SCLK_SPEED_DIV128				6
# define SPI_SCLK_SPEED_DIV256				7

// SPI DFF
# define SPI_DFF_8BITS						0
# define SPI_DFF_16BITS						1

//CPOL
# define SPI_CPOL_HIGH						1
# define SPI_CPOL_LOW						0

// CPHA
# define SPI_CPHA_HIGH						1
# define SPI_CPHA_LOW						0

//SSM
# define SPI_SSM_EN							1
# define SPI_SSM_DI							0

// SPI application states
#define SPI_READY 							0
#define SPI_BUSY_IN_RX						1
#define SPI_BUSY_IN_TX						2

//SPI application events
#define SPI_EVENT_TX_COMPLETE				1
#define SPI_EVENT_RX_COMPLETE				2
#define SPI_EVENT_OVR_ERR					3
#define SPI_EVENT_CRC_ERR					4
// SPI intialization and deinitialization functions
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DiInit(SPI_RegDef_t *pSPIx);
//SPI clock enable function:
void SPI_ClockControl(SPI_RegDef_t *pSPIx, uint8_t Enable_or_disable);

//Data send and receive functions:
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t bufferSize);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t bufferSize);

uint8_t SPI_SendData_nonBlocking(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t bufferSize);
uint8_t SPI_ReceiveData_nonBlocking(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t bufferSize);

void SPI_IRQConfig(uint8_t IRQNumber, uint8_t Enable_or_disable);
void SPI_PriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t Enable_or_disable);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t Enable_or_disable);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);
#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
