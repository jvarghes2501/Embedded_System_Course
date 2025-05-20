/*
 * spi_driver.c
 *
 *  Created on: May 10, 2025
 *      Author: jesti
 */
# include "stm32f407xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);
void SPI_ClockControl(SPI_RegDef_t *pSPIx, uint8_t Enable_or_disable){
	if (Enable_or_disable==ENABLE){
			if (pSPIx == SPI1){
				SPI1_CLOCK_ENABLE();
			}else if (pSPIx == SPI2){
				SPI2_CLOCK_ENABLE();
			}else if (pSPIx == SPI3){
				SPI3_CLOCK_ENABLE();
			}
		}else{
			if (pSPIx == SPI1){
				SPI1_CLOCK_DISABLE();
			}else if (pSPIx == SPI2){
				SPI2_CLOCK_DISABLE();
			}else if (pSPIx == SPI3){
				SPI3_CLOCK_DISABLE();
			}
		}

}

void SPI_Init(SPI_Handle_t *pSPIHandle){

	//enable the clock
	SPI_ClockControl(pSPIHandle->pSPIx, ENABLE);
	uint32_t temp_reg = 0;
	//configure the device mode
	temp_reg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	// configure the bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FULL_DUP){
		// bidi mode should be cleared
		temp_reg &= ~(1<<15);
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HALF_DUP){
		//bidi mode should be set
		temp_reg |= (1<<15);
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// bidi mode should be cleared
		temp_reg &= ~(1<<15);
		//RX only bit must be set
		temp_reg |= (1<<10);

	}

	// configure the spi serial clock speed
	temp_reg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << 3;
	//configure the DFF
	temp_reg |= pSPIHandle->SPIConfig.SPI_DFF << 11;
	//configure the CPOL
	temp_reg |= pSPIHandle->SPIConfig.SPI_CPOL << 1;
	// configure the CPHA
	temp_reg |= pSPIHandle->SPIConfig.SPI_CPHA << 0;

	pSPIHandle->pSPIx->CR1 = temp_reg;



}
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t Enable_or_disable){
	if(Enable_or_disable == ENABLE){
		pSPIx->CR1 |= (1<<SPI_CR1_SPE);
	}else{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);

	}
}
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t Enable_or_disable){
	if(Enable_or_disable == ENABLE){
		pSPIx->CR1 |= (1<<SPI_CR1_SSI);
	}else{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SSI);

		}
}
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t bufferSize){
	while (bufferSize > 0){
		// wait till TXE is set
		while (!(pSPIx->SR & (1<<1)));

		// check the DFF in CR1
		if((pSPIx->CR1 &(1<<SPI_CR1_DFF))){
			//16 bit DFF
			// load the data into DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			bufferSize--;
			bufferSize--;
			(uint16_t*)pTxBuffer++;
		}
		else{
			//8 bit DFF
			pSPIx->DR = *(pTxBuffer);
			bufferSize--;
			pTxBuffer++;
		}
	}
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t bufferSize){
	while (bufferSize > 0){
		// wait till RXNE is set
		while (!(pSPIx->SR & (1<<0)));

		// check the DFF in CR1
		if((pSPIx->CR1 &(1<<SPI_CR1_DFF))){
			//16 bit DFF
			// load the data from DR to rxbuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			bufferSize--;
			bufferSize--;
			(uint16_t*)pRxBuffer++;
		}
		else{
			//8 bit DFF
			*(pRxBuffer) = pSPIx->DR;
			bufferSize--;
			pRxBuffer++;
		}
	}

}

void SPI_IRQConfig(uint8_t IRQNumber, uint8_t Enable_or_disable){
	if (Enable_or_disable == ENABLE){
		if(IRQNumber <=31){
			//change the ISER0 reg
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if (IRQNumber > 31 && IRQNumber < 64){
			// change the ISER1 reg
			*NVIC_ISER1 |= (1 << (IRQNumber%32));

		}else if (IRQNumber >= 64 && IRQNumber < 96){
			// change the ISER2 reg
			*NVIC_ISER2 |= (1 << (IRQNumber%64));

		}
	}else{
		if(IRQNumber <=31){
			//change the ICER0 reg
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if (IRQNumber > 31 && IRQNumber < 64){
			// change the ICER1 reg
			*NVIC_ICER1 |= (1 << (IRQNumber%32));

		}else if (IRQNumber >= 64 && IRQNumber < 96){
			// change the ICER2 reg
			*NVIC_ICER2 |= (1 << (IRQNumber%64));

		}

	}
}
void SPI_PriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority){
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber % 4;

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << ((8*iprx_section)+(8-PR_BITS_IMPLEMENTED)));
}

uint8_t SPI_SendData_nonBlocking(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t bufferSize){
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		// save the tx buffer address and size in global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = bufferSize;
		//mark the spi state as busy in transmission so no other code can control same SPI peripheral until transmission is done
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		// enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_TXEIE);
		}
		//data transmission will be handled by the ISR code
	return pSPIHandle->TxState;
}
uint8_t SPI_ReceiveData_nonBlocking(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t bufferSize){
	if(pSPIHandle->RxState != SPI_BUSY_IN_RX){
		// save the tx buffer address and size in global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = bufferSize;
		//mark the spi state as busy in transmission so no other code can control same SPI peripheral until transmission is done
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		// enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_RXNEIE);
		}
		//data transmission will be handled by the ISR code
	return pSPIHandle->RxState;

}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	// check the DFF in CR1
	if((pSPIHandle->pSPIx->CR1 &(1<<SPI_CR1_DFF))){
		//16 bit DFF
		// load the data into DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else{
		//8 bit DFF
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
	if(!pSPIHandle->TxLen){
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_COMPLETE);
	}

}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
	// check the DFF in CR1
	if((pSPIHandle->pSPIx->CR1 &(1<<SPI_CR1_DFF))){
		//16 bit DFF
		// load the data from DR to rxbuffer address
		*((uint16_t*)pSPIHandle->pRxBuffer) =(uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer--;
		pSPIHandle->pRxBuffer--;
	}
	else{
		//8 bit DFF
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer--;
	}
	if(!pSPIHandle->RxLen)
	{
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_COMPLETE);
	}

}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){
	uint8_t temp;
	//clear the ovr flag and inform application
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);

}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	//Txlen is zero, close the spi communication and inform application that Tx is over
	pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	//reception is complete, turn of the rxneie interrupt
	pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}


void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
	uint8_t temp1,temp2;

	//check for TXE
	temp1 = pSPIHandle->pSPIx->SR & (1<<SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1<<SPI_CR2_TXEIE);

	if(temp1 && temp2){
		//handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}
	//check for RXE
	temp1 = pSPIHandle->pSPIx->SR & (1<<SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1<<SPI_CR2_RXNEIE);

	if(temp1 && temp2){
		//handle RXE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2){
		//handle overrun error
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}
}

__attribute__((weak))void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv){
	//weak implementation. user must override with own implementation
}

