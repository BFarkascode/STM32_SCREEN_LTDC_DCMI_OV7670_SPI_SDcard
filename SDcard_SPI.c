/*
 *  Created on: Aug 7, 2024
 *  Author: BalazsFarkas
 *  Project: STM32_SCREEN_LTDC_DCMI_SDCard
 *  Processor: STM32F429ZI
 *  Program version: 1.0
 *  Source file: SDcard_SPI.c
 *  Change history:
 */

#include "SDcard_SPI.h"


//1)SERCOM SPI init at 4 MHz
void SPI3_w_o_DMA_4MHZ_init(void) {

	/*
	 * 1) Enable peripheries and GPIO clocks
	 * 2) GPIO setting for SPI3
	 * 3) SPI3 setup
	 * 4) DMA setup
	 *
	 * SPI3 is on PB13,PB14 and PB15 on the DISCO board.
	 * CS pin for the SDcard will be on PB12
	 *
	 */

	//1)
	RCC->APB1ENR |= (1<<15);													//enable SPI3 interface

	RCC->AHB1ENR |=	(1<<1);														//PORTB clocking
	RCC->AHB1ENR |=	(1<<2);														//PORTC clocking

	//2)
	GPIOC->MODER &= ~(1<<20);													//alternate function for PC10
	GPIOC->MODER |= (1<<21);													//alternate function for PC10
	GPIOC->MODER &= ~(1<<24);													//alternate function for PC12
	GPIOC->MODER |= (1<<25);													//alternate function for PC12
	GPIOB->MODER &= ~(1<<8);													//alternate function for PB4
	GPIOB->MODER |= (1<<9);														//alternate function for PB4


	GPIOC->AFR[1] |= (6<<8);													//high speed PC10 - AF6 - SCK
	GPIOC->AFR[1] |= (6<<16);													//high speed PC12 - AF6 - MOSI
	GPIOB->AFR[0] |= (6<<16);													//high speed PB4 - AF6 - MISO

	Delay_us(1);																//this delay is necessary, otherwise GPIO setup may freeze

	//3
	SPI3->CR1 |= (1<<2);														//Master mode
	SPI3->CR1 &= ~(1<<3);
	SPI3->CR1 &= ~(1<<4);
	SPI3->CR1 &= ~(1<<5);
	SPI3->CR1 |= (2<<3);														//baud rate by diving the APB clock

	SPI3->CR1 |= (1<<9);														//SSM mode is software
	SPI3->CR1 |= (1<<8);														//we put a HIGH value on the SSI
	SPI3->CR1 &= ~(1<<3);														//no prescale
	SPI3->CR2 &= ~(1<<4);														//Motorola mode

}


//2)SPI SD write
void SPI3_Master_SD_write(uint8_t *tx_buffer_ptr, uint16_t len) {

//1)We reset the flags

//	GPIOB->BRR |= (1<<12);													//we enable the slave before we send the clock. CS setup time is minimum 5 ns, so no need for delay afterwards when using 32 MHz main clock (clock period is 31 ns)
	SPI3->CR1 |= (1<<6);													//SPI enabled. SCK is enabled
	uint32_t buf_junk = SPI3->DR;											//we reset the RX flag

//2)We load the message into the SPI module

	while (len)
	{
		SPI3->DR = (volatile uint8_t) *tx_buffer_ptr++;						//we load the byte into the Tx buffer
		while(!((SPI3->SR & (1<<1)) == (1<<1)));							//wait for the TXE flag to go HIGH and indicate that the TX buffer has been transferred to the shift register completely
		while(!((SPI3->SR & (1<<0)) == (1<<0)));							//we wait for the RXNE flag to go HIGH, indicating that the command has been transferred successfully
		buf_junk = SPI3->DR;												//we reset the RX flag
		len--;
	}

	while(((SPI3->SR & (1<<0)) == (1<<0)));									//we check that the Rx buffer is indeed empty
	while(!((SPI3->SR & (1<<1)) == (1<<1)));								//wait until TXE is set, meaning that Tx buffer is also empty
	while((SPI3->SR & (1<<7)) == (1<<7));
	SPI3->CR1 &= ~(1<<6);													//disable SPI

}


//3)SPI SD read
void SPI3_Master_SD_read(uint8_t *rx_buffer_ptr, uint16_t len) {

//1)We reset the flags

	SPI3->CR1 |= (1<<6);													//SPI enabled. SCK is enabled
	uint32_t buf_junk = SPI3->DR;											//we reset the RX flag

//2)We load the message into the SPI module

	while (len) {
		SPI3->DR = 0xFF;													//we load a dummy command into the DR register
		while(!((SPI3->SR & (1<<0)) == (1<<0)));
		*rx_buffer_ptr = SPI3->DR;											//we dereference the pointer, thus we can give it a value
																			//we extract the received value and by proxy reset the RX flag
		rx_buffer_ptr++;													//we step our pointer within the receiving end
		len--;

	}

	while(((SPI3->SR & (1<<0)) == (1<<0)));									//we check that the Rx buffer is indeed empty
	while(!((SPI3->SR & (1<<1)) == (1<<1)));								//wait until TXE is set, meaning that Tx buffer is also empty
	while((SPI3->SR & (1<<7)) == (1<<7));
																			//Note: this might not be necessary here
	SPI3->CR1 &= ~(1<<6);													//disable SPI

}


//4)SERCOM SPI init at 400 kHz
void SPI3_w_o_DMA_400KHZ_init(void) {

	//1)
	RCC->APB1ENR |= (1<<15);													//enable SPI3 interface

	RCC->AHB1ENR |=	(1<<1);														//PORTB clocking
	RCC->AHB1ENR |=	(1<<2);														//PORTC clocking

	//2)
	GPIOC->MODER &= ~(1<<20);													//alternate function for PC10
	GPIOC->MODER |= (1<<21);													//alternate function for PC10
	GPIOC->MODER &= ~(1<<24);													//alternate function for PC12
	GPIOC->MODER |= (1<<25);													//alternate function for PC12
	GPIOB->MODER &= ~(1<<8);													//alternate function for PB4
	GPIOB->MODER |= (1<<9);														//alternate function for PB4


	GPIOC->AFR[1] |= (6<<8);													//high speed PC10 - AF6 - SCK
	GPIOC->AFR[1] |= (6<<16);													//high speed PC12 - AF6 - MOSI
	GPIOB->AFR[0] |= (6<<16);													//high speed PB4 - AF6 - MISO

	Delay_us(1);																//this delay is necessary, otherwise GPIO setup may freeze

	//CS pin
	GPIOB->MODER |= (1<<24);													//GPIO output for PB12
	GPIOB->MODER &= ~(1<<25);													//GPIO output for PB12


	Delay_us(1);																//this delay is necessary, otherwise GPIO setup may freeze


	//3
	SPI3->CR1 |= (1<<2);														//Master mode
	SPI3->CR1 |= (6<<3);														//baud rate by diving the APB clock
																				//we can't give exact baud rate
																				//we currently divide by 128 (should be 234 kHz if APB is 30 MHz)
	SPI3->CR1 |= (1<<9);														//SSM mode is software
	SPI3->CR1 |= (1<<8);														//we put a HIGH value on the SSI
	SPI3->CR1 &= ~(1<<3);														//no prescale
	SPI3->CR2 &= ~(1<<4);														//Motorola mode

	//4
	GPIOB->BSRR |= (1<<12);														//we enable the slave before we send the clock. CS setup time is minimum 5 ns, so no need for delay afterwards when using 32 MHz main clock (clock period is 31 ns)

}
