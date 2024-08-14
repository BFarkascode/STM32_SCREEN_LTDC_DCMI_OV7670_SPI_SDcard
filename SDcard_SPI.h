/*
 *  Created on: Aug 7, 2024
 *  Author: BalazsFarkas
 *  Project: STM32_SCREEN_LTDC_DCMI_SDCard
 *  Processor: STM32F429ZI
 *  Program version: 1.0
 *  Header file: SDcard_SPI.h
 *  Change history:
 */

#ifndef INC_SDCARD_SPI_H_
#define INC_SDCARD_SPI_H_

#include "stdint.h"
#include "stm32f429xx.h"
#include "ClockDriver_STM32F4xx.h"

//LOCAL CONSTANT

//LOCAL VARIABLE

//EXTERNAL VARIABLE

//FUNCTION PROTOTYPES
void SPI3_w_o_DMA_400KHZ_init(void);
void SPI3_w_o_DMA_4MHZ_init(void);
void SPI3_Master_SD_write(uint8_t *tx_buffer_ptr, uint16_t len);
void SPI3_Master_SD_read(uint8_t *rx_buffer_ptr, uint16_t len);

#endif /* INC_SDCARD_SPI_H_ */
