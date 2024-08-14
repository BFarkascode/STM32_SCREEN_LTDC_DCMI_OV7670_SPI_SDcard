/*
 *  Created on: Aug 9, 2024
 *  Author: BalazsFarkas
 *  Project: STM32_SCREEN_LTDC_DCMI_SDCard
 *  Processor: STM32F429ZI
 *  Program version: 1.0
 *  Header file: SDcard_image_capture.h
 *  Change history:
 */

#ifndef INC_SDCARD_IMAGE_CAPTURE_H_
#define INC_SDCARD_IMAGE_CAPTURE_H_

#include "SDcard_diskio.h"
#include "stdlib.h"
#include "ff.h"

//LOCAL CONSTANT

//LOCAL VARIABLE

//EXTERNAL VARIABLE

extern uint8_t* image_read_ptr;
extern uint8_t log_image_flag;

//FUNCTION PROTOTYPES

void SDcard_init(void);
void SDcard_bmp_create(void);
void Push_button_Init(void);

#endif /* INC_SDCARD_IMAGE_CAPTURE_H_ */
