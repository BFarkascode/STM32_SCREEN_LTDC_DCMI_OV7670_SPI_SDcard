/*
 *  Created on: Aug 9, 2024
 *  Author: BalazsFarkas
 *  Project: STM32_SCREEN_LTDC_DCMI_SDCard
 *  Processor: STM32F429ZI
 *  Program version: 1.0
 *  Source file: SDcard_image_capture.c
 *  Change history:
 */


#include "SDcard_image_capture.h"

FATFS fs;             //file system
FIL fil;              //file
FILINFO filinfo;
FRESULT fresult;
char char_buffer[100];
uint8_t hex_buffer[100];

UINT br, bw;

FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

void SDcard_init(void){

	  fresult = f_mount(&fs, "", 0);                    //mount card
	  if (fresult != FR_OK) printf("Card mounting failed... \r\n");
	  else ("Card mounted... \r\n");

	  //--Capacity check--//
	  f_getfree("", &fre_clust, &pfs);

	  total = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	  printf("SD card total size is: \r\n");
	  printf("%d",total);

	  free_space = (uint32_t) (fre_clust * pfs->csize * 0.5);
	  printf("SD card free space is: \r\n");
	  printf("%d",free_space);

}


void SDcard_bmp_create(void){

	  char file_name[] = "000.bmp";							//we start from 0000 and then step through until we have

	  uint8_t file_cnt = 0;

	  fresult = f_stat(file_name, &filinfo);					//we check if the file exists already
	  while(fresult != FR_NO_FILE){

		  file_cnt++;
		  file_name[2] = file_cnt%10 + '0';
		  file_name[1] = (file_cnt%100)/10 + '0';
		  file_name[0] = file_cnt/100 + '0';
		  fresult = f_stat(file_name, &filinfo);					//we check if the file exists already

	  }

	  fresult = f_stat(file_name, &filinfo);					//we check if the file exists already
	  if(fresult == FR_NO_FILE){								//if it does not exist

		  f_open(&fil, file_name, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		  hex_buffer[0] = 0x42;			// Windows BMP signature
		  hex_buffer[1] = 0x4D;

		  hex_buffer[2] = 0x46;			// File size in bytes - update accordingly
		  hex_buffer[3] = 0xC2;
		  hex_buffer[4] = 0x01;
		  hex_buffer[5] = 0x00;

		  hex_buffer[6] = 0x00;			// Creator bytes (ignored)
		  hex_buffer[7] = 0x00;
		  hex_buffer[8] = 0x00;
		  hex_buffer[9] = 0x00;

		  hex_buffer[10] = 0x46;		// Offset to pixel data - update accordingly
		  hex_buffer[11] = 0x00;
		  hex_buffer[12] = 0x00;
		  hex_buffer[13] = 0x00;

		  hex_buffer[14] = 0x38;		// Header size in bytes - update accordingly
		  hex_buffer[15] = 0x00;
		  hex_buffer[16] = 0x00;
		  hex_buffer[17] = 0x00;

		  hex_buffer[18] = 0xF0;		// Width in pixels
		  hex_buffer[19] = 0x00;
		  hex_buffer[20] = 0x00;
		  hex_buffer[21] = 0x00;

		  hex_buffer[22] = 0xF0;		// Height in pixels
		  hex_buffer[23] = 0x00;
		  hex_buffer[24] = 0x00;
		  hex_buffer[25] = 0x00;


		  hex_buffer[26] = 0x01;		// Planes = 1
		  hex_buffer[27] = 0x00;

		  hex_buffer[28] = 0x10;		// Bits = 16
		  hex_buffer[29] = 0x00;

		  hex_buffer[30] = 0x03;		// Compression = bitfields
		  hex_buffer[31] = 0x00;
		  hex_buffer[32] = 0x00;
		  hex_buffer[33] = 0x00;

		  hex_buffer[34] = 0x00;		// Bitmap size
		  hex_buffer[35] = 0xE1;
		  hex_buffer[36] = 0x00;
		  hex_buffer[37] = 0x00;

		  hex_buffer[38] = 0x13;  		// Horiz resolution (72dpi)
		  hex_buffer[39] = 0x0B;
		  hex_buffer[40] = 0x00;
		  hex_buffer[41] = 0x00;

		  hex_buffer[42] = 0x13;		// Vert resolution (72dpi)
		  hex_buffer[43] = 0x0B;
		  hex_buffer[44] = 0x00;
		  hex_buffer[45] = 0x00;

		  hex_buffer[46] = 0x00;		 // Default # colors in palette
		  hex_buffer[47] = 0x00;
		  hex_buffer[48] = 0x00;
		  hex_buffer[49] = 0x00;

		  hex_buffer[50] = 0x00;		// Default # "important" colors
		  hex_buffer[51] = 0x00;
		  hex_buffer[52] = 0x00;
		  hex_buffer[53] = 0x00;

		  hex_buffer[54] = 0x00;		// Red mask
		  hex_buffer[55] = 0xF8;
		  hex_buffer[56] = 0x00;
		  hex_buffer[57] = 0x00;

		  hex_buffer[58] = 0xE0;		// Green mask
		  hex_buffer[59] = 0x07;
		  hex_buffer[60] = 0x00;
		  hex_buffer[61] = 0x00;

		  hex_buffer[62] = 0x1F;		// Blue mask
		  hex_buffer[63] = 0x00;
		  hex_buffer[64] = 0x00;
		  hex_buffer[65] = 0x00;

		  hex_buffer[66] = 0x00;		// Alpha mask
		  hex_buffer[67] = 0x00;
		  hex_buffer[68] = 0x00;
		  hex_buffer[69] = 0x00;


		  uint8_t temp = bufsize(hex_buffer);
		  f_write(&fil, hex_buffer, 70, &bw);
		  printf("File is created...\r\n");
		  bufclear();

		  f_write(&fil, image_read_ptr, 115200, &bw);
		  f_close(&fil);
	  }
}


int bufsize (char *buf)
{

  int i= 0;
  while(*buf++ != '\0') i++;
  return i;

}

void bufclear (void)                       //wipe buffer
{

  for (int i = 0; i < 100; i++){

    char_buffer[i] = '\0';
    hex_buffer[i] = 0x00;

  }

}


void Push_button_Init(void){
	/*
	 * EXTI on the blue push button (PA0)
	 *
	 * */

	//1)
		//Note: below we use PA0 "blue push button" and thus, EXTI0
	RCC->AHB1ENR |=	(1<<0);						//PORTA clocking
	GPIOA->MODER &= ~(1<<0);					//PA0 input - EXTI0
	GPIOA->MODER &= ~(1<<1);					//PA0 input - EXTI0
												//we use push-pull
												//no pull resistor

	//2)
	RCC->APB2ENR |= (1<<14);					//SYSCFG enable is on the APB2 enable register

	//3)
	SYSCFG->EXTICR[0] &= ~(1111<<0);			//since we want to use PA0, we write b'0000 to the [3:0] position of the fourth element of the register array

	//4)
	EXTI->IMR |= (1<<0);						//EXTI0 IMR unmasked
												//we leave the request masked
	EXTI->RTSR &= ~(1<<0);						//we disable the rising edge
	EXTI->FTSR |= (1<<0);						//we enable the falling edge

	//5)
	NVIC_SetPriority(EXTI0_IRQn, 1);			//we set the interrupt priority as 1 so it will be lower than the already running DMA IRQ for the SPI
	NVIC_EnableIRQ(EXTI0_IRQn);
}

//2)We define the callback function
void EXTI0_IRQHandler (void) {
	/*
	 * Note: it is good practice to double check that which pins have activated the EXTI.
	 *
	 * 1)Check which pin activated the EXTI
	 * 2)Act according to the pin activated
	 * 3)Reset the EXTI
	 *
	 * */

	//1)
	if (EXTI->PR & (1<<0)) {

		//2)
		log_image_flag = 1;

		//3)
		EXTI->PR |= (1<<0);						//we reset the IRQ connected to the EXTI13 by writing to the pending bit
	}
}

