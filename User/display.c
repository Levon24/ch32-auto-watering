#include <stdio.h>
#include <ch32v00x.h>
#include "display.h"

/**
 * Send data to display
 * 
 * @param data 
 * @param size 
 */
void displaySend(uint8_t command, uint8_t *data, uint8_t size) {
  while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) != RESET) {
    //printf("I2C_FLAG_BUSY\r\n");
  }
  
  I2C_GenerateSTART(I2C1, ENABLE);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
    //printf("I2C_EVENT_MASTER_MODE_SELECT\r\n");
  }

  I2C_Send7bitAddress(I2C1, DISPLAY_I2C_ADDRESS, I2C_Direction_Transmitter);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
    //printf("I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED\r\n");
  }

  I2C_SendData(I2C1, command);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
    //printf("I2C_EVENT_MASTER_BYTE_TRANSMITTED\r\n");
  }

  for (uint8_t p = 0; p < size; p++) {
    I2C_SendData(I2C1, *data);
    data++;

    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
      //printf("I2C_EVENT_MASTER_BYTE_TRANSMITTED\r\n");
    }
  }
  
  I2C_GenerateSTOP(I2C1, ENABLE);
}

/**
 * Send byte to display
 * 
 * @param byte 
 */
void displaySendCommand(uint8_t command) {
  displaySend(0, &command, 1);
}

/**
 * Initialization
 */
void displayInit() {
  // Set MUX Ratio A8h, 3Fh
  // Set Display Offset D3h, 00h
  // Set Display Start Line 40h
  // Set Segment re-map A0h/A1h
  // Set COM Output Scan Direction C0h/C8h
  // Set COM Pins hardware configuration DAh, 02
  // Set Contrast Control 81h, 7Fh
  // Disable Entire Display On A4h
  // Set Normal Display A6h
  // Set Osc Frequency D5h, 80h
  // Enable charge pump regulator 8Dh, 14h
  // Display On AFh

  Delay_Ms(100);

  displaySendCommand(SSD1306_DISPLAY_OFF);

	displaySendCommand(SSD1306_SET_DISPLAY_CLOCK_DIV);
  displaySendCommand(0x00);
  
  displaySendCommand(SSD1306_SET_MULTIPLEX);
  displaySendCommand(SSD1306_MULTIPLEX_128_32);

  displaySendCommand(SSD1306_SET_DISPLAY_OFFSET);
  displaySendCommand(0x00);

  displaySendCommand(SSD1306_SET_START_LINE | 0x00);

  displaySendCommand(SSD1306_CHARGE_PUMP);
  displaySendCommand(0x14); // Enable Charge Pump

  displaySendCommand(SSD1306_MEMORY_MODE);
  displaySendCommand(0x00); // Horizontal addressing mode (A[1:0]=00b)
  
  displaySendCommand(SSD1306_SEG_REMAP_YES);
	
  displaySendCommand(SSD1306_COM_SCAN_DEC); // Flip?

	displaySendCommand(SSD1306_SET_COM_PINS);
  displaySendCommand(0x02); //for 128x32 0x02, for 128x64 0x12;

  displaySendCommand(SSD1306_DEACTIVATE_SCROLL);

  displaySendCommand(SSD1306_COLUMN_ADDR);
  displaySendCommand(0x00);
  displaySendCommand(0xFF);

  displaySendCommand(SSD1306_PAGE_ADDR);
  displaySendCommand(0x00);
  displaySendCommand(0x07);

	displaySendCommand(SSD1306_SET_CONTRAST);
	displaySendCommand(DISPLAY_DEFAULT_CONTRAST);

	//displaySendCommand(SSD1306_SET_PRE_CHARGE);
	//displaySendCommand(0xF1);

	displaySendCommand(SSD1306_SET_V_COM_DETECT);
	displaySendCommand(0x40);

	displaySendCommand(SSD1306_DISPLAY_ALLON_RESUME);

	displaySendCommand(SSD1306_DISPLAY_ON);
}

void displaySetCursor(uint8_t x, uint8_t y) {
	// Y - 1 unit = 1 page (8 pixel rows)
	// X - 1 unit = 8 pixel columns

	displaySendCommand(0x00 | (8 * x & 0x0F)); 		      // Set column lower address
	displaySendCommand(0x10 | ((8 * x >> 4) & 0x0F));   // Set column higher address
	displaySendCommand(0xB0 | y);                       // Set page address
}

void displaySendData(uint8_t page, uint8_t *data, uint8_t size) {
	//displaySendCommand(SSD1306_PAGE_ADDR);
	//displaySendCommand(0x00);
	//displaySendCommand(0x07);

	displaySendCommand(SSD1306_SET_PAGE_START_ADDRESS | page);
	displaySendCommand(SSD1306_SET_HIGHER_COLUMN_START_ADDRESS);
	displaySendCommand(SSD1306_SET_LOWER_COLUMN_START_ADDRESS);

	displaySend(SSD1306_SET_START_LINE, data, size);
}

void displaySetContrast(uint8_t value) {
	displaySendCommand(SSD1306_SET_CONTRAST);
	displaySendCommand(value);
}