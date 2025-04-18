#include "debug.h"
#include "oled.h"

/**
 * Write data to oled
 * 
 * @param data 
 * @param size 
 */
void oledWrite(uint8_t command, uint8_t *data, uint8_t size) {
  while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) != RESET) {
    //printf("I2C_FLAG_BUSY\r\n");
  }
  
  I2C_GenerateSTART(I2C1, ENABLE);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
    //printf("I2C_EVENT_MASTER_MODE_SELECT\r\n");
  }

  I2C_Send7bitAddress(I2C1, OLED_I2C_ADDRESS, I2C_Direction_Transmitter);
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
 * Write byte to oled
 * 
 * @param byte 
 */
void oledWriteCommand(uint8_t command) {
  oledWrite(0, &command, 1);
}

/**
 * Initialization
 */
void oledInit() {
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

  oledWriteCommand(SSD1306_DISPLAY_OFF);

	oledWriteCommand(SSD1306_SET_DISPLAY_CLOCK_DIV);
  oledWriteCommand(0x00);
  
  oledWriteCommand(SSD1306_SET_MULTIPLEX);
  oledWriteCommand(SSD1306_MULTIPLEX_128_32);

  oledWriteCommand(SSD1306_SET_DISPLAY_OFFSET);
  oledWriteCommand(0x00);

  oledWriteCommand(SSD1306_SET_START_LINE | 0x00);

  oledWriteCommand(SSD1306_CHARGE_PUMP);
  oledWriteCommand(0x14); // Enable Charge Pump

  oledWriteCommand(SSD1306_MEMORY_MODE);
  oledWriteCommand(0x00); // Horizontal addressing mode (A[1:0]=00b)
  
  oledWriteCommand(SSD1306_SEG_REMAP_YES);
	
  oledWriteCommand(SSD1306_COM_SCAN_DEC); // Flip?

	oledWriteCommand(SSD1306_SET_COM_PINS);
  oledWriteCommand(0x02); //for 128x32 0x02, for 128x64 0x12;

  oledWriteCommand(SSD1306_DEACTIVATE_SCROLL);

  oledWriteCommand(SSD1306_COLUMN_ADDR);
  oledWriteCommand(0x00);
  oledWriteCommand(0xFF);

  oledWriteCommand(SSD1306_PAGE_ADDR);
  oledWriteCommand(0x00);
  oledWriteCommand(0x07);

  oledWriteCommand(SSD1306_SET_CONTRAST);
  oledWriteCommand(0xCF);

  oledWriteCommand(SSD1306_SET_PRE_CHARGE);
  oledWriteCommand(0xF1);

  oledWriteCommand(SSD1306_SET_V_COM_DETECT);
  oledWriteCommand(0x40);
  
  oledWriteCommand(SSD1306_DISPLAY_ALLON_RESUME);

	oledWriteCommand(SSD1306_DISPLAY_ON);
}

void oledSetCursor(uint8_t x, uint8_t y) {
	// Y - 1 unit = 1 page (8 pixel rows)
	// X - 1 unit = 8 pixel columns

  oledWriteCommand(0x00 | (8 * x & 0x0F)); 		      // Set column lower address
  oledWriteCommand(0x10 | ((8 * x >> 4) & 0x0F));   // Set column higher address
	oledWriteCommand(0xB0 | y);                       // Set page address
}

void oledWriteData(uint8_t page, uint8_t *data, uint8_t size) {
	//oledWriteCommand(SSD1306_PAGE_ADDR);
	//oledWriteCommand(0x00);
	//oledWriteCommand(0x07);

  oledWriteCommand(SSD1306_SET_PAGE_START_ADDRESS | page);
	oledWriteCommand(SSD1306_SET_HIGHER_COLUMN_START_ADDRESS);
	oledWriteCommand(SSD1306_SET_LOWER_COLUMN_START_ADDRESS);

  oledWrite(SSD1306_SET_START_LINE, data, size);
}