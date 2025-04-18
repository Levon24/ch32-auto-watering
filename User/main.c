#include "debug.h"
#include "oled.h"
#include "fonts.h"

uint8_t displayBuffer[128];
extern const uint8_t font8x8[][8];

/**
 * Initialization
 */
void init() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    
    GPIO_InitTypeDef GPIO_PortC = {0};
    GPIO_PortC.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_PortC.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_PortC.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOC, &GPIO_PortC);

    I2C_InitTypeDef I2C_I2C1 = {0};
    I2C_I2C1.I2C_ClockSpeed = OLED_I2C_SPEED;
    I2C_I2C1.I2C_Mode = I2C_Mode_I2C;
    I2C_I2C1.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_I2C1.I2C_OwnAddress1 = 0xEE;
    I2C_I2C1.I2C_Ack = I2C_Ack_Enable;
    I2C_I2C1.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_I2C1);

    I2C_Cmd(I2C1, ENABLE);
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void) {
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  SystemCoreClockUpdate();
  Delay_Init();
#if (SDI_PRINT == SDI_PR_OPEN)
  SDI_Printf_Enable();
#else
  USART_Printf_Init(115200);
#endif
  printf("SystemClk: %d\r\n", SystemCoreClock);
  printf("ChipID: %08x\r\n", DBGMCU_GetCHIPID());

  init();
  Delay_Ms(100);

  oledInit();
  //oledSetCursor(0, 0);

  uint8_t symbol = ' ';

  for (uint8_t page = 0; page < 4; page++) {
		for (uint8_t column = 0; column < 16; column++) {
      symbol++;
      for (uint8_t p = 0; p < 8; p++) {
        displayBuffer[column * 8 + p] = font8x8[symbol - 0x20][p];
      }		
		}
    oledWriteData(page, displayBuffer, sizeof(displayBuffer));

    Delay_Ms(1000);
	}

  while (1) {
  
  }
}
