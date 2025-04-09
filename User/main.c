#include "debug.h"

#define I2C_CLOCK_SPEED   100000

/**
 * Init 
 */
void init() {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure={0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    I2C_InitTypeDef I2C_InitTSturcture={0};
    I2C_InitTSturcture.I2C_ClockSpeed = I2C_CLOCK_SPEED;
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
    I2C_InitTSturcture.I2C_OwnAddress1 = 0x00;
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitTSturcture);

    I2C_Cmd(I2C1, ENABLE);
}

uint8_t checkAddress(uint8_t address, uint8_t max) {
  uint8_t count = 0;
  
  while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) != RESET) {
    count++;
    if (count >= max) {
      return 0;
    }
  }

  I2C_GenerateSTART(I2C1, ENABLE);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
    count++;
    if (count >= max) {
      return 0;
    }
  }

  I2C_Send7bitAddress(I2C1, address, I2C_Direction_Transmitter);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
    count++;
    if (count >= max) {
      return 0;
    }
  }

  I2C_GenerateSTOP(I2C1, ENABLE);

  return 1;
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

  for (uint8_t address = 1; address < 128; address++) {
    uint8_t status = checkAddress(address, 255);
    printf("Address: %d -> Status: %d\r\n", address, status);
  }
}
