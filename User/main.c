#include "debug.h"
#include "oled.h"
#include "fonts.h"

#define _TIM2_PSC   ((SystemCoreClock / 1000) - 1)
#define _TIM2_ARR   (1000 - 1)

#define ANALOG_PIN  GPIO_Pin_0
#define PUMP_PIN    GPIO_Pin_2
#define HUMIDITY    60
#define FLOOD_STEP  100
#define FLOOD_MAX   6000
#define DISPLAY_SEC 60

uint8_t displaySec = 0;
uint8_t displayBuffer[128];
uint8_t displayGraf[128];
extern const uint8_t font8x8[][8];

uint8_t humidity = 0;

/**
 * @brief Initialization
 */
void init() {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  
  // Port A
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitTypeDef GPIO_PortA = {0};
  GPIO_PortA.GPIO_Pin = ANALOG_PIN;
  GPIO_PortA.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_PortA);

  // Port C
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  
  GPIO_InitTypeDef GPIO_PortC = {0};
  GPIO_PortC.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
  GPIO_PortC.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_PortC.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_Init(GPIOC, &GPIO_PortC);

  // Port D
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

  GPIO_InitTypeDef GPIO_PortD = {0};
  GPIO_PortD.GPIO_Pin = PUMP_PIN;
  GPIO_PortD.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_PortD.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_Init(GPIOD, &GPIO_PortD);

  // I2C
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  RCC_ADCCLKConfig(RCC_PCLK2_Div12); // 2 MHz

  I2C_InitTypeDef I2C_Init1 = {0};
  I2C_Init1.I2C_ClockSpeed = OLED_I2C_SPEED;
  I2C_Init1.I2C_Mode = I2C_Mode_I2C;
  I2C_Init1.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_Init1.I2C_OwnAddress1 = 0x00;
  I2C_Init1.I2C_Ack = I2C_Ack_Enable;
  I2C_Init1.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(I2C1, &I2C_Init1);

  I2C_Cmd(I2C1, ENABLE);

  // ADC
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  ADC_DeInit(ADC1);

  ADC_InitTypeDef ADC_Init1 = {0};
  ADC_Init1.ADC_Mode = ADC_Mode_Independent;
  ADC_Init1.ADC_ScanConvMode = DISABLE;
  ADC_Init1.ADC_ContinuousConvMode = DISABLE;
  ADC_Init1.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_Init1.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_Init1.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_Init1);

  ADC_Calibration_Vol(ADC1, ADC_CALVOL_50PERCENT);
  ADC_Cmd(ADC1, ENABLE);

  ADC_ResetCalibration(ADC1);
  while (ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1);
  while (ADC_GetCalibrationStatus(ADC1));

  // TIM2
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
  TIM_TimeBaseInitTypeDef TIMBase_InitStruct2 = {0};
  TIMBase_InitStruct2.TIM_ClockDivision = TIM_CKD_DIV1;
  TIMBase_InitStruct2.TIM_Prescaler = _TIM2_PSC;
  TIMBase_InitStruct2.TIM_Period = _TIM2_ARR;
  TIMBase_InitStruct2.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIMBase_InitStruct2);

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

  NVIC_InitTypeDef NVIC_InitStruct2 = {0};
  NVIC_InitStruct2.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStruct2.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct2.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStruct2.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct2);
  
  TIM_Cmd(TIM2, ENABLE);
}

/**
 * @brief Get the Adc Value
 * 
 * @param channel 
 * @return uint16_t 
 */
uint16_t getAdcValue(uint8_t channel) {
  ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_241Cycles);
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
  return ADC_GetConversionValue(ADC1);
}

/**
 * IRQ Timer 2
 */
void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_IRQHandler(void) {
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    if (displaySec < DISPLAY_SEC) {
      displaySec++;
    } else {
      for (uint8_t p = 0; p < 127; p++) {
        displayGraf[p + 1] = displayGraf[p];
      }
      displayGraf[0] = humidity;
    }
    
    
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 
  }
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
  oledInit();

  uint8_t symbol = ' ';

  for (uint8_t page = 0; page < 4; page++) {
		for (uint8_t column = 0; column < 16; column++) {
      //symbol++;
      uint8_t b = symbol - 0x20;
      for (uint8_t p = 0; p < 8; p++) {
        displayBuffer[column * 8 + p] = font8x8[b][p];
      }		
		}
    oledWriteData(page, displayBuffer, sizeof(displayBuffer));
	}

  char buff[17];
  uint16_t flood = 0;

  while (1) {
    GPIO_WriteBit(GPIOD, PUMP_PIN, Bit_RESET);

    uint16_t adcValue = getAdcValue(ADC_Channel_0);
    humidity = (1023 - adcValue) / 10;
    sprintf(buff, "H: %d.", humidity);// F: %4i", humidity, flood);
    print(buff, displayBuffer);
    oledWriteData(3, displayBuffer, sizeof(displayBuffer));

    if (humidity < HUMIDITY && flood < FLOOD_MAX) {
      GPIO_WriteBit(GPIOD, PUMP_PIN, Bit_SET);
      flood = flood + FLOOD_STEP;
    }

    Delay_Ms(500);
    if (flood > 0) {
      flood--;
    }
  }
}
