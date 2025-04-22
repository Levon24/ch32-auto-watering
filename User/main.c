#include "debug.h"
#include "oled.h"
#include "fonts.h"
#include <stdlib.h>
#include <string.h>

#define ANALOG_PIN  GPIO_Pin_0
#define PUMP_PIN    GPIO_Pin_2
#define HUMIDITY    60
#define TIMES       6000

uint8_t displayBuffer[128];
extern const uint8_t font8x8[][8];

/**
 * Initialization
 */
void init() {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div12); // 2 MHz

    // Ports
    GPIO_InitTypeDef GPIO_PortA = {0};
    GPIO_PortA.GPIO_Pin = ANALOG_PIN;
    GPIO_PortA.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_PortA);
    
    GPIO_InitTypeDef GPIO_PortC = {0};
    GPIO_PortC.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_PortC.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_PortC.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOC, &GPIO_PortC);

    GPIO_InitTypeDef GPIO_PortD = {0};
    GPIO_PortD.GPIO_Pin = PUMP_PIN;
    GPIO_PortD.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_PortD.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOD, &GPIO_PortD);

    // I2C
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
}

uint16_t getAdcValue(uint8_t channel) {
  ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_241Cycles);
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
  return ADC_GetConversionValue(ADC1);
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
  //Delay_Ms(100);

  oledInit();
  //oledSetCursor(0, 0);

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

  char val[5];
  char buff[17];
  uint16_t times = 0;

  while (1) {
    GPIO_WriteBit(GPIOD, PUMP_PIN, Bit_RESET);

    uint16_t vref = getAdcValue(ADC_Channel_Vrefint);
    itoa(vref, val, 10);
    strcpy(buff, "Vref: ");
    strcat(buff, val);
    clear(displayBuffer, sizeof(displayBuffer));
    print(buff, displayBuffer);
    oledWriteData(0, displayBuffer, sizeof(displayBuffer));

    uint16_t vint = getAdcValue(ADC_Channel_Vcalint);
    itoa(vint, val, 10);
    strcpy(buff, "Vint: ");
    strcat(buff, val);
    clear(displayBuffer, sizeof(displayBuffer));
    print(buff, displayBuffer);
    oledWriteData(1, displayBuffer, sizeof(displayBuffer));

    uint16_t soil = getAdcValue(ADC_Channel_0);
    uint8_t humidity = (1023 - soil) / 10;
    itoa(humidity, val, 10);
    strcpy(buff, "Humidity: ");
    strcat(buff, val);
    strcat(buff, "%");
    clear(displayBuffer, sizeof(displayBuffer));
    print(buff, displayBuffer);
    oledWriteData(2, displayBuffer, sizeof(displayBuffer));

    itoa(times, val, 10);
    strcpy(buff, "Times: ");
    strcat(buff, val);
    clear(displayBuffer, sizeof(displayBuffer));
    print(buff, displayBuffer);
    oledWriteData(3, displayBuffer, sizeof(displayBuffer));

    if (humidity < HUMIDITY && times < TIMES) {
      GPIO_WriteBit(GPIOD, PUMP_PIN, Bit_SET);
      times = times + 100;
    }

    Delay_Ms(500);
    if (times > 0) {
      times--;
    }
  }
}
