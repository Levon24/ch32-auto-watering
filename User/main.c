#include "debug.h"
#include "display.h"
#include "fonts.h"

#define _TIM2_PSC   ((SystemCoreClock / 1000) - 1)
#define _TIM2_ARR   (1000 - 1)

#define SENSOR_CAP      1   // Емкостной датчик?
#define SENSOR_PIN      GPIO_Pin_0
#define PUMP_PIN        GPIO_Pin_2
#define MOISTURE        80  // Цель для влажности почвы
#define DELAY_MS        125 // Задержка в основном цикле
#define FLOOD_STEP      4   // Сколько за шаг полива добавлять в счетчик потопа
#define FLOOD_MAX       (30 * FLOOD_STEP * 1000) / DELAY_MS // 30 секунд максимум лить воду до потопа
#define CHART_SECONDS   5  // Через сколько секунд график сдвигать вправо
#define CHART_SIZE      8 * 3
#define CHART_MAX       10
#define U_VREF          1.2 // 1.2V 
#define U_VCC           3.3 // 3.3V
#define U_DRY           2.78 // Калибровка для емкостного датчика 0 влажности (для v1.2 = 2.193, для v2.0 = 2.752)
#define U_WET           1.11 // Калибровка для емкостного датчика 100 влажности (для v1.2 = 1.06, для v2.0 = 1.304)

#define BUTTON_SETTINGS GPIO_Pin_3
#define BUTTON_NEXT     GPIO_Pin_4
#define BUTTON_UP       GPIO_Pin_5
#define BUTTON_DOWN     GPIO_Pin_6
#define BUTTON_PRESSED  0

uint8_t seconds = 0;
uint8_t displayLine[128];
uint8_t chartValues[128];
enum _state state = show_chart;
enum _setup setup = setup_min_moisture; 
uint16_t flood = 0;

extern const uint8_t font8x8[][8];
const uint8_t levels[] = {0b00000000, 0b10000000, 0b11000000, 0b11100000, 0b11110000, 0b11111000, 0b11111100, 0b11111110, 0b11111111};

uint8_t contrast = DISPLAY_DEFAULT_CONTRAST;
uint8_t moisture = 0;
uint8_t minMoisture = MOISTURE;

/**
 * @brief Init Port A
 */
void initPortA() {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitTypeDef initTypeDef = {0};
  initTypeDef.GPIO_Pin = SENSOR_PIN;
  initTypeDef.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &initTypeDef);
}

/**
 * @brief Init Port C
 */
void initPortC() {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  
  GPIO_InitTypeDef initI2C = {0};
  initI2C.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
  initI2C.GPIO_Mode = GPIO_Mode_AF_OD;
  initI2C.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_Init(GPIOC, &initI2C);

  GPIO_InitTypeDef initButtons = {0};
  initButtons.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
  initButtons.GPIO_Mode = GPIO_Mode_IPU;
  initButtons.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_Init(GPIOC, &initButtons);
}

/**
 * @brief Init Port D
 */
void initPortD() {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

  GPIO_InitTypeDef initTypeDef = {0};
  initTypeDef.GPIO_Pin = PUMP_PIN;
  initTypeDef.GPIO_Mode = GPIO_Mode_Out_PP;
  initTypeDef.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_Init(GPIOD, &initTypeDef);
}

/**
 * @brief Init I2C
 */
void initI2C() {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

  I2C_InitTypeDef initTypeDef = {0};
  initTypeDef.I2C_ClockSpeed = DISPLAY_I2C_SPEED;
  initTypeDef.I2C_Mode = I2C_Mode_I2C;
  initTypeDef.I2C_DutyCycle = I2C_DutyCycle_2;
  initTypeDef.I2C_OwnAddress1 = 0x00;
  initTypeDef.I2C_Ack = I2C_Ack_Enable;
  initTypeDef.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(I2C1, &initTypeDef);

  I2C_Cmd(I2C1, ENABLE);
}

/**
 * @brief Init ADC
 */
void initADC() {
  RCC_ADCCLKConfig(RCC_PCLK2_Div12); // 2 MHz

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  ADC_DeInit(ADC1);

  ADC_InitTypeDef initTypeDef = {0};
  initTypeDef.ADC_Mode = ADC_Mode_Independent;
  initTypeDef.ADC_ScanConvMode = DISABLE;
  initTypeDef.ADC_ContinuousConvMode = DISABLE;
  initTypeDef.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  initTypeDef.ADC_DataAlign = ADC_DataAlign_Right;
  initTypeDef.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &initTypeDef);

  ADC_Calibration_Vol(ADC1, ADC_CALVOL_50PERCENT);
  ADC_Cmd(ADC1, ENABLE);

  ADC_ResetCalibration(ADC1);
  while (ADC_GetResetCalibrationStatus(ADC1));

  ADC_StartCalibration(ADC1);
  while (ADC_GetCalibrationStatus(ADC1));
}

/**
 * @brief Init TIM2
 */
void initTIM2() {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
  TIM_TimeBaseInitTypeDef timInitTypeDef = {0};
  timInitTypeDef.TIM_ClockDivision = TIM_CKD_DIV1;
  timInitTypeDef.TIM_Prescaler = _TIM2_PSC;
  timInitTypeDef.TIM_Period = _TIM2_ARR;
  timInitTypeDef.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &timInitTypeDef);

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

  NVIC_InitTypeDef nvicInitTypeDef = {0};
  nvicInitTypeDef.NVIC_IRQChannel = TIM2_IRQn;
  nvicInitTypeDef.NVIC_IRQChannelPreemptionPriority = 0;
  nvicInitTypeDef.NVIC_IRQChannelSubPriority = 1;
  nvicInitTypeDef.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicInitTypeDef);
  
  TIM_Cmd(TIM2, ENABLE);
}

/**
 * @brief Initialization
 */
void init() {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  
  initPortA();
  initPortC();
  initPortD();
  
  initI2C();
  initADC();

  initTIM2();
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
 * @brief IRQ Timer 2
 */
void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_IRQHandler(void) {
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    if (seconds < CHART_SECONDS) {
      seconds++;
    } else {
      seconds = 0;
      
      for (uint8_t p = 127; p > 0; p--) {
        chartValues[p] = chartValues[p - 1];
      }
      chartValues[0] = moisture;
    }
    
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 
  }
}

/**
 * @brief Read sensor & calc moisture
 */
void readMoisture() {
  // read sensor
  uint16_t adcValue = getAdcValue(ADC_Channel_0);
  //printf("ADC: %d\r\n", adcValue);

  // calculate moisture
#if (SENSOR_CAP == 1)
  uint16_t adcVref = getAdcValue(ADC_Channel_Vrefint);
  float u = ((float) U_VREF / adcVref) * adcValue;
  if (u > U_DRY) {
    moisture = 0;
  } else {
    moisture = ((U_DRY - u) * 100) / (U_DRY - U_WET);
  }
#else
  moisture = (1023 - adcValue) / 10;
#endif
}

/**
 * @brief pump control
 */
void turnPump() {
  if (moisture < minMoisture && flood < FLOOD_MAX) {
    GPIO_WriteBit(GPIOD, PUMP_PIN, Bit_SET);
    flood = flood + FLOOD_STEP;
  } else {
    GPIO_WriteBit(GPIOD, PUMP_PIN, Bit_RESET);
  }

  if (flood > 0) {
    flood--;
  }
}

/**
 * @brief display graf
 */
void showChart(uint16_t buttons) {
  char buff[17];
  
  for (uint8_t position = 0; position < 3; position++) {
    for (uint8_t p = 0; p < 128; p++) {
      uint8_t value = chartValues[p];

      uint8_t maxValue = minMoisture + CHART_MAX;
      if (value > maxValue) {
        value = maxValue;
      }

      uint8_t minValue = maxValue - CHART_SIZE;
      if (value < minValue) {
        value = minValue;
      }

      value = value - minValue;

      if (value > (2 - position) * 8) {
        uint8_t level = value - (2 - position) * 8;
        if (level > 8) {
          level = 8;
        }
        displayLine[p] = levels[level];
      } else {
        displayLine[p] = 0;
      }
    }
    displaySendData(position, displayLine, sizeof(displayLine));
  }

  sprintf(buff, "M: %3d%% F: %5d", moisture, flood);
  text(buff, displayLine);
  displaySendData(3, displayLine, sizeof(displayLine));

  if ((buttons & BUTTON_SETTINGS) == BUTTON_PRESSED) {
    state = show_settings;
  }
}

/**
 * @brief display settings
 */
void showSettings(uint16_t buttons) {
  char buff[17];

  text(" -= Settings =- ", displayLine);
  displaySendData(0, displayLine, sizeof(displayLine));

  sprintf(buff, "%s Moisture: %3d", (setup == setup_min_moisture ? ">" : " "), minMoisture);
  text(buff, displayLine);
  displaySendData(1, displayLine, sizeof(displayLine));

  sprintf(buff, "%s Contrast: %3d", (setup == setup_contrast ? ">" : " "), contrast);
  text(buff, displayLine);
  displaySendData(2, displayLine, sizeof(displayLine));

  clear(displayLine, sizeof(displayLine));
  displaySendData(3, displayLine, sizeof(displayLine));

  if ((buttons & BUTTON_SETTINGS) == BUTTON_PRESSED) {
    state = show_chart;
  }

  if ((buttons & BUTTON_NEXT) == BUTTON_PRESSED) {
    if (setup < setup_contrast) {
      setup++;
    } else {
      setup = setup_min_moisture;
    }
  }

  if ((buttons & BUTTON_UP) == BUTTON_PRESSED) {
    switch (setup) {
      case setup_min_moisture:
        if (minMoisture < 100) {
          minMoisture++;
        }
        break;

      case setup_contrast:
        if (contrast < 0xFF) {
          contrast++;
          displaySetContrast(contrast);
        }
        break;
    }
  }

  if ((buttons & BUTTON_DOWN) == BUTTON_PRESSED) {
    switch (setup) {
      case setup_min_moisture:
        if (minMoisture > 0) {
          minMoisture--;
        }
        break;

      case setup_contrast:
        if (contrast > 0) {
          contrast--;
          displaySetContrast(contrast);
        }
        break;
    }
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
  displayInit();

  while (1) {
    readMoisture();
    turnPump();

    uint16_t buttons = GPIO_ReadInputData(GPIOC);
    switch (state) {
      case show_chart:
        showChart(buttons);
        break;
      
      case show_settings:
        showSettings(buttons); 
        break;
    }
    
    Delay_Ms(DELAY_MS);
  }
}
