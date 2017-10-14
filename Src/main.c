/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "defines.h"
#include "font4.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint32_t ticks_per_us;
const uint8_t display_modules = 6;
const uint8_t display_columns = 48;
//const uint8_t on_time = 20;
const uint16_t display_scroll_period = 15;

uint8_t display_data[48];/* = {
    0b11111111,
    0b10000001,
    0b10000001,
    0b10000001,
    0b10000001,
    0b10000001,
    0b10000001,
    0b11111111,

    0b11111111,
    0b11000011,
    0b10100101,
    0b10011001,
    0b10011001,
    0b10100101,
    0b11000011,
    0b11111111,

    0b11111111,
    0b10000001,
    0b10000001,
    0b10000001,
    0b10000001,
    0b10000001,
    0b10000001,
    0b11111111,

    0b11111111,
    0b11000011,
    0b10100101,
    0b10011001,
    0b10011001,
    0b10100101,
    0b11000011,
    0b11111111,
};*/

uint8_t display_column = 0;
uint16_t display_period_cnt = 0;  // incremented by 1 every period (1ms)
uint16_t display_scroll = 0;

char txt_buffer[100];
uint8_t txt_len;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void column_flush();
void column_write(uint8_t);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void udelay(uint32_t delay)
{
  uint32_t ticks_start = __HAL_TIM_GET_COUNTER(&htim2);
  uint32_t ticks_delay = delay * ticks_per_us;
  while (__HAL_TIM_GET_COUNTER(&htim2) - ticks_start < ticks_delay) ;
}

void column_off(uint8_t depth)
{
  for (uint8_t i = 0; i < depth; i++)
    column_write(0);
  column_flush();
}

void column_write(uint8_t dat)
{
  // shift out column data (CDI, CCK)
  for (uint8_t i = 0; i < 8; i++) {
    HAL_GPIO_WritePin(CDI_PORT, CDI_PIN, dat & (0b10000000 >> i) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    //udelay(1);
    HAL_GPIO_WritePin(CCK_PORT, CCK_PIN, GPIO_PIN_SET);
    //udelay(1);
    HAL_GPIO_WritePin(CCK_PORT, CCK_PIN, GPIO_PIN_RESET);
    //udelay(1);
  }
}

void column_flush()
{
  // set column latch (CLE)
  HAL_GPIO_WritePin(CLE_PORT, CLE_PIN, GPIO_PIN_SET);
  //udelay(2);
  HAL_GPIO_WritePin(CLE_PORT, CLE_PIN, GPIO_PIN_RESET);
  //udelay(2);
}

void select_column(uint8_t col)
{
  HAL_GPIO_WritePin(COL_A_PORT, COL_A_PIN, col & 0b001 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(COL_B_PORT, COL_B_PIN, col & 0b010 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(COL_C_PORT, COL_C_PIN, col & 0b100 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/*
void display_flash()
{
  for (uint8_t row = 0; row < 8; row++) {
    select_column(row);
    column_write(display_data[row + 24]);
    column_write(display_data[row + 16]);
    column_write(display_data[row + 8]);
    column_write(display_data[row]);
    column_flush();
    udelay(on_time);
    column_off(4);
  }
}
*/

void display_clear()
{
  for (uint8_t col = 0; col < display_columns; col++) {
    display_data[col] = 0;
  }
}

void display_write_text(const char *text)
{
  uint8_t disp_col = 0;
  for (; *text != '\0'; text++) {
    uint16_t char_base = (*text - 32) * 5;

    for (uint8_t i = 0; i < 5; i++) {
      display_data[disp_col++] = font[char_base+i];
      if (disp_col > display_columns)
        return;
    }

    display_data[disp_col++] = 0;
    if (disp_col > display_columns)
      return;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim3) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);

    // flash current display column
    column_off(display_modules);
    select_column(display_column);
    for (int16_t i_mod = display_modules - 1; i_mod >= 0; i_mod--) {
      column_write(display_data[display_column + 8 * i_mod]);
    }
    column_flush();

    display_column++;
    if (display_column >= 8)
      display_column = 0;

    // after cycling through all columns (every 8ms)
    if (display_column == 0) {
      display_period_cnt++;

      // scroll every display_scroll_period ms
      if (display_period_cnt >= display_scroll_period) {
        display_scroll++;
        if (txt_len * 6 < display_columns || display_scroll >= txt_len * 6)
          display_scroll = 0;
        display_period_cnt = 0;
      }

      // TODO: update display from text after last column
      // TODO: sync??
      uint16_t i_char = display_scroll / 6;
      uint16_t char_base = (txt_buffer[i_char] - 32) * 5;
      uint16_t char_col = display_scroll % 6;
      for (uint8_t display_col = 0; display_col < display_columns; display_col++) {
        display_data[display_col] = char_col < 5 ? font[char_base + char_col] : 0x00;

        char_col++;
        if (char_col > 5) {
          i_char++;
          if (i_char >= txt_len) {
            i_char = 0;
          }
          char_base = (txt_buffer[i_char] - 32) * 5;
          char_col = 0;
        }
      }
    }
  }

  __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */

  ticks_per_us = (HAL_RCC_GetSysClockFreq() / 1000000) / (htim2.Init.Prescaler + 1);
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);



  uint32_t counter = 0;
  //uint8_t bar_pos = 0;
  //uint8_t moving_right = 1;

  //char txt_buf[32];

  /*
  HAL_GPIO_WritePin(CLE_PORT, CLE_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RI_A_PORT, RI_A_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RI_B_PORT, RI_B_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RI_C_PORT, RI_C_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CDI_PORT, CDI_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CCK_PORT, CCK_PIN, GPIO_PIN_RESET);
  */
  //display_clear();

  txt_len = sprintf(txt_buffer, "Freiburg: 18°C Berlin: 15°C Feldberg 10°C ");

  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

    /*
    ++counter;
    if (counter > 300 && counter % 5 == 0) {

      for (uint8_t col = 0; col < 16; col++) {
        if (col == bar_pos)
          display_data[col] = 0xff;
        else
          display_data[col] = 0x00;
      }

      if (moving_right)
        bar_pos++;
      else
        bar_pos--;

      if (bar_pos == 0 || bar_pos == 15)
        moving_right = !moving_right;

    }
    */

    //column_write(counter);
    //column_write(counter);
    //column_flush();

    // TODO: set row (RI_A, RI_B, RI_C)
    //select_column(counter);

    //counter++;

    //int written = sprintf(txt_buffer, "%lu ", ++counter);
    //txt_len = written > 0 ? written : 0;

    //display_write_text(txt_buffer);

    //display_flash();

    //HAL_Delay(100);

    /* Blinking LED:
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    //HAL_Delay(500);
    udelay(50000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    //HAL_Delay(500);
    udelay(50000);
    /**/
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 31;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 31999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_7B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB12 PB13 PB14 
                           PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_Delay(50);
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
