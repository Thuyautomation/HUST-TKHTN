/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "lcd.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t Count;
uint8_t RH1, RH2, TC1, TC2, SUM, CHECK;
uint32_t pMillis, cMillis;
float tCelsius = 0;
float tFahrenheit = 0;
float RH = 0;

uint8_t rx_data; 
char rx_buffer[100]; 
uint8_t rx_index = 0; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

volatile uint32_t tick = 0;  // Global tick counter

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_11
typedef enum {
    STATE_OFF,
    STATE_ON,
    STATE_START,
    STATE_RESTART,
    STATE_STOP,
} SystemState;
SystemState systemState = STATE_OFF;
typedef enum {
    DISPLAY_CELSIUS, // Hi?n th? nhi?t d? C
    DISPLAY_FAHRENHEIT // Hi?n th? nhi?t d? F
} DisplayMode;

DisplayMode displayMode = DISPLAY_CELSIUS;


int intPart;           
int firstDigit;        
int secondDigit;       
int decimalPart;      
int thirdDigit;       
int fourthDigit; 

uint8_t led_segments[] = {
    0xC0,  // 0
    0xF9,  // 1
    0xA4,  // 2
    0xB0,  // 3
    0x99,  // 4
    0x92,  // 5
    0x82,  // 6
    0xF8,  // 7
    0x80,  // 8
    0x90   // 9
};

void floatToString(float value, char *str)
{
    sprintf(str, "%.2f", value);  
}

void microDelay (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  while (__HAL_TIM_GET_COUNTER(&htim4) < delay);
}


uint8_t DHT11_Start (void)
{
  uint8_t Response = 0;
  GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
  GPIO_InitStructPrivate.Pin = DHT11_PIN;
  GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as output
  HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);   // pull the pin low
  microDelay (20000);   // wait for 20000us
  HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);   // pull the pin high
  microDelay (30);   // wait for 30us
  GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as input
  microDelay (40);
  if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
  {
    microDelay (80);
    if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1;
  }
  pMillis = HAL_GetTick();
  cMillis = HAL_GetTick();
  while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
  {
    cMillis = HAL_GetTick();
  }
  return Response;
}

uint8_t DHT11_Read (void)
{
  uint8_t a,b;
  for (a=0;a<8;a++)
  {
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go high
      cMillis = HAL_GetTick();
    }
    microDelay (40);   // wait for 40 us
    if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))   // if the pin is low
      b&= ~(1<<(7-a));
    else
      b|= (1<<(7-a));
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go low
      cMillis = HAL_GetTick();
    }
  }
  return b;
}

void displayTemperatureOnLCD(Lcd_HandleTypeDef *lcd, float temperatureC, float temperatureF, float humidity) {
    char tempStr[20], humStr[20];
    if (displayMode == DISPLAY_CELSIUS) {
        sprintf(tempStr, "%.2f\xDF""C", temperatureC); 
    } else {
        sprintf(tempStr, "%.2f\xDF""F", temperatureF); 
    }
    sprintf(humStr, "%.2f%%", humidity);
    Lcd_cursor(lcd, 0, 9); 
    Lcd_string(lcd, tempStr); 
    Lcd_cursor(lcd, 1, 9); 
    Lcd_string(lcd, humStr); 
}

void sendDataViaUART(UART_HandleTypeDef *huart, float temperature, float humidity) {
    char charData[50];
    sprintf(charData, "Lan do: %d\r\n", Count++);
    HAL_UART_Transmit(huart, (uint8_t *)charData, strlen(charData), 1000);
    sprintf(charData, "Nhiet do: %.2f C\r\n", temperature);
    HAL_UART_Transmit(huart, (uint8_t *)charData, strlen(charData), 1000);
    sprintf(charData, "Do am: %.2f %%\r\n", humidity);
    HAL_UART_Transmit(huart, (uint8_t *)charData, strlen(charData), 1000);
}

void processCommand(char* command) {
    if (strncmp(command, "SET_DISPLAY ", 12) == 0) {    
        char* mode = command + 12; 
        if (strcmp(mode, "C") == 0) {
            displayMode = DISPLAY_CELSIUS;
            HAL_UART_Transmit(&huart1, (uint8_t*)"Display mode set to Celsius\r\n", 29, HAL_MAX_DELAY);
        } else if (strcmp(mode, "F") == 0) {
            displayMode = DISPLAY_FAHRENHEIT;
            HAL_UART_Transmit(&huart1, (uint8_t*)"Display mode set to Fahrenheit\r\n", 32, HAL_MAX_DELAY);
        } else {
            HAL_UART_Transmit(&huart1, (uint8_t*)"Invalid display mode\r\n", 22, HAL_MAX_DELAY);
        }
    } else if (strcmp(command, "ON") == 0) {
        systemState = STATE_ON;
        HAL_UART_Transmit(&huart1, (uint8_t*)"System is ON\r\n", 14, HAL_MAX_DELAY);
    } else if (strcmp(command, "OFF") == 0) {
        systemState = STATE_OFF;
        HAL_UART_Transmit(&huart1, (uint8_t*)"System is OFF\r\n", 15, HAL_MAX_DELAY);
    } else if (strcmp(command, "START") == 0) {
        if (systemState == STATE_OFF) {
            HAL_UART_Transmit(&huart1, (uint8_t*)"Cannot START, system is OFF\r\n", 29, HAL_MAX_DELAY);
        } else {
            systemState = STATE_START;
            HAL_UART_Transmit(&huart1, (uint8_t*)"System STARTED\r\n", 16, HAL_MAX_DELAY);
        }
    } else if (strcmp(command, "RESTART") == 0) {				
        systemState = STATE_RESTART;
        HAL_UART_Transmit(&huart1, (uint8_t*)"System RESTARTED\r\n", 18, HAL_MAX_DELAY);
    } else if (strcmp(command, "STOP") == 0) {
        if (systemState == STATE_OFF) {
            HAL_UART_Transmit(&huart1, (uint8_t*)"Cannot STOP, system is OFF\r\n", 28, HAL_MAX_DELAY);
        } else {
            systemState = STATE_STOP;
            HAL_UART_Transmit(&huart1, (uint8_t*)"System STOPPED\r\n", 16, HAL_MAX_DELAY);
        }
    }
		else {
        // L?nh không h?p l?
        HAL_UART_Transmit(&huart1, (uint8_t*)"Invalid command\r\n", 17, HAL_MAX_DELAY);
    }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        if (rx_data == '\r' || rx_data == '\n') {
            rx_buffer[rx_index] = '\0';
            processCommand(rx_buffer);
            rx_index = 0;
        } else if (rx_index < sizeof(rx_buffer) - 1) {
            rx_buffer[rx_index++] = rx_data;
        } else {        
            rx_index = 0;
        }

        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
}



void displayDigit(uint8_t digit, uint8_t position) {

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_SET);

 
    GPIOA->ODR = (GPIOA->ODR & 0xFF00) | led_segments[digit];

  
    switch (position) {
        case 1: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); break;
        case 2: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); break;
        case 3: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); break;
        case 4: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); break;
    }
}

void displayTemperatureOnLED(float temperature) {
    int intPart = (int)temperature;
    int decimalPart = (int)((temperature - intPart) * 100);

    uint8_t digit1 = intPart / 10;   
    uint8_t digit2 = intPart % 10;    
    uint8_t digit3 = decimalPart / 10; 
    uint8_t digit4 = decimalPart % 10; 

    static uint8_t currentPosition = 1; 

    switch (currentPosition) {
        case 1: displayDigit(digit1, 1); break;
        case 2: displayDigit(digit2, 2); break;
        case 3: displayDigit(digit3, 3); break;
        case 4: displayDigit(digit4, 4); break;
    }

 
    currentPosition++;
    if (currentPosition > 4) currentPosition = 1; 
}

void display_dot_on_D3(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); 
}

void clear_dot_on_D3(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); 
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) 
		{
        tick++;  // Increment the global tick counter every 1ms
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim4);
	
  Lcd_PortType ports[] = { GPIOB, GPIOB, GPIOB, GPIOB };
  
  Lcd_PinType pins[] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15}; // Lcd_PinType pins[] = {D4_Pin, D5_Pin, D6_Pin, D7_Pin};
  Lcd_HandleTypeDef lcd;
  // Lcd_create(ports, pins, RS_GPIO_Port, RS_Pin, EN_GPIO_Port, EN_Pin, LCD_4_BIT_MODE);
  lcd = Lcd_create(ports, pins, GPIOA, GPIO_PIN_12, GPIOA, GPIO_PIN_11, LCD_4_BIT_MODE);
  Lcd_cursor(&lcd, 0,0);
  Lcd_string(&lcd, "Nhiet do:");
	HAL_Delay(200);		
	Lcd_cursor(&lcd, 1,0);
  Lcd_string(&lcd, "Do am:");
	HAL_Delay(200);		

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
        switch (systemState) {
            case STATE_OFF:
                break;
            case STATE_ON:
               systemState = STATE_START;
                break;
            case STATE_START:
                if (DHT11_Start()) {
                    RH1 = DHT11_Read();
                    RH2 = DHT11_Read();
                    TC1 = DHT11_Read();
                    TC2 = DHT11_Read();
                    SUM = DHT11_Read();
                    CHECK = RH1 + RH2 + TC1 + TC2;
                    if (CHECK == SUM) {
                        tCelsius = (TC1 > 127) ? (float)TC2 / 10 * (-1) : (float)((TC1 << 8) | TC2) / 400;
                        tFahrenheit = tCelsius * 9 / 5 + 32;
                        RH = (float)((RH1 << 8) | RH2) / 200;
                    }
                }

                sendDataViaUART(&huart1, tCelsius, RH);
                displayTemperatureOnLCD(&lcd, tCelsius, tFahrenheit, RH);
                display_dot_on_D3();
                displayTemperatureOnLED(tCelsius);
                HAL_Delay(1000);
                break;
            case STATE_RESTART:
                Count = 1 ;
                systemState = STATE_START;
                HAL_UART_Transmit(&huart1, (uint8_t*)"System RESTARTED\r\n", 18, HAL_MAX_DELAY);
                break;
            case STATE_STOP:               
                HAL_UART_Transmit(&huart1, (uint8_t*)"System STOPPED\r\n", 16, HAL_MAX_DELAY);
								while (systemState == STATE_STOP) {
									microDelay(50);
								}
								break;
							}
    }
}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
 
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72 -1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF -1 ;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
	HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, a_Pin|b_Pin|c_Pin|d_Pin
                          |e_Pin|f_Pin|g_Pin|h_Pin
                          |EN_Pin|CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DHT11_Pin|D4_Pin|D5_Pin|D6_Pin
                          |D7_Pin|D1_Pin|D2_Pin|D3_Pin
                          |D4B6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : a_Pin b_Pin c_Pin d_Pin
                           e_Pin f_Pin g_Pin h_Pin
                           EN_Pin CS_Pin */
  GPIO_InitStruct.Pin = a_Pin|b_Pin|c_Pin|d_Pin
                          |e_Pin|f_Pin|g_Pin|h_Pin
                          |EN_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT11_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D4_Pin D5_Pin D6_Pin D7_Pin
                           D1_Pin D2_Pin D3_Pin D4B6_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D7_Pin
                          |D1_Pin|D2_Pin|D3_Pin|D4B6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
