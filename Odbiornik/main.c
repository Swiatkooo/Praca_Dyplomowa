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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#include <string.h>
#include <stdio.h>
#include "main.h"

// --- Definicje pinów nRF905 ---
#define NRF905_CE_PORT     GPIOB
#define NRF905_CE_PIN      GPIO_PIN_1
#define NRF905_TXEN_PORT   GPIOB
#define NRF905_TXEN_PIN    GPIO_PIN_14
#define NRF905_CSN_PORT    GPIOC
#define NRF905_CSN_PIN     GPIO_PIN_5
#define NRF905_PWR_PORT    GPIOB
#define NRF905_PWR_PIN     GPIO_PIN_15
#define NRF905_DR_PORT     GPIOB
#define NRF905_DR_PIN      GPIO_PIN_12

#define NRF905_CE_HIGH()    HAL_GPIO_WritePin(NRF905_CE_PORT, NRF905_CE_PIN, GPIO_PIN_SET)
#define NRF905_CE_LOW()     HAL_GPIO_WritePin(NRF905_CE_PORT, NRF905_CE_PIN, GPIO_PIN_RESET)
#define NRF905_TXEN_HIGH()  HAL_GPIO_WritePin(NRF905_TXEN_PORT, NRF905_TXEN_PIN, GPIO_PIN_SET)
#define NRF905_TXEN_LOW()   HAL_GPIO_WritePin(NRF905_TXEN_PORT, NRF905_TXEN_PIN, GPIO_PIN_RESET)
#define NRF905_CSN_HIGH()   HAL_GPIO_WritePin(NRF905_CSN_PORT, NRF905_CSN_PIN, GPIO_PIN_SET)
#define NRF905_CSN_LOW()    HAL_GPIO_WritePin(NRF905_CSN_PORT, NRF905_CSN_PIN, GPIO_PIN_RESET)

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

uint8_t rxBuffer[32];

typedef struct __attribute__((packed)) {
    float temperature;
    float humidity;
    float pressure;
    float mic;
    uint16_t tvoc;        // TVOC w ppb
    uint16_t eco2;        // eCO2 w ppm
    uint8_t aqi;          // Air Quality Index
    uint8_t ens160_status;// Status ENS160
} SensorPacket;


// --- UART helper ---
void UART_Print(const char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

// --- nRF905 --- //
void NRF905_Init(void)
{
    NRF905_CSN_HIGH();
    NRF905_TXEN_LOW();
    NRF905_CE_LOW();
    HAL_GPIO_WritePin(NRF905_PWR_PORT, NRF905_PWR_PIN, GPIO_PIN_SET);
    HAL_Delay(10);

    // Konfiguracja RX
    uint8_t config[10] = {
        0x6C,       // channel = 108
        0x0C,       // HFREQ_PLL=0 (433MHz), PA_PWR=+10dBm, RX_RED_PWR=0, AUTO_RETRAN=0
        0x44,       // address width = 4 bytes
        0x20,       // RX payload size = 32 bytes
        0x20,       // TX payload size = 32 bytes
        0xE7, 0xE7, 0xE7, 0xE7, // TX address
        0xDB        // CRC=16bit, CRC_EN=ON, XOF=12MHz, UP_CLK_EN=0
    };

    uint8_t cmd = 0x00; // W_CONFIG
    NRF905_CSN_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, config, sizeof(config), HAL_MAX_DELAY);
    NRF905_CSN_HIGH();

    uint8_t rx_addr[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    cmd = 0x21; // W_RX_ADDRESS
    NRF905_CSN_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, rx_addr, 4, HAL_MAX_DELAY);
    NRF905_CSN_HIGH();

    // Włącz RX
    NRF905_TXEN_LOW();
    NRF905_CE_HIGH();
    UART_Print("NRF905 RX init done\r\n");
}

uint8_t NRF905_ReadStatus(void)
{
    uint8_t cmd = 0x1F; // STATUS
    uint8_t status = 0;
    NRF905_CSN_LOW();
    HAL_SPI_TransmitReceive(&hspi1, &cmd, &status, 1, HAL_MAX_DELAY);
    NRF905_CSN_HIGH();
    return status;
}

void NRF905_ReceivePayload(void)
{
    uint8_t cmd = 0x24; // R_RX_PAYLOAD
    NRF905_CSN_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, rxBuffer, 32, HAL_MAX_DELAY);
    NRF905_CSN_HIGH();

    SensorPacket pkt;
    memcpy(&pkt, rxBuffer, sizeof(SensorPacket));

    char buf[256];
    snprintf(buf, sizeof(buf),
             "T=%.2fC  H=%.2f%%  P=%.2fhPa  MIC=%.2fdB SPL  "
             "AQI=%u  TVOC=%uppb  eCO2=%uppm  Status=0x%02X\r\n",
             pkt.temperature,
             pkt.humidity,
             pkt.pressure,
             pkt.mic,
             pkt.aqi,
             pkt.tvoc,
             pkt.eco2,
             pkt.ens160_status);

    UART_Print(buf);
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


  UART_Print("=== nRF905 RX Test ===\r\n");
  NRF905_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      GPIO_PinState dr_pin = HAL_GPIO_ReadPin(NRF905_DR_PORT, NRF905_DR_PIN);

      if(dr_pin == GPIO_PIN_SET)
      {
          UART_Print("Receiving payload\r\n");
          NRF905_ReceivePayload();
          HAL_Delay(10);
      }

      HAL_Delay(100);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
