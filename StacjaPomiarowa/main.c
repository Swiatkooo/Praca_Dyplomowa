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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "main.h"

// --- Definicje pinów nRF905 ---
#define NRF905_CE_PORT     GPIOB
#define NRF905_CE_PIN      GPIO_PIN_1
#define NRF905_TXEN_PORT   GPIOC
#define NRF905_TXEN_PIN    GPIO_PIN_6
#define NRF905_CSN_PORT    GPIOC
#define NRF905_CSN_PIN     GPIO_PIN_9
#define NRF905_PWR_PORT    GPIOC
#define NRF905_PWR_PIN     GPIO_PIN_8
#define NRF905_DR_PORT     GPIOC
#define NRF905_DR_PIN      GPIO_PIN_5

#define NRF905_CE_HIGH()    HAL_GPIO_WritePin(NRF905_CE_PORT, NRF905_CE_PIN, GPIO_PIN_SET)
#define NRF905_CE_LOW()     HAL_GPIO_WritePin(NRF905_CE_PORT, NRF905_CE_PIN, GPIO_PIN_RESET)
#define NRF905_TXEN_HIGH()  HAL_GPIO_WritePin(NRF905_TXEN_PORT, NRF905_TXEN_PIN, GPIO_PIN_SET)
#define NRF905_TXEN_LOW()   HAL_GPIO_WritePin(NRF905_TXEN_PORT, NRF905_TXEN_PIN, GPIO_PIN_RESET)
#define NRF905_CSN_HIGH()   HAL_GPIO_WritePin(NRF905_CSN_PORT, NRF905_CSN_PIN, GPIO_PIN_SET)
#define NRF905_CSN_LOW()    HAL_GPIO_WritePin(NRF905_CSN_PORT, NRF905_CSN_PIN, GPIO_PIN_RESET)

#define BME280_ADDR (0x76 << 1)

typedef struct __attribute__((packed)) {
    float temperature;
    float humidity;
    float pressure;
    float mic;
    uint16_t tvoc;
    uint16_t eco2;
    uint8_t aqi;
    uint8_t ens160_status;
} SensorPacket;

uint8_t txBuffer[32];

// --- UART helper ---
void UART_Print(const char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

#define ENS160_I2C_ADDR    0xA4

#define ENS160_REG_PART_ID     0x00
#define ENS160_REG_OPMODE      0x10
#define ENS160_REG_CONFIG      0x11
#define ENS160_REG_TEMP_IN     0x13
#define ENS160_REG_RH_IN       0x15
#define ENS160_REG_DATA_STATUS 0x20
#define ENS160_REG_DATA_AQI    0x21
#define ENS160_REG_DATA_TVOC   0x22
#define ENS160_REG_DATA_ECO2   0x24

#define ENS160_MODE_IDLE     0x01
#define ENS160_MODE_STANDARD 0x02

// Odczyt PART_ID
uint8_t ENS160_ReadPartID(void)
{
    uint8_t id[2] = {0};
    HAL_I2C_Mem_Read(&hi2c2, ENS160_I2C_ADDR, ENS160_REG_PART_ID, 1, id, 2, 100);
    uint16_t part = id[0] | (id[1] << 8);
    return (part == 0x0160);
}

// Ustawienie temperatury i wilgotnosci
void ENS160_SetEnvironment(float tempC, float rhPercent)
{
    uint16_t t = (uint16_t)((tempC + 273.15f) * 64.0f);
    uint16_t h = (uint16_t)(rhPercent * 512.0f);

    uint8_t tb[2] = { t & 0xFF, t >> 8 };
    uint8_t hb[2] = { h & 0xFF, h >> 8 };

    HAL_I2C_Mem_Write(&hi2c2, ENS160_I2C_ADDR, ENS160_REG_TEMP_IN, 1, tb, 2, 100);
    HAL_I2C_Mem_Write(&hi2c2, ENS160_I2C_ADDR, ENS160_REG_RH_IN,   1, hb, 2, 100);
}

// Inicjalizacja ENS160
uint8_t ENS160_Init(void)
{
    HAL_Delay(50);
    if (!ENS160_ReadPartID()) return 0;

    uint8_t mode = ENS160_MODE_IDLE;
    HAL_I2C_Mem_Write(&hi2c2, ENS160_I2C_ADDR, ENS160_REG_OPMODE, 1, &mode, 1, 100);
    HAL_Delay(10);

    mode = ENS160_MODE_STANDARD;
    HAL_I2C_Mem_Write(&hi2c2, ENS160_I2C_ADDR, ENS160_REG_OPMODE, 1, &mode, 1, 100);
    HAL_Delay(200);

    ENS160_SetEnvironment(25.0f, 50.0f);

    return 1;
}

// Odczyt danych: AQI, TVOC, eCO2, Status
void ENS160_Read(uint8_t *aqi, uint16_t *tvoc, uint16_t *eco2, uint8_t *status)
{
    uint8_t buf[2];

    HAL_I2C_Mem_Read(&hi2c2, ENS160_I2C_ADDR, ENS160_REG_DATA_STATUS, 1, status, 1, 100);
    HAL_I2C_Mem_Read(&hi2c2, ENS160_I2C_ADDR, ENS160_REG_DATA_AQI,    1, aqi,    1, 100);

    HAL_I2C_Mem_Read(&hi2c2, ENS160_I2C_ADDR, ENS160_REG_DATA_TVOC,   1, buf, 2, 100);
    *tvoc = buf[0] | (buf[1] << 8);

    HAL_I2C_Mem_Read(&hi2c2, ENS160_I2C_ADDR, ENS160_REG_DATA_ECO2,   1, buf, 2, 100);
    *eco2 = buf[0] | (buf[1] << 8);
}

// --- BME280 Kalibracja i kompensacja ---
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
    int32_t  t_fine;
} BME280_CalibData;

BME280_CalibData bme280_calib;

void BME280_Write(uint8_t reg, uint8_t val)
{
    HAL_I2C_Mem_Write(&hi2c1, BME280_ADDR, reg, 1, &val, 1, HAL_MAX_DELAY);
}

void BME280_Read(uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR, reg, 1, buf, len, HAL_MAX_DELAY);
}

void BME280_ReadCalibration(void)
{
    uint8_t data[26];
    BME280_Read(0x88, data, 26);
    bme280_calib.dig_T1 = (data[1] << 8) | data[0];
    bme280_calib.dig_T2 = (data[3] << 8) | data[2];
    bme280_calib.dig_T3 = (data[5] << 8) | data[4];
    bme280_calib.dig_P1 = (data[7] << 8) | data[6];
    bme280_calib.dig_P2 = (data[9] << 8) | data[8];
    bme280_calib.dig_P3 = (data[11] << 8) | data[10];
    bme280_calib.dig_P4 = (data[13] << 8) | data[12];
    bme280_calib.dig_P5 = (data[15] << 8) | data[14];
    bme280_calib.dig_P6 = (data[17] << 8) | data[16];
    bme280_calib.dig_P7 = (data[19] << 8) | data[18];
    bme280_calib.dig_P8 = (data[21] << 8) | data[20];
    bme280_calib.dig_P9 = (data[23] << 8) | data[22];
    bme280_calib.dig_H1 = data[25];

    uint8_t data_h[7];
    BME280_Read(0xE1, data_h, 7);
    bme280_calib.dig_H2 = (data_h[1] << 8) | data_h[0];
    bme280_calib.dig_H3 = data_h[2];
    bme280_calib.dig_H4 = (data_h[3] << 4) | (data_h[4] & 0x0F);
    bme280_calib.dig_H5 = (data_h[5] << 4) | (data_h[4] >> 4);
    bme280_calib.dig_H6 = data_h[6];
}

void BME280_Init(void)
{
    uint8_t id;
    BME280_Read(0xD0, &id, 1);
    if (id != 0x60) {
        UART_Print("BME280 not detected!\r\n");
    }
    BME280_Write(0xF2, 0x01); // Humidity oversampling x1
    BME280_Write(0xF4, 0x27); // Temp+Press oversampling x1, normal mode
    BME280_Write(0xF5, 0xA0); // Standby 1000ms, filter off
    BME280_ReadCalibration();
}

void BME280_ReadData(float *temp, float *press, float *hum)
{
    uint8_t d[8];
    BME280_Read(0xF7, d, 8);

    int32_t adc_P = (d[0] << 12) | (d[1] << 4) | (d[2] >> 4);
    int32_t adc_T = (d[3] << 12) | (d[4] << 4) | (d[5] >> 4);
    int32_t adc_H = (d[6] << 8) | d[7];

    int32_t var1 = ((((adc_T >> 3) - ((int32_t)bme280_calib.dig_T1 << 1))) *
                    ((int32_t)bme280_calib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)bme280_calib.dig_T1)) *
                     ((adc_T >> 4) - ((int32_t)bme280_calib.dig_T1))) >> 12) *
                     ((int32_t)bme280_calib.dig_T3)) >> 14;
    bme280_calib.t_fine = var1 + var2;
    *temp = (bme280_calib.t_fine * 5 + 128) >> 8;
    *temp /= 100.0f;

    int64_t var1_p = ((int64_t)bme280_calib.t_fine) - 128000;
    int64_t var2_p = var1_p * var1_p * (int64_t)bme280_calib.dig_P6;
    var2_p = var2_p + ((var1_p * (int64_t)bme280_calib.dig_P5) << 17);
    var2_p = var2_p + (((int64_t)bme280_calib.dig_P4) << 35);
    var1_p = ((var1_p * var1_p * (int64_t)bme280_calib.dig_P3) >> 8) + ((var1_p * (int64_t)bme280_calib.dig_P2) << 12);
    var1_p = (((((int64_t)1) << 47) + var1_p)) * ((int64_t)bme280_calib.dig_P1) >> 33;
    if (var1_p == 0) *press = 0;
    else {
        int64_t p = 1048576 - adc_P;
        p = (((p << 31) - var2_p) * 3125) / var1_p;
        var1_p = (((int64_t)bme280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        var2_p = (((int64_t)bme280_calib.dig_P8) * p) >> 19;
        p = ((p + var1_p + var2_p) >> 8) + (((int64_t)bme280_calib.dig_P7) << 4);
        *press = (float)p / 25600.0f;
    }

    int32_t v_x1_u32r = bme280_calib.t_fine - 76800;
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)bme280_calib.dig_H4) << 20) -
                    (((int32_t)bme280_calib.dig_H5) * v_x1_u32r)) + 16384) >> 15) *
                  (((((((v_x1_u32r * ((int32_t)bme280_calib.dig_H6)) >> 10) *
                     (((v_x1_u32r * ((int32_t)bme280_calib.dig_H3)) >> 11) + 32768)) >> 10) +
                    2097152) * ((int32_t)bme280_calib.dig_H2) + 8192) >> 14));
    v_x1_u32r -= (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                  ((int32_t)bme280_calib.dig_H1)) >> 4);
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    *hum = (float)(v_x1_u32r >> 12) / 1024.0f;
}

volatile uint32_t sum = 0;
volatile uint64_t sumsq = 0;
volatile uint32_t count = 0;

// Callback ADC
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
    	uint16_t sample = ADC1->DR;
        sum   += sample;
        sumsq += (double)sample * (double)sample;
        count++;
    }
}

typedef struct { float x; float y; } cal_point_t;

static const cal_point_t cal_pts[] = {
    { -35.9176f, 31.0f },
    { -35.3910f, 39.0f },
    { -34.4249f, 45.0f },
    { -31.3727f, 50.0f },
    { -28.4043f, 55.0f },
    { -25.8486f, 59.0f },
    { -20.3546f, 63.5f },
    { -17.3933f, 67.5f },
    { -14.8945f, 70.0f },
    { -11.5957f, 74.0f },
    {  -8.1343f, 77.0f },
};
static const int cal_n = sizeof(cal_pts)/sizeof(cal_pts[0]);

static inline float x_from_vrms(float vrms) {
    return 20.0f * log10f(vrms);
}

// interpolacja odcinkowa
static float spl_from_vrms(float vrms) {
    float x = x_from_vrms(vrms);
    if (x <= cal_pts[0].x) return cal_pts[0].y;
    if (x >= cal_pts[cal_n-1].x) return cal_pts[cal_n-1].y;
    for (int i = 0; i < cal_n - 1; i++) {
        if (x >= cal_pts[i].x && x <= cal_pts[i+1].x) {
            float t = (x - cal_pts[i].x) / (cal_pts[i+1].x - cal_pts[i].x);
            return cal_pts[i].y + t * (cal_pts[i+1].y - cal_pts[i].y);
        }
    }
    return cal_pts[cal_n-1].y; // fallback
}

float measure_db_spl(void) {
    if (count == 0) return 0.0f;

    float mean     = (float)sum   / (float)count;
    float mean_sq  = (float)sumsq / (float)count;
    float variance = mean_sq - (mean * mean);

    float rms_counts = sqrtf(variance);
    float vrms       = rms_counts * (3.3f / 4096.0f);

    // reset okna
    sum = 0; sumsq = 0; count = 0;

    return spl_from_vrms(vrms);
}

// Start/stop
void audio_start(void) {
    sum = 0.0; sumsq = 0.0; count = 0;
    HAL_TIM_Base_Start(&htim3);
    HAL_ADC_Start_IT(&hadc1);
}

void audio_stop(void) {
    HAL_TIM_Base_Stop(&htim3);
    HAL_ADC_Stop_IT(&hadc1);
}

// --- nRF905 ---
uint8_t NRF905_ReadStatus(void)
{
    uint8_t cmd = 0x1F, status = 0;
    NRF905_CSN_LOW();
    HAL_SPI_TransmitReceive(&hspi1, &cmd, &status, 1, HAL_MAX_DELAY);
    NRF905_CSN_HIGH();
    return status;
}

void NRF905_Init(void)
{
    NRF905_CSN_HIGH();
    NRF905_CE_LOW();
    NRF905_TXEN_LOW();
    HAL_GPIO_WritePin(NRF905_PWR_PORT, NRF905_PWR_PIN, GPIO_PIN_SET);
    HAL_Delay(10);

    uint8_t cfg[10] = {
        0x6C,       // channel = 108
        0x0C,       // HFREQ_PLL=0 (433MHz), PA_PWR=+10dBm, RX_RED_PWR=0, AUTO_RETRAN=0
        0x44,       // address width = 4 bytes
        0x20,       // RX payload size = 32 bytes
        0x20,       // TX payload size = 32 bytes
        0xE7, 0xE7, 0xE7, 0xE7, // TX address
        0xDB        // CRC=16bit, CRC_EN=ON, XOF=12MHz, UP_CLK_EN=0
    };

    uint8_t cmd = 0x00;
    NRF905_CSN_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, cfg, 10, HAL_MAX_DELAY);
    NRF905_CSN_HIGH();

    uint8_t addr[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    cmd = 0x22;
    NRF905_CSN_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, addr, 4, HAL_MAX_DELAY);
    NRF905_CSN_HIGH();

    UART_Print("NRF905 TX init done\r\n");
}

void NRF905_Transmit(SensorPacket *pkt)
{
    uint8_t tx_buffer[32] = {0};

    memcpy(tx_buffer, pkt, sizeof(SensorPacket));

    uint8_t cmd = 0x20; // W_TX_PAYLOAD
    NRF905_CSN_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, tx_buffer, 32, HAL_MAX_DELAY);
    NRF905_CSN_HIGH();

    NRF905_TXEN_HIGH();
    NRF905_CE_HIGH();

    HAL_Delay(50);

    uint8_t status = NRF905_ReadStatus();
    GPIO_PinState dr_state = HAL_GPIO_ReadPin(NRF905_DR_PORT, NRF905_DR_PIN);

    char log[128];
    snprintf(log, sizeof(log), "TX done. STATUS=0x%02X DR=%d\r\n", status, dr_state);
    UART_Print(log);

    NRF905_CE_LOW();
    NRF905_TXEN_LOW();

    cmd = 0x20; // W_TX_PAYLOAD
    uint8_t empty_buffer[32] = {0};
    NRF905_CSN_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, empty_buffer, 32, HAL_MAX_DELAY);
    NRF905_CSN_HIGH();
}

void NRF905_SetTxAddress(uint8_t *addr)
{
    uint8_t cmd = 0x22; // W_TX_ADDRESS
    NRF905_CSN_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, addr, 4, HAL_MAX_DELAY);
    NRF905_CSN_HIGH();
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  UART_Print("=== nRF905 TX + BME280 ===\r\n");

  NRF905_Init();
  BME280_Init();

  if (!ENS160_Init()) {
       UART_Print("ENS160 init failed!\r\n");
   }

  SensorPacket pkt;

  uint8_t tx_addr[4] = {0xE7, 0xE7, 0xE7, 0xE7};
  NRF905_SetTxAddress(tx_addr);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  BME280_ReadData(&pkt.temperature, &pkt.pressure, &pkt.humidity);

	  audio_start();
	  HAL_Delay(100);
	  audio_stop();

	  pkt.mic = measure_db_spl();

	  ENS160_SetEnvironment(pkt.temperature, pkt.humidity);
	  HAL_Delay(100);

	  uint8_t aqi, status;
	  uint16_t tvoc, eco2;
	  ENS160_Read(&aqi, &tvoc, &eco2, &status);

	  pkt.aqi = aqi;
	  pkt.tvoc = tvoc;
	  pkt.eco2 = eco2;
	  pkt.ens160_status = status;

      char buf[200];
      snprintf(buf, sizeof(buf),
          "T=%.1fC H=%.1f%% P=%.1fhPa\r\n"
          "MIC=%.2fdB SPL\r\n"
          "ENS160: AQI=%u TVOC=%uppb eCO2=%uppm Status=0x%02X\r\n\n",
          pkt.temperature,
          pkt.humidity,
          pkt.pressure,
          pkt.mic,
          pkt.aqi,
          pkt.tvoc,
          pkt.eco2,
          pkt.ens160_status);

      UART_Print(buf);
	  NRF905_Transmit(&pkt);

	  HAL_Delay(750);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 24;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
