/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "iks01a3_motion_sensors.h"
#include "iks01a3_env_sensors.h"
#include "stm32l1xx_nucleo_bus.h"
#include "max7219_Yncrea2.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  APP_ENGINE_STOPPED = 0,
  APP_ENGINE_RUNNING,
  APP_ENGINE_STUTTERING
} APP_EngineState_t;

typedef enum
{
  APP_FLAPS_UP = 0,
  APP_FLAPS_MID,
  APP_FLAPS_DOWN
} APP_FlapsState_t;

typedef struct
{
  int32_t temperature_c;
  int32_t pressure_hpa;
  float pressure_hpa_raw;
  int32_t humidity_pc;
  int32_t magnetic_mgauss;
  IKS01A3_MOTION_SENSOR_Axes_t accel;
  uint8_t accel_valid;
  uint8_t pressure_valid;
} APP_Sensors_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_ADC_MAX_VALUE              4095U
#define APP_ADC_PERIOD_MS              50U
#define APP_SENSOR_PERIOD_MS           200U
#define APP_DISPLAY_PERIOD_MS          80U
#define APP_SCROLL_PERIOD_MS           300U
#define APP_MESSAGE_MS                 1800U
#define APP_MODE_PULSE_MS              1000U
#define APP_SELF_TEST_MS               2000U
#define APP_TEXT_MAX                   48U
#define APP_MODE_COUNT                 8U

#define APP_NORMAL_BRIGHTNESS          8U
#define APP_FULL_BRIGHTNESS            15U
#define APP_FUEL_MAX_PERMILLE          1000U
#define APP_FUEL_LOW_PERMILLE          100U
#define APP_FUEL_STEP_PERMILLE         100U
#define APP_FUEL_FINE_STEP_PERMILLE    10U
#define APP_FUEL_BURN_DENOMINATOR      36000000UL
#define APP_ENGINE_STOP_DELAY_MS       3000U
#define APP_LOW_FUEL_STUTTER_MS        650U

#define APP_INERTIAL_DELTA_MG          180L
#define APP_PRESSURE_LIFT_DELTA_HPA    0.70f
#define APP_DEBOUNCE_MS                80U
#define APP_DISPLAY_DIAG_ONLY          1U
#define APP_DISPLAY_REVERSE_DIGITS     0U
#define APP_DISPLAY_TEXT_DELAY_MS      1200U
#define APP_DISPLAY_SCROLL_DELAY_MS    250U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#if defined(__GNUC__)
#define APP_UNUSED __attribute__((unused))
#else
#define APP_UNUSED
#endif

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint8_t app_lsm6dso_acc_ok = 0U;
static uint8_t app_lsm6dso_gyr_ok = 0U;
static uint8_t app_lis2dw12_acc_ok = 0U;
static uint8_t app_lis2mdl_mag_ok = 0U;
static uint8_t app_hts221_ok = 0U;
static uint8_t app_lps22hh_ok = 0U;
static uint8_t app_stts751_ok = 0U;

static volatile uint8_t app_evt_btn1 = 0U;
static volatile uint8_t app_evt_btn2 = 0U;
static volatile uint8_t app_evt_btn3 = 0U;
static volatile uint8_t app_evt_btn4 = 0U;
static volatile uint8_t app_tim6_1s = 0U;

static uint32_t app_last_btn1_irq_ms = 0U;
static uint32_t app_last_btn2_irq_ms = 0U;
static uint32_t app_last_btn3_irq_ms = 0U;
static uint32_t app_last_btn4_irq_ms = 0U;

static APP_Sensors_t app_sensors = {0};
static APP_EngineState_t app_engine_state = APP_ENGINE_STOPPED;
static APP_FlapsState_t app_flaps = APP_FLAPS_UP;
static uint8_t app_gear_down = 0U;
static uint8_t app_mode = 0U;
static uint8_t app_mode_pulse_led = 0U;
static uint8_t app_throttle_pc = 0U;
static uint8_t app_mixture_pc = 0U;
static uint16_t app_fuel_permille = APP_FUEL_MAX_PERMILLE;
static uint32_t app_fuel_burn_acc = 0U;
static uint32_t app_last_fuel_ms = 0U;
static uint32_t app_engine_stutter_until_ms = 0U;
static uint32_t app_low_fuel_stutter_until_ms = 0U;
static uint32_t app_next_low_fuel_stutter_ms = 0U;

static IKS01A3_MOTION_SENSOR_Axes_t app_accel_ref = {0};
static float app_pressure_ref_hpa = 0.0f;
static uint8_t app_inertial_calibrated = 0U;
static uint8_t app_pose_alert = 0U;

static uint8_t app_b1_down = 0U;
static uint8_t app_b1_combo_used = 0U;

static char app_mode_text[APP_TEXT_MAX] = "ISEN";
static char app_message_text[APP_TEXT_MAX] = {0};
static uint32_t app_message_until_ms = 0U;
static uint8_t app_message_scroll = 0U;
static uint32_t app_mode_pulse_until_ms = 0U;
static uint32_t app_last_adc_ms = 0U;
static uint32_t app_last_sensor_ms = 0U;
static uint32_t app_last_display_ms = 0U;
static uint32_t app_last_scroll_ms = 0U;
static uint8_t app_scroll_index = 0U;
static uint8_t app_display_brightness = 0xFFU;
static uint32_t app_beep_until_ms = 0U;
static uint16_t app_beep_freq_hz = 0U;
static uint32_t app_random_state = 0x2342U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
static void APP_IKS01A3_Init(void) APP_UNUSED;
static void APP_PrintInitResult(const char *name, int32_t ret);
static void APP_Trace_Init(void);
static void APP_PutChar(int ch);
static uint8_t APP_I2C1_Scan(void);
static void APP_I2C1_PrintLineState(const char *label);
static void APP_I2C1_RecoverBus(void);
static void APP_OutputSelfTest(void) APP_UNUSED;
static void APP_Process(void) APP_UNUSED;
static void APP_ReadAdc(void);
static void APP_ReadSensors(void);
static void APP_CalibrateInertial(void) APP_UNUSED;
static void APP_HandleButtons(uint32_t now_ms);
static void APP_AttemptEngineStart(void);
static void APP_StopEngine(void);
static void APP_AdjustFuel(int8_t direction);
static void APP_EngineTask(uint32_t now_ms);
static void APP_DisplayTask(uint32_t now_ms);
static void APP_OutputTask(uint32_t now_ms);
static void APP_BuzzerTask(uint32_t now_ms);
static void APP_UpdateModeText(void);
static void APP_DisplayWindow(const char *text, uint8_t scroll, uint32_t now_ms);
static void APP_ResetDisplayScroll(void);
static void APP_RequestMessage(const char *text, uint32_t duration_ms, uint8_t scroll);
static void APP_SetMode(uint8_t mode, uint32_t now_ms);
static void APP_SetLedMask(uint8_t mask);
static uint8_t APP_LedLevelToMask(uint8_t level);
static uint8_t APP_ApplyLowFuelLed(uint8_t mask, uint32_t now_ms);
static void APP_SetBuzzer(uint16_t freq_hz, uint8_t duty_pc);
static void APP_Beep(uint16_t freq_hz, uint32_t duration_ms);
static uint16_t APP_EngineFrequency(void);
static uint8_t APP_PercentToLedLevel(uint8_t pc);
static int32_t APP_FloatToInt(float value);
static int32_t APP_Abs32(int32_t value);
static uint32_t APP_RandRange(uint32_t min_value, uint32_t max_value);
static void HAL_GPIO_EXTI_DebouncedEvent(uint32_t *last_ms, volatile uint8_t *event);
static void APP_Display7Seg_Init(void);
static void APP_Display7Seg_ShowText(const char *text);
static void APP_Display7Seg_ScrollText(const char *text);
static void APP_Display7Seg_WriteChar(uint8_t position, char character);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  if (ch == '\n')
  {
    APP_PutChar('\r');
  }

  APP_PutChar(ch);

  return ch;
}

static void APP_Trace_Init(void)
{
#if defined(DBGMCU_CR_TRACE_IOEN) && defined(DBGMCU_CR_TRACE_MODE)
  DBGMCU->CR = (DBGMCU->CR & ~DBGMCU_CR_TRACE_MODE) | DBGMCU_CR_TRACE_IOEN;
#endif
}

static void APP_PutChar(int ch)
{
  uint8_t c;

  c = (uint8_t)ch;
  (void)ITM_SendChar((uint32_t)c);

  if (huart2.Instance != NULL)
  {
    (void)HAL_UART_Transmit(&huart2, &c, 1U, 10U);
  }
}

static void APP_OutputSelfTest(void)
{
  APP_SetLedMask(0xFFU);
  MAX7219_SetBrightness(APP_FULL_BRIGHTNESS);
  app_display_brightness = APP_FULL_BRIGHTNESS;
  MAX7219_DisplayTestStart();
  APP_SetBuzzer(900U, 50U);
  HAL_Delay(APP_SELF_TEST_MS);
  APP_SetBuzzer(0U, 0U);
  MAX7219_DisplayTestStop();
  MAX7219_Clear();
  APP_SetLedMask(0x00U);
}

static void APP_IKS01A3_Init(void)
{
  int32_t ret;
  uint8_t detected;

  printf("\n========================================\n");
  printf("Initialisation X-NUCLEO-IKS01A3\n");
  printf("Console UART2 : 115200 8N1\n");
  printf("========================================\n");

  detected = APP_I2C1_Scan();

  if (detected == 0U)
  {
    printf("Aucune adresse I2C detectee sur PB8/PB9.\n");
    printf("Verifier : shield IKS01A3 bien enfonce, alimentation 3V3/5V, SDA=D14/PB9, SCL=D15/PB8.\n");
  }

  ret = IKS01A3_MOTION_SENSOR_Init(IKS01A3_LSM6DSO_0, MOTION_ACCELERO);
  APP_PrintInitResult("LSM6DSO accelerometre", ret);
  if (ret == BSP_ERROR_NONE)
  {
    app_lsm6dso_acc_ok = 1U;
    (void)IKS01A3_MOTION_SENSOR_Enable(IKS01A3_LSM6DSO_0, MOTION_ACCELERO);
    (void)IKS01A3_MOTION_SENSOR_SetFullScale(IKS01A3_LSM6DSO_0, MOTION_ACCELERO, 2);
    (void)IKS01A3_MOTION_SENSOR_SetOutputDataRate(IKS01A3_LSM6DSO_0, MOTION_ACCELERO, 100.0f);
  }

  ret = IKS01A3_MOTION_SENSOR_Init(IKS01A3_LSM6DSO_0, MOTION_GYRO);
  APP_PrintInitResult("LSM6DSO gyroscope", ret);
  if (ret == BSP_ERROR_NONE)
  {
    app_lsm6dso_gyr_ok = 1U;
    (void)IKS01A3_MOTION_SENSOR_Enable(IKS01A3_LSM6DSO_0, MOTION_GYRO);
    (void)IKS01A3_MOTION_SENSOR_SetFullScale(IKS01A3_LSM6DSO_0, MOTION_GYRO, 250);
    (void)IKS01A3_MOTION_SENSOR_SetOutputDataRate(IKS01A3_LSM6DSO_0, MOTION_GYRO, 100.0f);
  }

  ret = IKS01A3_MOTION_SENSOR_Init(IKS01A3_LIS2DW12_0, MOTION_ACCELERO);
  APP_PrintInitResult("LIS2DW12 accelerometre", ret);
  if (ret == BSP_ERROR_NONE)
  {
    app_lis2dw12_acc_ok = 1U;
    (void)IKS01A3_MOTION_SENSOR_Enable(IKS01A3_LIS2DW12_0, MOTION_ACCELERO);
    (void)IKS01A3_MOTION_SENSOR_SetFullScale(IKS01A3_LIS2DW12_0, MOTION_ACCELERO, 2);
    (void)IKS01A3_MOTION_SENSOR_SetOutputDataRate(IKS01A3_LIS2DW12_0, MOTION_ACCELERO, 100.0f);
  }

  ret = IKS01A3_MOTION_SENSOR_Init(IKS01A3_LIS2MDL_0, MOTION_MAGNETO);
  APP_PrintInitResult("LIS2MDL magnetometre", ret);
  if (ret == BSP_ERROR_NONE)
  {
    app_lis2mdl_mag_ok = 1U;
    (void)IKS01A3_MOTION_SENSOR_Enable(IKS01A3_LIS2MDL_0, MOTION_MAGNETO);
    (void)IKS01A3_MOTION_SENSOR_SetOutputDataRate(IKS01A3_LIS2MDL_0, MOTION_MAGNETO, 100.0f);
  }

  ret = IKS01A3_ENV_SENSOR_Init(IKS01A3_HTS221_0, ENV_TEMPERATURE | ENV_HUMIDITY);
  APP_PrintInitResult("HTS221 temperature + humidite", ret);
  if (ret == BSP_ERROR_NONE)
  {
    app_hts221_ok = 1U;
    (void)IKS01A3_ENV_SENSOR_Enable(IKS01A3_HTS221_0, ENV_TEMPERATURE);
    (void)IKS01A3_ENV_SENSOR_Enable(IKS01A3_HTS221_0, ENV_HUMIDITY);
  }

  ret = IKS01A3_ENV_SENSOR_Init(IKS01A3_LPS22HH_0, ENV_PRESSURE | ENV_TEMPERATURE);
  APP_PrintInitResult("LPS22HH pression + temperature", ret);
  if (ret == BSP_ERROR_NONE)
  {
    app_lps22hh_ok = 1U;
    (void)IKS01A3_ENV_SENSOR_Enable(IKS01A3_LPS22HH_0, ENV_PRESSURE);
    (void)IKS01A3_ENV_SENSOR_Enable(IKS01A3_LPS22HH_0, ENV_TEMPERATURE);
  }

  ret = IKS01A3_ENV_SENSOR_Init(IKS01A3_STTS751_0, ENV_TEMPERATURE);
  APP_PrintInitResult("STTS751 temperature", ret);
  if (ret == BSP_ERROR_NONE)
  {
    app_stts751_ok = 1U;
    (void)IKS01A3_ENV_SENSOR_Enable(IKS01A3_STTS751_0, ENV_TEMPERATURE);
  }

  printf("========================================\n");
  printf("Application avion active\n");
  printf("========================================\n");
}

static uint8_t APP_I2C1_Scan(void)
{
  uint8_t count = 0U;
  int32_t ret;

  printf("Scan I2C1 PB8=SCL PB9=SDA...\n");

  APP_I2C1_RecoverBus();

  ret = BSP_I2C1_Init();
  if (ret != BSP_ERROR_NONE)
  {
    printf("BSP_I2C1_Init erreur ret=%ld HALerr=0x%08lx\n",
           (long)ret,
           (unsigned long)HAL_I2C_GetError(&hi2c1));
    return 0U;
  }

  APP_I2C1_PrintLineState("Etat lignes apres init");

  for (uint8_t addr = 0x08U; addr <= 0x77U; addr++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 2U, 20U) == HAL_OK)
    {
      printf("  - device 0x%02X\n", addr);
      count++;
    }
  }

  if (count == 0U)
  {
    printf("  aucun device trouve\n");
  }
  else
  {
    printf("  %u device(s) trouve(s)\n", count);
    printf("  attendus IKS01A3 typiques: 0x19/0x1E/0x5D/0x5F/0x6B/0x4A\n");
  }

  return count;
}

static void APP_I2C1_PrintLineState(const char *label)
{
  GPIO_PinState scl = HAL_GPIO_ReadPin(BUS_I2C1_SCL_GPIO_PORT, BUS_I2C1_SCL_GPIO_PIN);
  GPIO_PinState sda = HAL_GPIO_ReadPin(BUS_I2C1_SDA_GPIO_PORT, BUS_I2C1_SDA_GPIO_PIN);

  printf("%s : SCL=%s SDA=%s\n",
         label,
         (scl == GPIO_PIN_SET) ? "HIGH" : "LOW",
         (sda == GPIO_PIN_SET) ? "HIGH" : "LOW");

  if ((scl == GPIO_PIN_RESET) || (sda == GPIO_PIN_RESET))
  {
    printf("  Attention : une ligne I2C est maintenue a 0, bus bloque ou court-circuit possible.\n");
  }
}

static void APP_I2C1_RecoverBus(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_I2C1_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = BUS_I2C1_SCL_GPIO_PIN | BUS_I2C1_SDA_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(BUS_I2C1_SCL_GPIO_PORT, BUS_I2C1_SCL_GPIO_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BUS_I2C1_SDA_GPIO_PORT, BUS_I2C1_SDA_GPIO_PIN, GPIO_PIN_SET);
  HAL_Delay(2U);

  APP_I2C1_PrintLineState("Etat lignes avant recovery");

  for (uint8_t i = 0U; i < 16U; i++)
  {
    if (HAL_GPIO_ReadPin(BUS_I2C1_SDA_GPIO_PORT, BUS_I2C1_SDA_GPIO_PIN) == GPIO_PIN_SET)
    {
      break;
    }

    HAL_GPIO_WritePin(BUS_I2C1_SCL_GPIO_PORT, BUS_I2C1_SCL_GPIO_PIN, GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(BUS_I2C1_SCL_GPIO_PORT, BUS_I2C1_SCL_GPIO_PIN, GPIO_PIN_SET);
    HAL_Delay(1U);
  }

  HAL_GPIO_WritePin(BUS_I2C1_SDA_GPIO_PORT, BUS_I2C1_SDA_GPIO_PIN, GPIO_PIN_RESET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(BUS_I2C1_SCL_GPIO_PORT, BUS_I2C1_SCL_GPIO_PIN, GPIO_PIN_SET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(BUS_I2C1_SDA_GPIO_PORT, BUS_I2C1_SDA_GPIO_PIN, GPIO_PIN_SET);
  HAL_Delay(2U);

  APP_I2C1_PrintLineState("Etat lignes apres recovery");
}

static void APP_PrintInitResult(const char *name, int32_t ret)
{
  if (ret == BSP_ERROR_NONE)
  {
    printf("[OK]    %s\n", name);
  }
  else
  {
    printf("[ERREUR] %s ret=%ld\n", name, (long)ret);
  }
}

static void APP_Display7Seg_Init(void)
{
  printf("\n========================================\n");
  printf("Diagnostic afficheur MAX7219\n");
  printf("SPI1 : PA5=SCK PA7=MOSI PA8=CS\n");
  printf("========================================\n");

  SPI_CS_High();
  HAL_Delay(10U);
  printf("Init MAX7219...\n");
  MAX7219_Init();
  MAX7219_SetBrightness(15);

  printf("Test 1 : affichage 88888888 pendant 2 s\n");
  APP_Display7Seg_ShowText("88888888");
  HAL_Delay(2000U);
  printf("Test 1 termine, clear\n");
  MAX7219_Clear();
  HAL_Delay(200U);
  printf("Init afficheur terminee\n");
}

static void APP_Display7Seg_ShowText(const char *text)
{
  size_t len = strlen(text);

  for (uint8_t i = 0U; i < 8U; i++)
  {
    char c = (i < len) ? text[i] : ' ';

    if ((c >= 'a') && (c <= 'z'))
    {
      c = (char)(c - ('a' - 'A'));
    }

    APP_Display7Seg_WriteChar(i, c);
  }
}

static void APP_Display7Seg_ScrollText(const char *text)
{
  size_t len = strlen(text);

  for (uint8_t start = 0U; start < (uint8_t)(len + 8U); start++)
  {
    char window[9];

    for (uint8_t i = 0U; i < 8U; i++)
    {
      int16_t src = (int16_t)start + (int16_t)i - 8;
      window[i] = ((src >= 0) && ((size_t)src < len)) ? text[src] : ' ';
    }

    window[8] = '\0';
    APP_Display7Seg_ShowText(window);
    HAL_Delay(APP_DISPLAY_SCROLL_DELAY_MS);
  }
}

static void APP_Display7Seg_WriteChar(uint8_t position, char character)
{
  uint8_t digit = (uint8_t)(position + 1U);

  if (position > 7U)
  {
    return;
  }

  if (APP_DISPLAY_REVERSE_DIGITS != 0U)
  {
    digit = (uint8_t)(8U - position);
  }

  MAX7219_DisplayChar((char)digit, character);
}

static void APP_Process(void)
{
  uint32_t now_ms = HAL_GetTick();

  if (app_tim6_1s != 0U)
  {
    app_tim6_1s = 0U;
  }

  if ((now_ms - app_last_adc_ms) >= APP_ADC_PERIOD_MS)
  {
    app_last_adc_ms = now_ms;
    APP_ReadAdc();
  }

  if ((now_ms - app_last_sensor_ms) >= APP_SENSOR_PERIOD_MS)
  {
    app_last_sensor_ms = now_ms;
    APP_ReadSensors();
  }

  APP_HandleButtons(now_ms);
  APP_EngineTask(now_ms);
  APP_DisplayTask(now_ms);
  APP_OutputTask(now_ms);
  APP_BuzzerTask(now_ms);
}

static void APP_ReadAdc(void)
{
  uint32_t raw_rv2 = 0U;
  uint32_t raw_rv1 = 0U;

  if (HAL_ADC_Start(&hadc) == HAL_OK)
  {
    if (HAL_ADC_PollForConversion(&hadc, 10U) == HAL_OK)
    {
      raw_rv2 = HAL_ADC_GetValue(&hadc);

      if (HAL_ADC_PollForConversion(&hadc, 10U) == HAL_OK)
      {
        raw_rv1 = HAL_ADC_GetValue(&hadc);
        app_throttle_pc = (uint8_t)((raw_rv1 * 100U) / APP_ADC_MAX_VALUE);
        app_mixture_pc = (uint8_t)((raw_rv2 * 100U) / APP_ADC_MAX_VALUE);
      }
    }

    HAL_ADC_Stop(&hadc);
  }
}

static void APP_ReadSensors(void)
{
  IKS01A3_MOTION_SENSOR_Axes_t axes;
  float value = 0.0f;
  uint8_t accel_valid = 0U;
  uint8_t pressure_valid = 0U;

  if (app_hts221_ok != 0U)
  {
    if (IKS01A3_ENV_SENSOR_GetValue(IKS01A3_HTS221_0, ENV_TEMPERATURE, &value) == BSP_ERROR_NONE)
    {
      app_sensors.temperature_c = APP_FloatToInt(value);
    }
  }
  else if (app_stts751_ok != 0U)
  {
    if (IKS01A3_ENV_SENSOR_GetValue(IKS01A3_STTS751_0, ENV_TEMPERATURE, &value) == BSP_ERROR_NONE)
    {
      app_sensors.temperature_c = APP_FloatToInt(value);
    }
  }
  else if (app_lps22hh_ok != 0U)
  {
    if (IKS01A3_ENV_SENSOR_GetValue(IKS01A3_LPS22HH_0, ENV_TEMPERATURE, &value) == BSP_ERROR_NONE)
    {
      app_sensors.temperature_c = APP_FloatToInt(value);
    }
  }

  if (app_lps22hh_ok != 0U)
  {
    if (IKS01A3_ENV_SENSOR_GetValue(IKS01A3_LPS22HH_0, ENV_PRESSURE, &value) == BSP_ERROR_NONE)
    {
      app_sensors.pressure_hpa_raw = value;
      app_sensors.pressure_hpa = APP_FloatToInt(value);
      pressure_valid = 1U;
    }
  }

  if (app_hts221_ok != 0U)
  {
    if (IKS01A3_ENV_SENSOR_GetValue(IKS01A3_HTS221_0, ENV_HUMIDITY, &value) == BSP_ERROR_NONE)
    {
      app_sensors.humidity_pc = APP_FloatToInt(value);

      if (app_sensors.humidity_pc < 0L)
      {
        app_sensors.humidity_pc = 0L;
      }
      else if (app_sensors.humidity_pc > 100L)
      {
        app_sensors.humidity_pc = 100L;
      }
    }
  }

  if (app_lis2mdl_mag_ok != 0U)
  {
    if (IKS01A3_MOTION_SENSOR_GetAxes(IKS01A3_LIS2MDL_0, MOTION_MAGNETO, &axes) == BSP_ERROR_NONE)
    {
      app_sensors.magnetic_mgauss = (APP_Abs32(axes.x) + APP_Abs32(axes.y) + APP_Abs32(axes.z)) / 3L;
    }
  }

  if (app_lsm6dso_acc_ok != 0U)
  {
    if (IKS01A3_MOTION_SENSOR_GetAxes(IKS01A3_LSM6DSO_0, MOTION_ACCELERO, &axes) == BSP_ERROR_NONE)
    {
      app_sensors.accel = axes;
      accel_valid = 1U;
    }
  }
  else if (app_lis2dw12_acc_ok != 0U)
  {
    if (IKS01A3_MOTION_SENSOR_GetAxes(IKS01A3_LIS2DW12_0, MOTION_ACCELERO, &axes) == BSP_ERROR_NONE)
    {
      app_sensors.accel = axes;
      accel_valid = 1U;
    }
  }

  app_sensors.accel_valid = accel_valid;
  app_sensors.pressure_valid = pressure_valid;
}

static void APP_CalibrateInertial(void)
{
  int32_t x_sum = 0L;
  int32_t y_sum = 0L;
  int32_t z_sum = 0L;
  float pressure_sum = 0.0f;
  uint8_t accel_count = 0U;
  uint8_t pressure_count = 0U;

  for (uint8_t i = 0U; i < 8U; i++)
  {
    APP_ReadSensors();

    if (app_sensors.accel_valid != 0U)
    {
      x_sum += app_sensors.accel.x;
      y_sum += app_sensors.accel.y;
      z_sum += app_sensors.accel.z;
      accel_count++;
    }

    if (app_sensors.pressure_valid != 0U)
    {
      pressure_sum += app_sensors.pressure_hpa_raw;
      pressure_count++;
    }

    HAL_Delay(40U);
  }

  if (accel_count != 0U)
  {
    app_accel_ref.x = x_sum / (int32_t)accel_count;
    app_accel_ref.y = y_sum / (int32_t)accel_count;
    app_accel_ref.z = z_sum / (int32_t)accel_count;
    app_inertial_calibrated = 1U;
  }

  if (pressure_count != 0U)
  {
    app_pressure_ref_hpa = pressure_sum / (float)pressure_count;
    app_inertial_calibrated = 1U;
  }

  app_random_state ^= (uint32_t)APP_Abs32(app_accel_ref.x);
  app_random_state ^= ((uint32_t)APP_Abs32(app_accel_ref.y) << 8);
  app_random_state ^= ((uint32_t)APP_Abs32(app_accel_ref.z) << 16);

  printf("Calibration inertielle : %s\n", (app_inertial_calibrated != 0U) ? "OK" : "ERREUR");
}

static void APP_HandleButtons(uint32_t now_ms)
{
  uint8_t btn1;
  uint8_t btn2;
  uint8_t btn3;
  uint8_t btn4;
  uint8_t b1_pressed;

  __disable_irq();
  btn1 = app_evt_btn1;
  btn2 = app_evt_btn2;
  btn3 = app_evt_btn3;
  btn4 = app_evt_btn4;
  app_evt_btn1 = 0U;
  app_evt_btn2 = 0U;
  app_evt_btn3 = 0U;
  app_evt_btn4 = 0U;
  __enable_irq();

  b1_pressed = (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET) ? 1U : 0U;

  if ((b1_pressed != 0U) && (app_b1_down == 0U))
  {
    app_b1_down = 1U;
    app_b1_combo_used = 0U;
  }

  if (b1_pressed != 0U)
  {
    if (btn1 != 0U)
    {
      app_b1_combo_used = 1U;

      if (app_engine_state == APP_ENGINE_STOPPED)
      {
        APP_AttemptEngineStart();
      }
      else
      {
        APP_AdjustFuel(-1);
      }

      btn1 = 0U;
    }

    if (btn2 != 0U)
    {
      app_b1_combo_used = 1U;
      APP_AdjustFuel(1);
      btn2 = 0U;
    }
  }
  else if (app_b1_down != 0U)
  {
    if (app_b1_combo_used == 0U)
    {
      APP_SetMode((app_mode >= APP_MODE_COUNT) ? 1U : (uint8_t)(app_mode + 1U), now_ms);
    }

    app_b1_down = 0U;
  }

  if (b1_pressed == 0U)
  {
    if (btn4 != 0U)
    {
      app_gear_down = 1U;
      APP_RequestMessage("BAS", APP_MESSAGE_MS, 0U);
    }

    if (btn2 != 0U)
    {
      app_gear_down = 0U;
      APP_RequestMessage("HAUT", APP_MESSAGE_MS, 0U);
    }

    if (btn3 != 0U)
    {
      if (app_flaps == APP_FLAPS_UP)
      {
        app_flaps = APP_FLAPS_MID;
        APP_RequestMessage("MID", APP_MESSAGE_MS, 0U);
      }
      else
      {
        app_flaps = APP_FLAPS_DOWN;
        APP_RequestMessage("DOWN", APP_MESSAGE_MS, 0U);
      }
    }

    if (btn1 != 0U)
    {
      if (app_flaps == APP_FLAPS_DOWN)
      {
        app_flaps = APP_FLAPS_MID;
        APP_RequestMessage("MID", APP_MESSAGE_MS, 0U);
      }
      else
      {
        app_flaps = APP_FLAPS_UP;
        APP_RequestMessage("UP", APP_MESSAGE_MS, 0U);
      }
    }
  }
}

static void APP_AttemptEngineStart(void)
{
  uint32_t now_ms = HAL_GetTick();

  if ((app_mixture_pc >= 95U) && (app_throttle_pc >= 20U) && (app_fuel_permille != 0U))
  {
    app_engine_state = APP_ENGINE_RUNNING;
    app_last_fuel_ms = now_ms;
    app_next_low_fuel_stutter_ms = now_ms + APP_RandRange(2000U, 5000U);
    APP_RequestMessage("START", APP_MESSAGE_MS, 0U);
    APP_Beep(1200U, 250U);
  }
  else
  {
    app_engine_state = APP_ENGINE_STUTTERING;
    app_engine_stutter_until_ms = now_ms + APP_ENGINE_STOP_DELAY_MS;
    APP_RequestMessage("BROUT", APP_MESSAGE_MS, 0U);
    APP_Beep(600U, 250U);
  }
}

static void APP_StopEngine(void)
{
  app_engine_state = APP_ENGINE_STOPPED;
  app_low_fuel_stutter_until_ms = 0U;
  app_engine_stutter_until_ms = 0U;
  APP_RequestMessage("STOP", APP_MESSAGE_MS, 0U);
  APP_Beep(700U, 450U);
}

static void APP_AdjustFuel(int8_t direction)
{
  int32_t next = (int32_t)app_fuel_permille;
  uint16_t step = APP_FUEL_STEP_PERMILLE;

  if ((direction < 0) && (app_fuel_permille <= APP_FUEL_LOW_PERMILLE))
  {
    step = APP_FUEL_FINE_STEP_PERMILLE;
  }

  if (direction > 0)
  {
    next += (int32_t)step;
  }
  else
  {
    next -= (int32_t)step;
  }

  if (next < 0L)
  {
    next = 0L;
  }
  else if (next > (int32_t)APP_FUEL_MAX_PERMILLE)
  {
    next = (int32_t)APP_FUEL_MAX_PERMILLE;
  }

  app_fuel_permille = (uint16_t)next;
  snprintf(app_message_text, sizeof(app_message_text), "RES=%uPC", (unsigned int)(app_fuel_permille / 10U));
  app_message_until_ms = HAL_GetTick() + APP_MESSAGE_MS;
  app_message_scroll = 0U;
  APP_ResetDisplayScroll();

  if ((app_fuel_permille == 0U) && (app_engine_state != APP_ENGINE_STOPPED))
  {
    APP_StopEngine();
  }
}

static void APP_EngineTask(uint32_t now_ms)
{
  uint8_t lifted = 0U;

  if (app_engine_state == APP_ENGINE_RUNNING)
  {
    uint32_t delta_ms = now_ms - app_last_fuel_ms;
    app_last_fuel_ms = now_ms;

    if ((app_mixture_pc < 5U) || (app_fuel_permille == 0U))
    {
      app_engine_state = APP_ENGINE_STUTTERING;
      app_engine_stutter_until_ms = now_ms + APP_ENGINE_STOP_DELAY_MS;
      APP_RequestMessage("STOP", APP_MESSAGE_MS, 0U);
    }
    else
    {
      app_fuel_burn_acc += delta_ms * (uint32_t)app_throttle_pc * (uint32_t)app_mixture_pc;

      while ((app_fuel_burn_acc >= APP_FUEL_BURN_DENOMINATOR) && (app_fuel_permille != 0U))
      {
        app_fuel_burn_acc -= APP_FUEL_BURN_DENOMINATOR;
        app_fuel_permille--;
      }

      if (app_fuel_permille <= APP_FUEL_LOW_PERMILLE)
      {
        if (now_ms >= app_next_low_fuel_stutter_ms)
        {
          app_low_fuel_stutter_until_ms = now_ms + APP_LOW_FUEL_STUTTER_MS;
          app_next_low_fuel_stutter_ms = now_ms + APP_RandRange(2000U, 5000U);
        }
      }
      else
      {
        app_low_fuel_stutter_until_ms = 0U;
        app_next_low_fuel_stutter_ms = now_ms + APP_RandRange(2000U, 5000U);
      }
    }
  }

  if ((app_engine_state == APP_ENGINE_STUTTERING) && (now_ms >= app_engine_stutter_until_ms))
  {
    APP_StopEngine();
  }

  if ((app_inertial_calibrated != 0U) && (app_engine_state == APP_ENGINE_STOPPED))
  {
    if (app_sensors.accel_valid != 0U)
    {
      int32_t dx = APP_Abs32(app_sensors.accel.x - app_accel_ref.x);
      int32_t dy = APP_Abs32(app_sensors.accel.y - app_accel_ref.y);
      int32_t dz = APP_Abs32(app_sensors.accel.z - app_accel_ref.z);

      if ((dx > APP_INERTIAL_DELTA_MG) || (dy > APP_INERTIAL_DELTA_MG) || (dz > APP_INERTIAL_DELTA_MG))
      {
        lifted = 1U;
      }
    }

    if (app_sensors.pressure_valid != 0U)
    {
      if ((app_pressure_ref_hpa - app_sensors.pressure_hpa_raw) > APP_PRESSURE_LIFT_DELTA_HPA)
      {
        lifted = 1U;
      }
    }
  }

  app_pose_alert = lifted;
}

static void APP_DisplayTask(uint32_t now_ms)
{
  uint8_t brightness = APP_NORMAL_BRIGHTNESS;

  if ((now_ms - app_last_display_ms) < APP_DISPLAY_PERIOD_MS)
  {
    return;
  }
  app_last_display_ms = now_ms;

  if (app_pose_alert != 0U)
  {
    MAX7219_SetBrightness(APP_FULL_BRIGHTNESS);
    app_display_brightness = APP_FULL_BRIGHTNESS;

    if (((now_ms / 250U) % 2U) == 0U)
    {
      APP_DisplayWindow("POSE", 0U, now_ms);
    }
    else
    {
      MAX7219_Clear();
    }

    return;
  }

  if (app_mode == 8U)
  {
    uint32_t phase = now_ms % 1600U;

    if (phase > 800U)
    {
      phase = 1600U - phase;
    }

    brightness = (uint8_t)(2U + ((phase * 13U) / 800U));
  }

  if (brightness != app_display_brightness)
  {
    MAX7219_SetBrightness((char)brightness);
    app_display_brightness = brightness;
  }

  if (now_ms < app_message_until_ms)
  {
    APP_DisplayWindow(app_message_text, app_message_scroll, now_ms);
    return;
  }

  APP_UpdateModeText();
  APP_DisplayWindow(app_mode_text, (strlen(app_mode_text) > 8U) ? 1U : 0U, now_ms);
}

static void APP_OutputTask(uint32_t now_ms)
{
  uint8_t mask = 0U;

  if (now_ms < app_mode_pulse_until_ms)
  {
    mask = (uint8_t)(1U << app_mode_pulse_led);
    APP_SetLedMask(APP_ApplyLowFuelLed(mask, now_ms));
    return;
  }

  switch (app_mode)
  {
    case 3U:
    {
      uint32_t mag = (uint32_t)APP_Abs32(app_sensors.magnetic_mgauss);
      uint8_t level = (mag >= 800U) ? 8U : (uint8_t)((mag + 99U) / 100U);
      mask = APP_LedLevelToMask(level);
      break;
    }

    case 4U:
      mask = APP_LedLevelToMask(APP_PercentToLedLevel((uint8_t)app_sensors.humidity_pc));
      break;

    case 5U:
      mask = APP_LedLevelToMask(APP_PercentToLedLevel((uint8_t)(app_fuel_permille / 10U)));
      break;

    case 6U:
      mask = (app_gear_down != 0U) ? 0xFFU : 0x00U;
      break;

    case 7U:
      if (app_flaps == APP_FLAPS_DOWN)
      {
        mask = APP_LedLevelToMask(8U);
      }
      else if (app_flaps == APP_FLAPS_MID)
      {
        mask = APP_LedLevelToMask(4U);
      }
      else
      {
        mask = APP_LedLevelToMask(0U);
      }
      break;

    default:
      mask = 0U;
      break;
  }

  APP_SetLedMask(APP_ApplyLowFuelLed(mask, now_ms));
}

static void APP_BuzzerTask(uint32_t now_ms)
{
  uint16_t freq = 0U;

  if (now_ms < app_beep_until_ms)
  {
    APP_SetBuzzer(app_beep_freq_hz, 50U);
    return;
  }

  if (app_pose_alert != 0U)
  {
    if (((now_ms / 250U) % 2U) == 0U)
    {
      APP_SetBuzzer(1500U, 50U);
    }
    else
    {
      APP_SetBuzzer(0U, 0U);
    }
    return;
  }

  if ((app_engine_state == APP_ENGINE_STUTTERING) || (now_ms < app_low_fuel_stutter_until_ms))
  {
    if (((now_ms / 140U) % 2U) == 0U)
    {
      APP_SetBuzzer(APP_EngineFrequency(), 35U);
    }
    else
    {
      APP_SetBuzzer(0U, 0U);
    }
    return;
  }

  if (app_engine_state == APP_ENGINE_RUNNING)
  {
    freq = APP_EngineFrequency();
    APP_SetBuzzer(freq, 30U);
  }
  else
  {
    APP_SetBuzzer(0U, 0U);
  }
}

static void APP_UpdateModeText(void)
{
  switch (app_mode)
  {
    case 1U:
      snprintf(app_mode_text, sizeof(app_mode_text), "TMP=%ldC", (long)app_sensors.temperature_c);
      break;

    case 2U:
      snprintf(app_mode_text, sizeof(app_mode_text), "PRES=%ldHPA", (long)app_sensors.pressure_hpa);
      break;

    case 3U:
      snprintf(app_mode_text, sizeof(app_mode_text), "MAG=%ld", (long)app_sensors.magnetic_mgauss);
      break;

    case 4U:
      snprintf(app_mode_text, sizeof(app_mode_text), "HUMI=%ldPC", (long)app_sensors.humidity_pc);
      break;

    case 5U:
      snprintf(app_mode_text, sizeof(app_mode_text), "RESERVE=%uPC", (unsigned int)(app_fuel_permille / 10U));
      break;

    case 6U:
      snprintf(app_mode_text, sizeof(app_mode_text), "%s", (app_gear_down != 0U) ? "BAS" : "HAUT");
      break;

    case 7U:
      if (app_flaps == APP_FLAPS_DOWN)
      {
        snprintf(app_mode_text, sizeof(app_mode_text), "DOWN");
      }
      else if (app_flaps == APP_FLAPS_MID)
      {
        snprintf(app_mode_text, sizeof(app_mode_text), "MID");
      }
      else
      {
        snprintf(app_mode_text, sizeof(app_mode_text), "UP");
      }
      break;

    case 8U:
    default:
      snprintf(app_mode_text, sizeof(app_mode_text), "ISEN");
      break;
  }
}

static void APP_DisplayWindow(const char *text, uint8_t scroll, uint32_t now_ms)
{
  size_t len = strlen(text);

  if ((scroll == 0U) || (len <= 8U))
  {
    for (uint8_t i = 0U; i < 8U; i++)
    {
      char c = (i < len) ? text[i] : ' ';
      APP_Display7Seg_WriteChar(i, c);
    }
    return;
  }

  if ((now_ms - app_last_scroll_ms) >= APP_SCROLL_PERIOD_MS)
  {
    app_last_scroll_ms = now_ms;
    app_scroll_index++;

    if (app_scroll_index > (len + 8U))
    {
      app_scroll_index = 0U;
    }
  }

  for (uint8_t i = 0U; i < 8U; i++)
  {
    int16_t src = (int16_t)app_scroll_index + (int16_t)i - 8;
    char c = ' ';

    if ((src >= 0) && ((size_t)src < len))
    {
      c = text[src];
    }

    APP_Display7Seg_WriteChar(i, c);
  }
}

static void APP_ResetDisplayScroll(void)
{
  app_scroll_index = 0U;
  app_last_scroll_ms = 0U;
}

static void APP_RequestMessage(const char *text, uint32_t duration_ms, uint8_t scroll)
{
  strncpy(app_message_text, text, APP_TEXT_MAX - 1U);
  app_message_text[APP_TEXT_MAX - 1U] = '\0';
  app_message_until_ms = HAL_GetTick() + duration_ms;
  app_message_scroll = scroll;
  APP_ResetDisplayScroll();
}

static void APP_SetMode(uint8_t mode, uint32_t now_ms)
{
  if (mode == 0U)
  {
    mode = 1U;
  }
  else if (mode > APP_MODE_COUNT)
  {
    mode = APP_MODE_COUNT;
  }

  app_mode = mode;
  app_mode_pulse_led = (uint8_t)(mode - 1U);
  app_mode_pulse_until_ms = now_ms + APP_MODE_PULSE_MS;
  APP_ResetDisplayScroll();
}

static void APP_SetLedMask(uint8_t mask)
{
  HAL_GPIO_WritePin(L0_GPIO_Port, L0_Pin, ((mask & 0x01U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, ((mask & 0x02U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, ((mask & 0x04U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, ((mask & 0x08U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin, ((mask & 0x10U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(L5_GPIO_Port, L5_Pin, ((mask & 0x20U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(L6_GPIO_Port, L6_Pin, ((mask & 0x40U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(L7_GPIO_Port, L7_Pin, ((mask & 0x80U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t APP_LedLevelToMask(uint8_t level)
{
  uint8_t mask = 0U;

  if (level > 8U)
  {
    level = 8U;
  }

  for (uint8_t i = 0U; i < level; i++)
  {
    mask |= (uint8_t)(1U << i);
  }

  return mask;
}

static uint8_t APP_ApplyLowFuelLed(uint8_t mask, uint32_t now_ms)
{
  if ((app_engine_state == APP_ENGINE_RUNNING) && (app_fuel_permille <= APP_FUEL_LOW_PERMILLE))
  {
    uint32_t period_ms = 150U + ((uint32_t)app_fuel_permille * 8U);

    if (((now_ms / period_ms) % 2U) == 0U)
    {
      mask |= 0x80U;
    }
    else
    {
      mask &= (uint8_t)~0x80U;
    }
  }

  return mask;
}

static void APP_SetBuzzer(uint16_t freq_hz, uint8_t duty_pc)
{
  uint32_t timer_clk;
  uint32_t prescaler;
  uint32_t period;
  uint32_t pulse;

  if ((freq_hz == 0U) || (duty_pc == 0U))
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
    return;
  }

  if (duty_pc > 90U)
  {
    duty_pc = 90U;
  }

  timer_clk = HAL_RCC_GetPCLK1Freq();
  prescaler = (uint32_t)htim3.Init.Prescaler + 1U;
  period = timer_clk / (prescaler * (uint32_t)freq_hz);

  if (period < 2U)
  {
    period = 2U;
  }

  period -= 1U;
  pulse = (period * (uint32_t)duty_pc) / 100U;

  __HAL_TIM_SET_AUTORELOAD(&htim3, period);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

static void APP_Beep(uint16_t freq_hz, uint32_t duration_ms)
{
  app_beep_freq_hz = freq_hz;
  app_beep_until_ms = HAL_GetTick() + duration_ms;
}

static uint16_t APP_EngineFrequency(void)
{
  uint32_t freq = 180U + ((uint32_t)app_throttle_pc * 8U) + ((uint32_t)app_mixture_pc * 5U);

  if (freq > 1800U)
  {
    freq = 1800U;
  }

  return (uint16_t)freq;
}

static uint8_t APP_PercentToLedLevel(uint8_t pc)
{
  if (pc > 100U)
  {
    pc = 100U;
  }

  if (pc == 0U)
  {
    return 0U;
  }

  return (uint8_t)(((uint16_t)pc * 8U + 99U) / 100U);
}

static int32_t APP_FloatToInt(float value)
{
  if (value >= 0.0f)
  {
    return (int32_t)(value + 0.5f);
  }

  return (int32_t)(value - 0.5f);
}

static int32_t APP_Abs32(int32_t value)
{
  return (value < 0L) ? -value : value;
}

static uint32_t APP_RandRange(uint32_t min_value, uint32_t max_value)
{
  uint32_t span = (max_value - min_value) + 1U;

  app_random_state = (app_random_state * 1664525UL) + 1013904223UL;

  return min_value + (app_random_state % span);
}

static void HAL_GPIO_EXTI_DebouncedEvent(uint32_t *last_ms, volatile uint8_t *event)
{
  uint32_t now_ms = HAL_GetTick();

  if ((now_ms - *last_ms) >= APP_DEBOUNCE_MS)
  {
    *event = 1U;
    *last_ms = now_ms;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BTN1_Pin)
  {
    HAL_GPIO_EXTI_DebouncedEvent(&app_last_btn1_irq_ms, &app_evt_btn1);
  }
  else if (GPIO_Pin == BTN2_Pin)
  {
    HAL_GPIO_EXTI_DebouncedEvent(&app_last_btn2_irq_ms, &app_evt_btn2);
  }
  else if (GPIO_Pin == BTN3_Pin)
  {
    HAL_GPIO_EXTI_DebouncedEvent(&app_last_btn3_irq_ms, &app_evt_btn3);
  }
  else if (GPIO_Pin == BTN4_Pin)
  {
    HAL_GPIO_EXTI_DebouncedEvent(&app_last_btn4_irq_ms, &app_evt_btn4);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    app_tim6_1s = 1U;
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
  APP_Trace_Init();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
#if APP_DISPLAY_DIAG_ONLY
  APP_Display7Seg_Init();
  printf("Test 2 : affichage 12345678\n");
  APP_Display7Seg_ShowText("12345678");
  HAL_Delay(APP_DISPLAY_TEXT_DELAY_MS);
  printf("Test 3 : affichage ISEN\n");
  APP_Display7Seg_ShowText("ISEN");
  HAL_Delay(APP_DISPLAY_TEXT_DELAY_MS);
  printf("Boucle diagnostic active\n");
#else
  MAX7219_Init();
  (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  (void)HAL_TIM_Base_Start_IT(&htim6);

  APP_OutputSelfTest();
  APP_IKS01A3_Init();
  APP_ReadAdc();
  APP_ReadSensors();
  APP_CalibrateInertial();
  APP_ResetDisplayScroll();
  APP_RequestMessage("ISEN", APP_MESSAGE_MS, 0U);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if APP_DISPLAY_DIAG_ONLY
    printf("Boucle : ISEN\n");
    APP_Display7Seg_ShowText("ISEN");
    HAL_Delay(APP_DISPLAY_TEXT_DELAY_MS);
    printf("Boucle : 12345678\n");
    APP_Display7Seg_ShowText("12345678");
    HAL_Delay(APP_DISPLAY_TEXT_DELAY_MS);
    printf("Boucle : defilement\n");
    APP_Display7Seg_ScrollText(" ISEN TMP=23C PRES=1013HPA HUMI=50PC ");
#else
    APP_Process();
    HAL_Delay(5U);
#endif
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 2;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 32;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 32000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L0_Pin|L1_Pin|L2_Pin|L3_Pin
                          |L4_Pin|L5_Pin|L6_Pin|L7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
n : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN4_Pin BTN3_Pin */
  GPIO_InitStruct.Pin = BTN4_Pin|BTN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : L0_Pin L1_Pin L2_Pin L3_Pin
                           L4_Pin L5_Pin L6_Pin L7_Pin */
  GPIO_InitStruct.Pin = L0_Pin|L1_Pin|L2_Pin|L3_Pin
                          |L4_Pin|L5_Pin|L6_Pin|L7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN1_Pin BTN2_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin|BTN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
#ifdef USE_FULL_ASSERT
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
