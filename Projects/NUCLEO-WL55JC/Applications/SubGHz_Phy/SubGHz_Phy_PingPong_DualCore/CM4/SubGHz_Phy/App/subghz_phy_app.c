/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    subghz_phy_app.c
  * @author  MCD Application Team
  * @brief   Application of the SubGHz_Phy Middleware
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"
#include "adc.h"

/* USER CODE BEGIN Includes */
#include "stm32_timer.h"
#include "stm32_seq.h"
#include "utilities_def.h"
#include "app_version.h"
#include "mbmuxif_sys.h"
#include <string.h>
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Configurations */
#define ADC_TIMEOUT_MS                10
#define ADC_AVERAGE_SAMPLES           4
#define SENSOR1_ADC_CHANNEL           ADC_CHANNEL_0  /* Final PCB sensor 1 on PA0 */
#define SENSOR2_ADC_CHANNEL           ADC_CHANNEL_1  /* Final PCB sensor 2 on PA1 */
#define SENSOR_PAYLOAD_LENGTH         17             /* "R1:01234,R2:02345" */
#define TX_TIMEOUT_VALUE              3000
#define TX_PERIOD_MS                  2000    /* Transmission interval (2 seconds) */
/* REYAX Frame Format */
#define REYAX_DST_ADDR                51      /* Destination address (RYLR896 receiver) */
#define REYAX_SRC_ADDR                1       /* Source address (STM32 transmitter) */
/*Size of the payload to be sent*/
#define MAX_APP_BUFFER_SIZE          255
#if (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE)
#error PAYLOAD_LEN must be less or equal than MAX_APP_BUFFER_SIZE
#endif /* (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE) */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;

/* USER CODE BEGIN PV */
/* App Tx Buffer*/
static uint8_t BufferTx[MAX_APP_BUFFER_SIZE];
/* Transmission timer */
static UTIL_TIMER_Object_t timerTx;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone(void);

/**
  * @brief Function to be executed on Radio Rx Done event
  * @param  payload ptr of buffer received
  * @param  size buffer size
  * @param  rssi
  * @param  LoraSnr_FskCfo
  */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);

/**
  * @brief Function executed on Radio Tx Timeout event
  */
static void OnTxTimeout(void);

/**
  * @brief Function executed on Radio Rx Timeout event
  */
static void OnRxTimeout(void);

/**
  * @brief Function executed on Radio Rx Error event
  */
static void OnRxError(void);

/* USER CODE BEGIN PFP */
/**
  * @brief  Function executed when transmission timer elapses
  * @param  context ptr of timer context
  */
static void OnTxTimerEvent(void *context);

/**
  * @brief Sequencer task for periodic TX (runs in main context, not IRQ context)
  */
static void TxTask(void);

/**
  * @brief Configure and send REYAX frame
  */
static void RadioSend(void);

/**
  * @brief Read one raw ADC channel value
  * @param channel ADC channel to sample
  * @return 12-bit ADC conversion result, or 0 on error
  */
static uint16_t ADC_ReadChannelRaw(uint32_t channel);

/**
  * @brief Format REYAX frame with both ADC readings
  * @param sensor1Value Raw ADC value for sensor 1
  * @param sensor2Value Raw ADC value for sensor 2
  * @param buffer Output buffer for REYAX frame
  * @return Total frame length including header
  */
static uint8_t FormatReyaxFrame(uint16_t sensor1Value, uint16_t sensor2Value, uint8_t *buffer);

/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */
  FEAT_INFO_Param_t *p_cm0plus_specific_features_info;
  uint32_t feature_version = 0UL;

  APP_LOG(TS_OFF, VLEVEL_M, "\n\rREYAX LoRa Transmitter\n\r");
  /* Get CM4 SubGHY_Phy APP version*/
  APP_LOG(TS_OFF, VLEVEL_M, "M4 APP_VERSION:      V%X.%X.%X\r\n",
          (uint8_t)(APP_VERSION_MAIN),
          (uint8_t)(APP_VERSION_SUB1),
          (uint8_t)(APP_VERSION_SUB2));

  /* Get CM0 SubGHY_Phy APP version*/
  p_cm0plus_specific_features_info = MBMUXIF_SystemGetFeatCapabInfoPtr(FEAT_INFO_SYSTEM_ID);
  feature_version = p_cm0plus_specific_features_info->Feat_Info_Feature_Version;
  APP_LOG(TS_OFF, VLEVEL_M, "M0PLUS_APP_VERSION:  V%X.%X.%X\r\n",
          (uint8_t)(feature_version >> 24),
          (uint8_t)(feature_version >> 16),
          (uint8_t)(feature_version >> 8));

  /* Get MW SubGhz_Phy info */
  p_cm0plus_specific_features_info = MBMUXIF_SystemGetFeatCapabInfoPtr(FEAT_INFO_RADIO_ID);
  feature_version = p_cm0plus_specific_features_info->Feat_Info_Feature_Version;
  APP_LOG(TS_OFF, VLEVEL_M, "MW_RADIO_VERSION:    V%X.%X.%X\r\n",
          (uint8_t)(feature_version >> 24),
          (uint8_t)(feature_version >> 16),
          (uint8_t)(feature_version >> 8));

  /* Transmission timer - one-shot; re-started after each send */
  UTIL_TIMER_Create(&timerTx, TX_PERIOD_MS, UTIL_TIMER_ONESHOT, OnTxTimerEvent, NULL);
  /* USER CODE END SubghzApp_Init_1 */

  /* Radio initialization */
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init(&RadioEvents);

  /* USER CODE BEGIN SubghzApp_Init_2 */
  /* Radio Set frequency */
  Radio.SetChannel(RF_FREQUENCY);

  /* Radio configuration */
#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))
  APP_LOG(TS_OFF, VLEVEL_M, "---------------\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_MODULATION\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_BW=%d kHz\n\r", (1 << LORA_BANDWIDTH) * 125);
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_SF=%d\n\r", LORA_SPREADING_FACTOR);
  APP_LOG(TS_OFF, VLEVEL_M, "Frequency=%d Hz\n\r", RF_FREQUENCY);
#elif ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))
  APP_LOG(TS_OFF, VLEVEL_M, "---------------\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "FSK_MODULATION\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "FSK_BW=%d Hz\n\r", FSK_BANDWIDTH);
  APP_LOG(TS_OFF, VLEVEL_M, "FSK_DR=%d bits/s\n\r", FSK_DATARATE);
#else
#error "Please define a modulation in the subghz_phy_app.h file."
#endif /* USE_MODEM_LORA | USE_MODEM_FSK */

  /* Initialize tx buffer*/
  memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);

  /* Register TX task and kick first send */
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), UTIL_SEQ_RFU, TxTask);
  APP_LOG(TS_OFF, VLEVEL_M, "Starting one-way TX every %d ms\n\r", TX_PERIOD_MS);
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */
  APP_LOG(TS_ON, VLEVEL_L, "Tx Done\n\r");
  Radio.Sleep();
  /* USER CODE END OnTxDone */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */
  /* USER CODE END OnRxDone */
}

static void OnTxTimeout(void)
{
  /* USER CODE BEGIN OnTxTimeout */
  APP_LOG(TS_ON, VLEVEL_L, "Tx Timeout\n\r");
  Radio.Sleep();
  /* USER CODE END OnTxTimeout */
}

static void OnRxTimeout(void)
{
  /* USER CODE BEGIN OnRxTimeout */
  /* USER CODE END OnRxTimeout */
}

static void OnRxError(void)
{
  /* USER CODE BEGIN OnRxError */
  /* USER CODE END OnRxError */
}

/* USER CODE BEGIN PrFD */
static void TxTask(void)
{
  /* Send frame in task context */
  RadioSend();

  /* Re-arm one-shot timer for next send */
  UTIL_TIMER_Start(&timerTx);
}

static void RadioSend(void)
{
  uint16_t sensor1Value;
  uint16_t sensor2Value;
  uint8_t frameLength;
  
#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))
  Radio.Standby();
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);
  Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);
#elif ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))
  Radio.Standby();
  Radio.SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                    FSK_DATARATE, 0,
                    FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, 0, TX_TIMEOUT_VALUE);
  Radio.SetMaxPayloadLength(MODEM_FSK, MAX_APP_BUFFER_SIZE);
#else
#error "Please define a modulation in the subghz_phy_app.h file."
#endif /* USE_MODEM_LORA | USE_MODEM_FSK */

  sensor1Value = ADC_ReadChannelRaw(SENSOR1_ADC_CHANNEL);
  sensor2Value = ADC_ReadChannelRaw(SENSOR2_ADC_CHANNEL);
  frameLength = FormatReyaxFrame(sensor1Value, sensor2Value, BufferTx);
  APP_LOG(TS_ON, VLEVEL_L, "ADC1=%d ADC2=%d\n\r", sensor1Value, sensor2Value);
  
  Radio.Send(BufferTx, frameLength);
}

static uint16_t ADC_ReadChannelRaw(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  uint32_t adcSum = 0;
  uint16_t adcValue = 0;
  uint8_t sampleIdx;

  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;

  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    return 0;
  }

  /* Discard the first conversion after switching channels to reduce carry-over. */
  if (HAL_ADC_Start(&hadc) == HAL_OK)
  {
    (void)HAL_ADC_PollForConversion(&hadc, ADC_TIMEOUT_MS);
    (void)HAL_ADC_GetValue(&hadc);
    (void)HAL_ADC_Stop(&hadc);
  }

  for (sampleIdx = 0; sampleIdx < ADC_AVERAGE_SAMPLES; sampleIdx++)
  {
    if (HAL_ADC_Start(&hadc) != HAL_OK)
    {
      return 0;
    }

    if (HAL_ADC_PollForConversion(&hadc, ADC_TIMEOUT_MS) == HAL_OK)
    {
      adcSum += HAL_ADC_GetValue(&hadc);
    }

    (void)HAL_ADC_Stop(&hadc);
  }

  adcValue = (uint16_t)(adcSum / ADC_AVERAGE_SAMPLES);
  return adcValue;
}

/**
  * @brief  Function executed when transmission timer elapses
  * @param  context ptr of timer context
  */
static void OnTxTimerEvent(void *context)
{
  (void)context;
  /* Timer callback may run from RTC/IRQ context: only schedule task */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
}

/**
  * @brief Format REYAX frame with both ADC readings
  * @param sensor1Value Raw ADC value for sensor 1
  * @param sensor2Value Raw ADC value for sensor 2
  * @param buffer Output buffer for REYAX frame
  * @return Total frame length including header
  */
static uint8_t FormatReyaxFrame(uint16_t sensor1Value, uint16_t sensor2Value, uint8_t *buffer)
{
  uint8_t idx = 0;
  uint16_t tempValue;
  uint8_t digit;
  
  /* REYAX Frame Format:
   * Byte 0-1: Destination Address (LSB, MSB) = 0x33, 0x00 (51)
   * Byte 2-3: Source Address (LSB, MSB) = 0x01, 0x00 (1)
   * Byte 4:   Payload Length (N) = 17 for "R1:01234,R2:02345"
   * Byte 5-N: Payload data (ASCII string with both raw ADC readings)
   */
  
  /* Destination Address (LSB, MSB) */
  buffer[idx++] = (uint8_t)(REYAX_DST_ADDR & 0xFF);        /* LSB */
  buffer[idx++] = (uint8_t)((REYAX_DST_ADDR >> 8) & 0xFF); /* MSB */
  
  /* Source Address (LSB, MSB) */
  buffer[idx++] = (uint8_t)(REYAX_SRC_ADDR & 0xFF);        /* LSB */
  buffer[idx++] = (uint8_t)((REYAX_SRC_ADDR >> 8) & 0xFF); /* MSB */
  
  /* Payload Length = 17 bytes for "R1:01234,R2:02345" */
  buffer[idx++] = SENSOR_PAYLOAD_LENGTH;
  
  /* Payload: "R1:01234,R2:02345" */
  buffer[idx++] = 'R';
  buffer[idx++] = '1';
  buffer[idx++] = ':';
  
  tempValue = sensor1Value;
  digit = (uint8_t)((tempValue / 10000) % 10);
  buffer[idx++] = '0' + digit;
  digit = (uint8_t)((tempValue / 1000) % 10);
  buffer[idx++] = '0' + digit;
  digit = (uint8_t)((tempValue / 100) % 10);
  buffer[idx++] = '0' + digit;
  digit = (uint8_t)((tempValue / 10) % 10);
  buffer[idx++] = '0' + digit;
  digit = (uint8_t)(tempValue % 10);
  buffer[idx++] = '0' + digit;

  buffer[idx++] = ',';
  buffer[idx++] = 'R';
  buffer[idx++] = '2';
  buffer[idx++] = ':';

  tempValue = sensor2Value;
  digit = (uint8_t)((tempValue / 10000) % 10);
  buffer[idx++] = '0' + digit;
  digit = (uint8_t)((tempValue / 1000) % 10);
  buffer[idx++] = '0' + digit;
  digit = (uint8_t)((tempValue / 100) % 10);
  buffer[idx++] = '0' + digit;
  digit = (uint8_t)((tempValue / 10) % 10);
  buffer[idx++] = '0' + digit;
  digit = (uint8_t)(tempValue % 10);
  buffer[idx++] = '0' + digit;
  
  return idx; /* Total frame length = 22 bytes */
}

/* USER CODE END PrFD */
