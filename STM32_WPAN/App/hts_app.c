/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    hts_app.c
  * @author  MCD Application Team
  * @brief   Health Thermometer Service Application
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019-2021 STMicroelectronics.
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
#include "app_common.h"
#include "dbg_trace.h"
#include "app_ble.h"
#include "ble.h"
#include "hts_app.h"
#include <time.h>
#include "stm32_seq.h"
#include "stm32wbxx_hal.h"
#include "stm32_lpm.h"
#include "stm32_lpm_if.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  HTS_TemperatureValue_t TemperatureMeasurementChar;
  uint8_t TimerMeasurement_Id;
  uint8_t TimerMeasurementStarted;
} HTSAPP_Context_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
#define DEFAULT_HTS_MEASUREMENT_INTERVAL   (100000/CFG_TS_TICK_VAL)  /**< 1s */
#define DEFAULT_TEMPERATURE_TYPE          TT_Armpit
#define NB_SAVED_MEASURES                                                     10
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static HTSAPP_Context_t HTSAPP_Context;
static HTS_TemperatureValue_t HTSMeasurement[NB_SAVED_MEASURES];
static int8_t HTS_CurrentIndex, HTS_OldIndex;

/**
 * END of Section BLE_APP_CONTEXT
 */

 /* USER CODE BEGIN PV */
volatile uint32_t measurement = 0;
extern ADC_HandleTypeDef hadc1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void HTSAPP_Update_TimeStamp(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
static void HTSAPP_UpdateMeasurement( void )
{
/* USER CODE BEGIN HTSAPP_UpdateMeasurement */
  /**
   * The code shall be executed in the background as aci command may be sent
   * The background is the only place where the application can make sure a new aci command
   * is not sent if there is a pending one
   */
  
  UTIL_SEQ_SetTask( 1<<CFG_TASK_HTS_MEAS_START_REQ_ID,CFG_SCH_PRIO_0);
  
/* USER CODE END HTSAPP_UpdateMeasurement */
  return;
}

#if(BLE_CFG_HTS_TIME_STAMP_FLAG != 0)
static void HTSAPP_Update_TimeStamp(void)
{

  HTSAPP_Context.TemperatureMeasurementChar.TimeStamp.Seconds += 1;
  
  while(HTSAPP_Context.TemperatureMeasurementChar.TimeStamp.Seconds >= 60)
  {
    HTSAPP_Context.TemperatureMeasurementChar.TimeStamp.Seconds -= 60;
    HTSAPP_Context.TemperatureMeasurementChar.TimeStamp.Minutes += 1;
  }
  while(HTSAPP_Context.TemperatureMeasurementChar.TimeStamp.Minutes >= 60)
  {
    HTSAPP_Context.TemperatureMeasurementChar.TimeStamp.Minutes -= 60;
    HTSAPP_Context.TemperatureMeasurementChar.TimeStamp.Hours += 1;
  }
  while(HTSAPP_Context.TemperatureMeasurementChar.TimeStamp.Hours >= 24)
  {
    HTSAPP_Context.TemperatureMeasurementChar.TimeStamp.Hours -= 24;
    HTSAPP_Context.TemperatureMeasurementChar.TimeStamp.Day += 1;
  }
  while(HTSAPP_Context.TemperatureMeasurementChar.TimeStamp.Day >= 31)
  {
    HTSAPP_Context.TemperatureMeasurementChar.TimeStamp.Day -= 31;
    HTSAPP_Context.TemperatureMeasurementChar.TimeStamp.Month += 1;
  }
  while(HTSAPP_Context.TemperatureMeasurementChar.TimeStamp.Month >= 12)
  {
    HTSAPP_Context.TemperatureMeasurementChar.TimeStamp.Month -= 11;
    HTSAPP_Context.TemperatureMeasurementChar.TimeStamp.Year += 1;
  }
/* USER CODE END HTSAPP_Update_TimeStamp */
}
#endif

#if(BLE_CFG_HTS_TIME_STAMP_FLAG != 0)
static void HTSAPP_Store(void)
{
/* USER CODE BEGIN HTSAPP_Store */
  HTS_CurrentIndex++;
  if(HTS_CurrentIndex == NB_SAVED_MEASURES)
  {
    HTS_CurrentIndex = 0;
  }
  if((HTS_CurrentIndex == HTS_OldIndex) && (HTSMeasurement[HTS_CurrentIndex].MeasurementValue > 0))
  {
    HTS_OldIndex = HTS_CurrentIndex + 1;
    if(HTS_OldIndex == NB_SAVED_MEASURES)
    {
      HTS_OldIndex = 0;
    }
  }
  APP_DBG_MSG ("Stored measurement %d, index of first measure saved: %d\n", 
               HTS_CurrentIndex, HTS_OldIndex);  
  memcpy(&HTSMeasurement[HTS_CurrentIndex], 
         &(HTSAPP_Context.TemperatureMeasurementChar), 
         sizeof(HTS_TemperatureValue_t)); 
/* USER CODE END HTSAPP_Store */
}

static void HTSAPP_Suppress(void)
{
/* USER CODE BEGIN HTSAPP_Suppress */
  HTS_Update_Char(TEMPERATURE_MEASUREMENT_CHAR_UUID, 
                  (uint8_t *)&HTSMeasurement[HTS_OldIndex]);
  
  HTSMeasurement[HTS_OldIndex].MeasurementValue = 0;
  HTS_OldIndex++;
  if(HTS_OldIndex == NB_SAVED_MEASURES)
  {
    HTS_OldIndex = 0;
  }
/* USER CODE END HTSAPP_Suppress */
}
#endif

/* Public functions ----------------------------------------------------------*/

void HTS_App_Notification(HTS_App_Notification_evt_t *pNotification)
{
/* USER CODE BEGIN HTS_App_Notification */
  switch(pNotification->HTS_Evt_Opcode)
  {
    case HTS_MEASUREMENT_IND_ENABLED_EVT:
      {
        APP_DBG_MSG("HTS_MEASUREMENT_IND_ENABLED_EVT\n");
        HW_TS_Stop(HTSAPP_Context.TimerMeasurement_Id);
        HW_TS_Start(HTSAPP_Context.TimerMeasurement_Id, 2*DEFAULT_HTS_MEASUREMENT_INTERVAL);
        HTSAPP_Context.TimerMeasurementStarted = 1;
      }
      break;

    case HTS_MEASUREMENT_IND_DISABLED_EVT:
      {
        APP_DBG_MSG("HTS_MEASUREMENT_IND_DISABLED_EVT\n");
        HW_TS_Stop(HTSAPP_Context.TimerMeasurement_Id);
        HTSAPP_Context.TimerMeasurementStarted = 0;
      }
      break;

    default:
      break;
  }

/* USER CODE END HTS_App_Notification */
  return;
}

void HTSAPP_Init(void)
{
/* USER CODE BEGIN HTSAPP_Init */
  UTIL_SEQ_RegTask( 1<< CFG_TASK_HTS_MEAS_START_REQ_ID, UTIL_SEQ_RFU, HTSAPP_Measurement_start);
  UTIL_SEQ_RegTask( 1<< CFG_TASK_HTS_MEAS_PROCESS_REQ_ID, UTIL_SEQ_RFU, HTSAPP_Measurement_process);

  /**
   * Initialize Flags
   */
  HTSAPP_Context.TemperatureMeasurementChar.Flags = (uint8_t)NO_FLAGS;
#if(BLE_CFG_HTS_TIME_STAMP_FLAG != 0)
  HTSAPP_Context.TemperatureMeasurementChar.Flags |= (uint8_t)SENSOR_TIME_STAMP_PRESENT;
#endif
  /**
   * Temperature Type shall be present in the flags when it is NOT static
   */
  HTSAPP_Context.TemperatureMeasurementChar.TemperatureType = DEFAULT_TEMPERATURE_TYPE;
  HTSAPP_Context.TemperatureMeasurementChar.Flags |= (uint8_t)SENSOR_TEMPERATURE_TYPE_PRESENT;


  /**
   * Create timer for Health Temperature Measurement
   */
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(HTSAPP_Context.TimerMeasurement_Id), hw_ts_Repeated, HTSAPP_UpdateMeasurement);
  HTSAPP_Context.TimerMeasurementStarted = 0;
  
  HTS_CurrentIndex = -1;
  HTS_OldIndex = 0;

/* USER CODE END HTSAPP_Init */
return;
}

void HTSAPP_Stop_Timer(void)
{
  APP_DBG_MSG("HTS_MEASUREMENT_IND_DISABLED_EVT\n");
  HW_TS_Stop(HTSAPP_Context.TimerMeasurement_Id);
  HTSAPP_Context.TimerMeasurementStarted = 0;
}
  
void HTSAPP_Measurement_start(void){
  UTIL_LPM_SetOffMode(1 << CFG_LPM_HTS_APP, UTIL_LPM_DISABLE);
  UTIL_LPM_SetStopMode(1 << CFG_LPM_HTS_APP, UTIL_LPM_DISABLE);
  MX_ADC1_Init();
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_IT(&hadc1);
}

void HTSAPP_Measurement_process(void)
{
/* USER CODE BEGIN HTSAPP_Measurement */
  MX_ADC1_Deinit();

  HTSAPP_Context.TemperatureMeasurementChar.MeasurementValue = measurement;
  APP_DBG_MSG("HTSAPP_Measurement: %d \n", (int)measurement);
#if(BLE_CFG_HTS_TIME_STAMP_FLAG != 0)
  HTSAPP_Update_TimeStamp();
#endif

  if((APP_BLE_Get_Server_Connection_Status() == APP_BLE_CONNECTED_SERVER) &&
     (HTSAPP_Context.TimerMeasurementStarted == 1))
  {
#if(BLE_CFG_HTS_TIME_STAMP_FLAG != 0)
    if(HTSMeasurement[HTS_OldIndex].MeasurementValue > 0)
    {
      APP_DBG_MSG ("Send stored measurement %d\n", HTS_OldIndex);  
      HTSAPP_Suppress();
      HW_TS_Stop(HTSAPP_Context.TimerMeasurement_Id);
      HW_TS_Start(HTSAPP_Context.TimerMeasurement_Id, DEFAULT_HTS_MEASUREMENT_INTERVAL);
    }
    else
    {
      HW_TS_Stop(HTSAPP_Context.TimerMeasurement_Id);
      HW_TS_Start(HTSAPP_Context.TimerMeasurement_Id, DEFAULT_HTS_MEASUREMENT_INTERVAL*10);
      HTS_Update_Char(TEMPERATURE_MEASUREMENT_CHAR_UUID, 
                      (uint8_t *)&HTSAPP_Context.TemperatureMeasurementChar);
    }
#endif
  }  
#if(BLE_CFG_HTS_TIME_STAMP_FLAG != 0)
  else
  {
    HTSAPP_Store();
  }
#endif
  UTIL_LPM_SetOffMode(1 << CFG_LPM_HTS_APP, UTIL_LPM_ENABLE);
  UTIL_LPM_SetStopMode(1 << CFG_LPM_HTS_APP, UTIL_LPM_ENABLE);

/* USER CODE END HTSAPP_Measurement */
  return;
}


/* USER CODE BEGIN FD */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  __IO uint16_t ConvertedData = LL_ADC_REG_ReadConversionData12(ADC1);
  measurement = __HAL_ADC_CALC_TEMPERATURE(3300u, ConvertedData, LL_ADC_RESOLUTION_12B);
  UTIL_SEQ_SetTask( 1<<CFG_TASK_HTS_MEAS_PROCESS_REQ_ID,CFG_SCH_PRIO_0);
}
/* USER CODE END FD */
