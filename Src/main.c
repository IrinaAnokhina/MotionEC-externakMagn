/**
 ******************************************************************************
 * @file    main.c
 * @author  MEMS Software Solutions Team
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
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

/**
 * @mainpage Documentation for MotionEC package of X-CUBE-MEMS1 Software for
 * X-NUCLEO-IKS01A2 expansion board
 *
 * @image html st_logo.png
 *
 * <b>Introduction</b>
 *
 * MotionEC software is an add-on for the X-CUBE-MEMS1 software and provides
 * E-Compass functionality.
 * The expansion is built on top of STM32Cube software technology that eases
 * portability across different STM32 microcontrollers.
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "main.h"
#include "com.h"
#include "DemoSerial.h"
#include "MotionMC_Manager.h"
#include "MotionEC_Manager.h"
#include "LSM9DS1.h"
#include "i2c.h"
/** @addtogroup MOTION_APPLICATIONS MOTION APPLICATIONS
 * @{
 */

/** @addtogroup E_COMPASS E_COMPASS
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
uint32_t SensorsEnabled = 0;
TIM_HandleTypeDef AlgoTimHandle;

IKS01A2_MOTION_SENSOR_Axes_t AccValue;     /* Raw accelerometer data [mg] */
IKS01A2_MOTION_SENSOR_Axes_t MagValue;     /* Raw magnetometer data [mGauss] */
IKS01A2_MOTION_SENSOR_Axes_t MagValueComp; /* Compensated magnetometer data [mGauss] */

int UseLSI = 0;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint8_t SensorReadRequest = 0;

static int RtcSynchPrediv;
static RTC_HandleTypeDef RtcHandle;
#define ADDR 0x3D
/* Private function prototypes -----------------------------------------------*/
static void RTC_Config(void);
static void RTC_TimeStampConfig(void);
static void Init_Sensors(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM_ALGO_Init(void);
static void RTC_Handler(TMsg *Msg);
static void Accelero_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Gyro_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Magneto_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Pressure_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Humidity_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Temperature_Sensor_Handler(TMsg *Msg, uint32_t Instance);
void sensors_enable();
extern I2C_HandleTypeDef hi2c3;
 HAL_StatusTypeDef status;
/* Public functions ----------------------------------------------------------*/
/**
 * @brief  Main function is to show how to use X_NUCLEO_IKS01A2
 *         expansion board to perform E-Compass functionality and send it from a Nucleo
 *         board to a connected PC, using UART, displaying it on Unicleo-GUI
 *         application, developed by STMicroelectronics.
 *         After connection has been established with GUI, the user can visualize
 *         the data and save datalog for offline analysis.
 *         See User Manual for details.
 * @param  None
 * @retval None
 */
int main(void); /* This "redundant" line is here to fulfil MISRA C-2012 rule 8.4 */
int main(void)
{
  char lib_version[35];
  int lib_version_len;
  TMsg msg_dat;
  TMsg msg_cmd;

  /* STM32xxxx HAL library initialization:
   *   - Configure the Flash prefetch, instruction and Data caches
   *   - Configure the Systick to generate an interrupt each 1 msec
   *   - Set NVIC Group Priority to 4
   *   - Global MSP (MCU Support Package) initialization
   */
  (void)HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the SysTick IRQ priority - set the second lowest priority */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0x0E, 0);

  /* Initialize GPIOs */
  MX_GPIO_Init();

  /* Initialize CRC */
  MX_CRC_Init();

  /* Initialize (disabled) Sensors */
	 MX_I2C2_Init();
	 // while(1)
 //{
	 init_LSM9DS1_I2C ();
	 //status = HAL_I2C_IsDeviceReady(&hi2c3, ADDR, (uint32_t)3, (uint32_t)1000);
/*	 if(status != HAL_OK)
	 {
		 I2C3->CR1 |= I2C_CR1_SWRST;
		 MX_I2C2_Init();
	 HAL_Delay(10);
	 }*/
	// HAL_Delay(10);
 //}
  Init_Sensors();

  /* Magnetometer Calibration API initialization function */
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
  MotionMC_manager_init((int)ALGO_PERIOD, 1);

#elif (defined (USE_STM32L0XX_NUCLEO))
  MotionMC_manager_init((int)ALGO_PERIOD, MMC_CM0P_HI_AND_SI, 1);

#else
#error Not supported platform
#endif

  /* E-Compass API initialization function */
  MotionEC_manager_init((float)ALGO_FREQ);

  /* OPTIONAL */
  /* Get library version */
  MotionEC_manager_get_version(lib_version, &lib_version_len);

  /* Initialize Communication Peripheral for data log */
  USARTConfig();

  /* RTC Initialization */
  RTC_Config();
  RTC_TimeStampConfig();

  /* Timer for algorithm synchronization initialization */
  MX_TIM_ALGO_Init();

  /* LED Blink */
  BSP_LED_On(LED2);
  HAL_Delay(500);
  BSP_LED_Off(LED2);
	
 

 init_LSM9DS1_I2C ();
 //sensors_enable();
  for (;;)
  {
    if (UART_ReceivedMSG((TMsg *)&msg_cmd) != 1)
    {
      if (msg_cmd.Data[0] == DEV_ADDR)
      {
        (void)HandleMSG((TMsg *)&msg_cmd);
      }
    }

    if (SensorReadRequest == 1U)
    {
      if (DataLoggerActive == 1U)
      {
        SensorReadRequest = 0;

        /* Acquire data from enabled sensors and fill Msg stream */
        RTC_Handler(&msg_dat);
        Accelero_Sensor_Handler(&msg_dat, IKS01A2_LSM303AGR_ACC_0);
        Gyro_Sensor_Handler(&msg_dat, IKS01A2_LSM6DSL_0);
        Magneto_Sensor_Handler(&msg_dat, IKS01A2_LSM303AGR_MAG_0);
        Humidity_Sensor_Handler(&msg_dat, IKS01A2_HTS221_0);
        Temperature_Sensor_Handler(&msg_dat, IKS01A2_HTS221_0);
        Pressure_Sensor_Handler(&msg_dat, IKS01A2_LPS22HB_0);

        /* E-Compass specific part */
        MotionEC_manager_run(&msg_dat);

        /* Send data stream */
        INIT_STREAMING_HEADER(&msg_dat);
        msg_dat.Len = STREAMING_MSG_LENGTH;
        UART_SendMsg(&msg_dat);
      }
    }
  }
}

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void Init_Sensors(void)
{
  (void)IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO);
  (void)IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM6DSL_0, MOTION_GYRO);
  (void)IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO);
  (void)IKS01A2_ENV_SENSOR_Init(IKS01A2_HTS221_0, ENV_TEMPERATURE | ENV_HUMIDITY);
  (void)IKS01A2_ENV_SENSOR_Init(IKS01A2_LPS22HB_0, ENV_PRESSURE);

  /* Set accelerometer:
   *   - ODR >= 100Hz
   *   - FS   = <-2g, 2g>
   */
  (void)IKS01A2_MOTION_SENSOR_SetOutputDataRate(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO, 100.0f);
  (void)IKS01A2_MOTION_SENSOR_SetFullScale(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO, 2);

  /* Set magnetometer:
   *   - ODR >= 100Hz
   *   - FS   = 50Gauss (always)
   */
  (void)IKS01A2_MOTION_SENSOR_SetOutputDataRate(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO, 100.0f);
}

/**
 * @brief  GPIO init function.
 * @param  None
 * @retval None
 * @details GPIOs initialized are User LED(PA5) and User Push Button(PC1)
 */
static void MX_GPIO_Init(void)
{
  /* Initialize LED */
  BSP_LED_Init(LED2);

  /* Initialize push button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
}

/**
 * @brief  CRC init function.
 * @param  None
 * @retval None
 */
static void MX_CRC_Init(void)
{
  __CRC_CLK_ENABLE();
}

/**
 * @brief  TIM_ALGO init function.
 * @param  None
 * @retval None
 * @details This function intializes the Timer used to synchronize the algorithm.
 */
static void MX_TIM_ALGO_Init(void)
{
#if (defined (USE_STM32F4XX_NUCLEO))
#define CPU_CLOCK  84000000U

#elif (defined (USE_STM32L0XX_NUCLEO))
#define CPU_CLOCK  32000000U

#elif (defined (USE_STM32L1XX_NUCLEO))
#define CPU_CLOCK  32000000U

#elif (defined (USE_STM32L4XX_NUCLEO))
#define CPU_CLOCK  80000000U

#else
#error Not supported platform
#endif

#define TIM_CLOCK  2000U

  const uint32_t prescaler = CPU_CLOCK / TIM_CLOCK - 1U;
  const uint32_t tim_period = TIM_CLOCK / ALGO_FREQ - 1U;

  TIM_ClockConfigTypeDef s_clock_source_config;
  TIM_MasterConfigTypeDef s_master_config;

  AlgoTimHandle.Instance           = TIM_ALGO;
  AlgoTimHandle.Init.Prescaler     = prescaler;
  AlgoTimHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
  AlgoTimHandle.Init.Period        = tim_period;
  AlgoTimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  (void)HAL_TIM_Base_Init(&AlgoTimHandle);

  s_clock_source_config.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  (void)HAL_TIM_ConfigClockSource(&AlgoTimHandle, &s_clock_source_config);

  s_master_config.MasterOutputTrigger = TIM_TRGO_RESET;
  s_master_config.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  (void)HAL_TIMEx_MasterConfigSynchronization(&AlgoTimHandle, &s_master_config);
}

/**
 * @brief  Handles the time+date getting/sending
 * @param  Msg the time+date part of the stream
 * @retval None
 */
static void RTC_Handler(TMsg *Msg)
{
  uint8_t sub_sec;
  uint32_t ans_uint32;
  int32_t ans_int32;
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructure;

  (void)HAL_RTC_GetTime(&RtcHandle, &stimestructure, FORMAT_BIN);
  (void)HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, FORMAT_BIN);

  /* To be MISRA C-2012 compliant the original calculation:
     sub_sec = ((((((int)RtcSynchPrediv) - ((int)stimestructure.SubSeconds)) * 100) / (RtcSynchPrediv + 1)) & 0xFF);
     has been split to separate expressions */
  ans_int32 = (RtcSynchPrediv - (int32_t)stimestructure.SubSeconds) * 100;
  ans_int32 /= RtcSynchPrediv + 1;
  ans_uint32 = (uint32_t)ans_int32 & 0xFFU;
  sub_sec = (uint8_t)ans_uint32;

  Msg->Data[3] = (uint8_t)stimestructure.Hours;
  Msg->Data[4] = (uint8_t)stimestructure.Minutes;
  Msg->Data[5] = (uint8_t)stimestructure.Seconds;
  Msg->Data[6] = sub_sec;
}

/**
 * @brief  Handles the ACC axes data getting/sending
 * @param  Msg the ACC part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Accelero_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
 if ((SensorsEnabled & ACCELEROMETER_SENSOR) == ACCELEROMETER_SENSOR)
 {
	(void)IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_ACCELERO, &AccValue);
/*	uint8_t data[2] = {0};
	 status = HAL_I2C_Mem_Read(&hi2c3, 0xD6, 0x28, I2C_MEMADD_SIZE_8BIT , data, 0x02, 100); 
	
	int16_t a;
	a=(data[1] << 8)|data[0];
		AccValue.x = a;//(int16_t)data[1] << 8 | data[0];							
		HAL_Delay(10);
		data[0] = 0; data[1] = 0;
	
	
		 status = HAL_I2C_Mem_Read(&hi2c3, 0xD6, 0x2A, I2C_MEMADD_SIZE_8BIT , data, 0x02, 100);
a=(data[1] << 8)|data[0];	
		AccValue.y = a;//(int16_t)data[1] << 8 | data[0];	
		data[0] = 0; data[1] = 0;
		HAL_Delay(10);
		status = HAL_I2C_Mem_Read(&hi2c3, 0xD6, 0x2C, I2C_MEMADD_SIZE_8BIT , data, 0x02, 100); 
		a=(data[1] << 8)|data[0];
		AccValue.z = a;//(int16_t)data[1] << 8 | data[0];	
		HAL_Delay(10);*/
	
    
    Serialize_s32(&Msg->Data[19], (int32_t)AccValue.x, 4);
    Serialize_s32(&Msg->Data[23], (int32_t)AccValue.y, 4);
    Serialize_s32(&Msg->Data[27], (int32_t)AccValue.z, 4);
  }
}

/**
 * @brief  Handles the GYR axes data getting/sending
 * @param  Msg the GYR part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Gyro_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
  IKS01A2_MOTION_SENSOR_Axes_t gyr_value;

  if ((SensorsEnabled & GYROSCOPE_SENSOR) == GYROSCOPE_SENSOR)
  {
    (void)IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_GYRO, &gyr_value);
    Serialize_s32(&Msg->Data[31], gyr_value.x, 4);
    Serialize_s32(&Msg->Data[35], gyr_value.y, 4);
    Serialize_s32(&Msg->Data[39], gyr_value.z, 4);
  }
}

/**
 * @brief  Handles the MAG axes data getting/sending
 * @param  Msg the MAG part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Magneto_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
	uint8_t data[2] = {0};
  //if ((SensorsEnabled & MAGNETIC_SENSOR) == MAGNETIC_SENSOR)
  //{
   // (void)IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_MAGNETO, &MagValue);
		
   status = HAL_I2C_Mem_Read(&hi2c3, ADDR, 0x28, I2C_MEMADD_SIZE_8BIT , data, 0x02, 100); 
	
	int16_t a;
	a=(data[1] << 8)|data[0];
		MagValue.x =  a/10;					
		HAL_Delay(10);
		data[0] = 0; data[1] = 0;
	
	
		 status = HAL_I2C_Mem_Read(&hi2c3, ADDR, 0x2A, I2C_MEMADD_SIZE_8BIT , data, 0x02, 100);
a=(data[1] << 8)|data[0];	
		MagValue.y = a/10;
		data[0] = 0; data[1] = 0;
		HAL_Delay(10);
		status = HAL_I2C_Mem_Read(&hi2c3, ADDR, 0x2C, I2C_MEMADD_SIZE_8BIT , data, 0x02, 100); 
		a=(data[1] << 8)|data[0];
		MagValue.z =  a/10;  	
		HAL_Delay(10);
    /* Magnetometer calibration */
    MotionMC_manager_run(Msg);
	//}
}

/**
 * @brief  Handles the PRESS sensor data getting/sending.
 * @param  Msg the PRESS part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Pressure_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
  float pres_value;

  if ((SensorsEnabled & PRESSURE_SENSOR) == PRESSURE_SENSOR)
  {
    (void)IKS01A2_ENV_SENSOR_GetValue(Instance, ENV_PRESSURE, &pres_value);
    (void)memcpy(&Msg->Data[7], (void *)&pres_value, sizeof(float));
  }
}

/**
 * @brief  Handles the TEMP axes data getting/sending
 * @param  Msg the TEMP part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Temperature_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
  float temp_value;

  if ((SensorsEnabled & TEMPERATURE_SENSOR) == TEMPERATURE_SENSOR)
  {
    (void)IKS01A2_ENV_SENSOR_GetValue(Instance, ENV_TEMPERATURE, &temp_value);
    (void)memcpy(&Msg->Data[11], (void *)&temp_value, sizeof(float));
  }
}

/**
 * @brief  Handles the HUM axes data getting/sending
 * @param  Msg the HUM part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Humidity_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
  float hum_value;

  if ((SensorsEnabled & HUMIDITY_SENSOR) == HUMIDITY_SENSOR)
  {
    (void)IKS01A2_ENV_SENSOR_GetValue(Instance, ENV_HUMIDITY, &hum_value);
    (void)memcpy(&Msg->Data[15], (void *)&hum_value, sizeof(float));;
  }
}

/**
 * @brief  Configures the RTC
 * @param  None
 * @retval None
 */
static void RTC_Config(void)
{
  /*##-1- Configure the RTC peripheral #######################################*/
  /* Check if LSE can be used */
  RCC_OscInitTypeDef rcc_osc_init_struct;

  /*##-2- Configure LSE as RTC clock soucre ###################################*/
  rcc_osc_init_struct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  rcc_osc_init_struct.PLL.PLLState   = RCC_PLL_NONE;
  rcc_osc_init_struct.LSEState       = RCC_LSE_ON;
  rcc_osc_init_struct.LSIState       = RCC_LSI_OFF;

  if (HAL_RCC_OscConfig(&rcc_osc_init_struct) != HAL_OK)
  {
    /* LSE not available, we use LSI */
    UseLSI = 1;
    RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV_LSI;
    RtcHandle.Init.SynchPrediv  = RTC_SYNCH_PREDIV_LSI;
    RtcSynchPrediv = RTC_SYNCH_PREDIV_LSI;
  }
  else
  {
    /* We use LSE */
    UseLSI = 0;
    RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV_LSE;
    RtcHandle.Init.SynchPrediv  = RTC_SYNCH_PREDIV_LSE;
    RtcSynchPrediv = RTC_SYNCH_PREDIV_LSE;
  }

  RtcHandle.Instance = RTC;

  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follow:
       - Hour Format    = Format 12
       - Asynch Prediv  = Value according to source clock
       - Synch Prediv   = Value according to source clock
       - OutPut         = Output Disable
       - OutPutPolarity = High Polarity
       - OutPutType     = Open Drain
   */
  RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_12;
  RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;

  if (HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
 * @brief  Configures the current time and date
 * @param  None
 * @retval None
 */
static void RTC_TimeStampConfig(void)
{
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;

  /* Configure the Date */
  /* Set Date: Monday January 1st 2001 */
  sdatestructure.Year    = 0x01;
  sdatestructure.Month   = RTC_MONTH_JANUARY;
  sdatestructure.Date    = 0x01;
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;

  if (HAL_RTC_SetDate(&RtcHandle, &sdatestructure, FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Configure the Time */
  /* Set Time: 00:00:00 */
  stimestructure.Hours          = 0x00;
  stimestructure.Minutes        = 0x00;
  stimestructure.Seconds        = 0x00;
  stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
 * @brief  Configures the current date
 * @param  y the year value to be set
 * @param  m the month value to be set
 * @param  d the day value to be set
 * @param  dw the day-week value to be set
 * @retval None
 */
void RTC_DateRegulate(uint8_t y, uint8_t m, uint8_t d, uint8_t dw)
{
  RTC_DateTypeDef sdatestructure;

  sdatestructure.Year    = y;
  sdatestructure.Month   = m;
  sdatestructure.Date    = d;
  sdatestructure.WeekDay = dw;

  if (HAL_RTC_SetDate(&RtcHandle, &sdatestructure, FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
 * @brief  Configures the current time
 * @param  hh the hour value to be set
 * @param  mm the minute value to be set
 * @param  ss the second value to be set
 * @retval None
 */
void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss)
{
  RTC_TimeTypeDef stimestructure;

  stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
  stimestructure.Hours          = hh;
  stimestructure.Minutes        = mm;
  stimestructure.Seconds        = ss;
  stimestructure.SubSeconds     = 0;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
 * @brief  This function is executed in case of error occurrence
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
  for (;;)
  {
    BSP_LED_On(LED2);
    HAL_Delay(100);
    BSP_LED_Off(LED2);
    HAL_Delay(100);
  }
}

/**
 * @brief  EXTI line detection callbacks
 * @param  GpioPin the pin connected to EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GpioPin)
{
}

/**
 * @brief  Period elapsed callback
 * @param  htim pointer to a TIM_HandleTypeDef structure that contains
 *              the configuration information for TIM module.
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM_ALGO)
  {
    SensorReadRequest = 1;
    TimeStamp += ALGO_PERIOD;
  }
}

void sensors_enable()
{
	  /* Start enabled sensors */
     // if ((SensorsEnabled & PRESSURE_SENSOR) == PRESSURE_SENSOR)
     // {
        (void)IKS01A2_ENV_SENSOR_Enable(IKS01A2_LPS22HB_0, ENV_PRESSURE);
     // }

     // if ((SensorsEnabled & TEMPERATURE_SENSOR) == TEMPERATURE_SENSOR)
     // {
        (void)IKS01A2_ENV_SENSOR_Enable(IKS01A2_HTS221_0, ENV_TEMPERATURE);
     // }

     // if ((SensorsEnabled & HUMIDITY_SENSOR) == HUMIDITY_SENSOR)
     // {
        (void)IKS01A2_ENV_SENSOR_Enable(IKS01A2_HTS221_0, ENV_HUMIDITY);
     // }

     // if ((SensorsEnabled & ACCELEROMETER_SENSOR) == ACCELEROMETER_SENSOR)
      //{
        (void)IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO);
     // }

     // if ((SensorsEnabled & GYROSCOPE_SENSOR) == GYROSCOPE_SENSOR)
     // {
        (void)IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM6DSL_0, MOTION_GYRO);
     // }

     // if ((SensorsEnabled & MAGNETIC_SENSOR) == MAGNETIC_SENSOR)
     // {
        (void)IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO);
     // }

      (void)HAL_TIM_Base_Start_IT(&AlgoTimHandle);
      DataLoggerActive = 1;
		}
#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred
 * @param  file pointer to the source file name
 * @param  line assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  for (;;)
  {}
}
#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
