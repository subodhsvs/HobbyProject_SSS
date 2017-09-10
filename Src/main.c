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
#include "stm32l4xx_hal.h"
#include "crc.h"
#include "dma.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "MotionFX_Manager.h"
#include <math.h>   /* trunc */
#include <stdio.h>  /* snprintf */

#define SAMPLE_FREQ                     ((uint8_t)100)  /* [Hz] */
#define SAMPLE_PERIOD                   ((uint8_t)10)   /* [ms] */
#define MOTIONFX_ENGINE_DELTATIME       0.01f

#define FROM_MG_TO_G                    0.001f
#define FROM_G_TO_MG                    1000.0f
#define FROM_MDPS_TO_DPS                0.001f
#define FROM_DPS_TO_MDPS                1000.0f
#define FROM_MGAUSS_TO_UT50             (0.1f/50.0f)
#define FROM_UT50_TO_MGAUSS             500.0f

#define MAX_BUF_SIZE 256
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private typedef -----------------------------------------------------------*/
typedef struct displayFloatToInt_s {
  int8_t sign; /* 0 means positive, 1 means negative*/
  uint32_t  out_int;
  uint32_t  out_dec;
} displayFloatToInt_t;

/* Public variables Stop---------------------------------------------------------*/
static char dataOut[MAX_BUF_SIZE];
uint8_t gyro_status = 1;
uint8_t DataLoggerActive = 0;
uint32_t Sensors_Enabled = 0;
uint32_t mag_time_stamp = 0;
uint8_t mag_cal_status = 0;
extern uint8_t SF_6X_Enabled;
int use_LSI = 0;

SensorAxes_t ACC_Value;
SensorAxes_t GYR_Value;
SensorAxes_t MAG_Value;
SensorAxes_t MAG_Offset;

void *ACCELERO_handle    = NULL;
void *GYRO_handle        = NULL;
void *MAGNETO_handle     = NULL;
void *TEMPERATURE_handle = NULL;
void *PRESSURE_handle    = NULL;

/* Public variables Stop-----------------------------------------------------*/

static char dataOut[MAX_BUF_SIZE];
volatile static uint8_t change     = 1;

volatile static uint8_t sensor_read_request     = 0;
volatile static uint8_t magcal_request          = 0;
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;

static void Init_Sensors(void);
static void FX_Data_Handler(void);
static void Accelero_Sensor_Handler(void);
static void Gyro_Sensor_Handler(void);
static void Magneto_Sensor_Handler(void);

static void Enable_Sensors(void);
static void Disable_Sensors(void);
static void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  char lib_version[35];
  int lib_version_len;
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
  MX_RTC_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_CRC_Init();

  /* USER CODE BEGIN 2 */
  
  /* Initialize (disabled) Sensors */
  Init_Sensors();
  
  /* Sensor Fusion API initialization function */
  MotionFX_manager_init(GYRO_handle);
  
  /* OPTIONAL */
  /* Get library version */
  MotionFX_manager_get_version(lib_version, &lib_version_len);
      
  /* LED Blink */
  BSP_LED_On(LED2);
  HAL_Delay(500);
  BSP_LED_Off(LED2);
  
  /* Enable magnetometer calibration */
  MotionFX_manager_MagCal_start(SAMPLE_PERIOD);

  /* Test if calibration data are available */
  MFX_MagCal_output_t mag_cal_test;
  MotionFX_MagCal_getParams(&mag_cal_test);

  /* If calibration data are available lood HI coeficients */
  if (mag_cal_test.cal_quality == MFX_MAGCALGOOD)
  {
    MAG_Offset.AXIS_X = (int32_t) (mag_cal_test.hi_bias[0] * FROM_UT50_TO_MGAUSS);
    MAG_Offset.AXIS_Y = (int32_t) (mag_cal_test.hi_bias[1] * FROM_UT50_TO_MGAUSS);
    MAG_Offset.AXIS_Z = (int32_t) (mag_cal_test.hi_bias[2] * FROM_UT50_TO_MGAUSS);

    mag_cal_status = 1;
    BSP_LED_On(LED2);
  }
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  /*############# 3- Start PWM signals generation ##############*/
  /* Start channel 1 */
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  /* Start channel 2 */
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  /* Start channel 3 */
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) != HAL_OK)
  {
    /* PWM generation Error */
    Error_Handler();
  }
  /* Start channel 4 */
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4) != HAL_OK)
  {
    /* PWM generation Error */
    Error_Handler();
  }
  
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    uint8_t instance;

    if (magcal_request)
    {
      magcal_request = 0;

      /* Reset magnetometer calibration value*/
      mag_cal_status = 0;
      MAG_Offset.AXIS_X = 0;
      MAG_Offset.AXIS_Y = 0;
      MAG_Offset.AXIS_Z = 0;

#if (defined (MOTION_FX_STORE_CALIB_FLASH))
      /* Reset values in memory */
      ResetCalibrationInMemory();
#endif

      /* Enable magnetometer calibration */
      MotionFX_manager_MagCal_start(SAMPLE_PERIOD);

      /* Switch off the LED */
      BSP_LED_Off(LED2);
    }
    
    if (change == 1)
    {
      change = 0;
      BSP_GYRO_Get_Instance(GYRO_handle, &instance);
      BSP_MAGNETO_Get_Instance( MAGNETO_handle, &instance );
      BSP_ACCELERO_Get_Instance( ACCELERO_handle, &instance);
      
      /* Start enabled sensors */
      BSP_ACCELERO_Sensor_Enable(ACCELERO_handle);
      BSP_GYRO_Sensor_Enable(GYRO_handle);
      BSP_MAGNETO_Sensor_Enable(MAGNETO_handle);
      HAL_TIM_Base_Start_IT(&htim3);
      MotionFX_manager_stop_6X(); 
      MotionFX_manager_start_9X(); 

    }
    
    if (sensor_read_request)
    {
      sensor_read_request = 0;
 
      Enable_Sensors();
      /* Acquire data from enabled sensors and fill Msg stream */
      Accelero_Sensor_Handler();
      Gyro_Sensor_Handler();
      Magneto_Sensor_Handler();

      /* Sensor Fusion specific part */ 
      FX_Data_Handler();
      __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 500); //update pwm value
    }
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

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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

/* USER CODE BEGIN 4 */
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize all sensors
  * @param  None
  * @retval None
  */
static void Init_Sensors(void)
{
  /* Initialize and Configure all sensors */
  BSP_ACCELERO_Init(LSM6DS0_X_0, &ACCELERO_handle);
  BSP_GYRO_Init(LSM6DS0_G_0, &GYRO_handle);
  BSP_MAGNETO_Init(LIS3MDL_0, &MAGNETO_handle);
}

/**
  * @brief  Enable all sensors
  * @param  None
  * @retval None
  */
static void Enable_Sensors(void)
{
  /* Initialize and Configure all sensors */
      if ( ACCELEROMETER_SENSOR ) 
        BSP_ACCELERO_Sensor_Enable(ACCELERO_handle);
      
      if ( GYROSCOPE_SENSOR ) 
        BSP_GYRO_Sensor_Enable(GYRO_handle);
      
      if ( MAGNETIC_SENSOR ) 
        BSP_MAGNETO_Sensor_Enable(MAGNETO_handle);
     
}

/**
  * @brief  Enable all sensors
  * @param  None
  * @retval None
  */
static void Disable_Sensors(void)
{
  /* Initialize and Configure all sensors */
      if ( ACCELEROMETER_SENSOR ) 
        BSP_ACCELERO_Sensor_Disable(ACCELERO_handle);
     
      if ( GYROSCOPE_SENSOR ) 
        BSP_GYRO_Sensor_Disable(GYRO_handle);
      
      if ( MAGNETIC_SENSOR ) 
        BSP_MAGNETO_Sensor_Disable(MAGNETO_handle);
      
}

/**
  * @brief  Handles the ACC axes data getting/sending
  * @param  Msg - ACC part of the stream
  * @retval None
  */
static void Accelero_Sensor_Handler(void)
{
  uint8_t status = 0;

  if (ACCELEROMETER_SENSOR)
  {
    if (BSP_ACCELERO_IsInitialized(ACCELERO_handle, &status) == COMPONENT_ERROR )
    {
      sprintf( dataOut, "Initialisation ERROR");
      HAL_UART_Transmit( &huart2, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );    
    }
    BSP_ACCELERO_Get_Axes(ACCELERO_handle, &ACC_Value);
  }
}

/**
  * @brief  Handles the GYR axes data getting/sending
  * @param  Msg - GYR part of the stream
  * @retval None
  */
static void Gyro_Sensor_Handler(void)
{
  uint8_t status = 0;

  if (GYROSCOPE_SENSOR)
  {
    if (BSP_GYRO_IsInitialized(GYRO_handle, &status) == COMPONENT_ERROR)
    {
      sprintf( dataOut, "Initialisation ERROR");
      HAL_UART_Transmit( &huart2, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );
    }   
    BSP_GYRO_Get_Axes(GYRO_handle, &GYR_Value);
  }
}


/**
  * @brief  Handles the MAG axes data getting/sending
  * @param  Msg - MAG part of the stream
  * @retval None
  */
static void Magneto_Sensor_Handler(void)
{
  uint8_t status = 0;
  MFX_MagCal_input_t mag_data_in;
  MFX_MagCal_output_t mag_data_out;
    
  if (MAGNETIC_SENSOR)
  {  
    if (BSP_MAGNETO_IsInitialized(MAGNETO_handle, &status) == COMPONENT_ERROR)
    {
      sprintf( dataOut, "Initialisation ERROR");
      HAL_UART_Transmit( &huart2, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );
    }
    BSP_MAGNETO_Get_Axes(MAGNETO_handle, &MAG_Value);
    
//    if (BSP_MAGNETO_IsInitialized(MAGNETO_handle, &status) == COMPONENT_OK && status == 1)
//    {
//      BSP_MAGNETO_Get_Axes(MAGNETO_handle, &MAG_Value);
//       
//      if (mag_cal_status == 0)
//      {
//        mag_data_in.mag[0] = MAG_Value.AXIS_X * FROM_MGAUSS_TO_UT50;
//        mag_data_in.mag[1] = MAG_Value.AXIS_Y * FROM_MGAUSS_TO_UT50;
//        mag_data_in.mag[2] = MAG_Value.AXIS_Z * FROM_MGAUSS_TO_UT50;
//        mag_data_in.time_stamp = mag_time_stamp;
//        mag_time_stamp += SAMPLE_PERIOD;
//
//        MotionFX_manager_MagCal_run(&mag_data_in, &mag_data_out);
//
//        if (mag_data_out.cal_quality == MFX_MAGCALGOOD)
//        {
//          mag_cal_status = 1;
//
//          MAG_Offset.AXIS_X = (int32_t) (mag_data_out.hi_bias[0] * FROM_UT50_TO_MGAUSS);
//          MAG_Offset.AXIS_Y = (int32_t) (mag_data_out.hi_bias[1] * FROM_UT50_TO_MGAUSS);
//          MAG_Offset.AXIS_Z = (int32_t) (mag_data_out.hi_bias[2] * FROM_UT50_TO_MGAUSS);
//
//          /* Disable magnetometer calibration */
//          MotionFX_manager_MagCal_stop(SAMPLE_PERIOD);
//
//          /* Switch on the LED */
//          BSP_LED_On(LED2);
//        }
//      }

//      MAG_Value.AXIS_X = (int32_t) (MAG_Value.AXIS_X - MAG_Offset.AXIS_X);
//      MAG_Value.AXIS_Y = (int32_t) (MAG_Value.AXIS_Y - MAG_Offset.AXIS_Y);
//      MAG_Value.AXIS_Z = (int32_t) (MAG_Value.AXIS_Z - MAG_Offset.AXIS_Z);

//      Serialize_s32(&Msg->Data[43], MAG_Value.AXIS_X, 4);
//      Serialize_s32(&Msg->Data[47], MAG_Value.AXIS_Y, 4);
//      Serialize_s32(&Msg->Data[51], MAG_Value.AXIS_Z, 4);
    }
  
}

/**
  * @brief  Sensor Fusion data handler
  * @param  Msg - Sensor Fusion data part of the stream
  * @retval None
  */
//static void FX_Data_Handler(TMsg *Msg)
static void FX_Data_Handler(void)
{
  MFX_input_t data_in;
  MFX_input_t *pdata_in = &data_in;
  MFX_output_t data_out;
  MFX_output_t *pdata_out = &data_out;
  displayFloatToInt_t quat_0,quat_1,quat_2,quat_3;
  displayFloatToInt_t rot_0,rot_1,rot_2;
  displayFloatToInt_t linAcc_0,linAcc_1,linAcc_2;

  
  if ((ACCELEROMETER_SENSOR) && (GYROSCOPE_SENSOR) && (MAGNETIC_SENSOR))
  {  
    
    sprintf( dataOut, "\n\rAcc:Gyr:Mag:Quat:Rot:LinAcc %06d:%06d:%06d", ACC_Value.AXIS_X, ACC_Value.AXIS_Y, ACC_Value.AXIS_Z );
    HAL_UART_Transmit( &huart2, ( uint8_t *)dataOut, strlen( dataOut ), 5000 );

    sprintf( dataOut, "  %06d:%06d:%06d", GYR_Value.AXIS_X, GYR_Value.AXIS_Y, GYR_Value.AXIS_Z );
    HAL_UART_Transmit( &huart2, ( uint8_t *)dataOut, strlen( dataOut ), 5000 );
    
    sprintf( dataOut, "  %06d:%06d:%06d", MAG_Value.AXIS_X, MAG_Value.AXIS_Y, MAG_Value.AXIS_Z );
    HAL_UART_Transmit( &huart2, ( uint8_t *)dataOut, strlen( dataOut ), 5000 );
    
    if ((ACCELEROMETER_SENSOR) && (GYROSCOPE_SENSOR) && (MAGNETIC_SENSOR))
    {
      data_in.gyro[0] = GYR_Value.AXIS_X  * FROM_MDPS_TO_DPS;
      data_in.gyro[1] = GYR_Value.AXIS_Y  * FROM_MDPS_TO_DPS;
      data_in.gyro[2] = GYR_Value.AXIS_Z  * FROM_MDPS_TO_DPS;

      data_in.acc[0] = ACC_Value.AXIS_X * FROM_MG_TO_G;
      data_in.acc[1] = ACC_Value.AXIS_Y * FROM_MG_TO_G;
      data_in.acc[2] = ACC_Value.AXIS_Z * FROM_MG_TO_G;

      data_in.mag[0] = MAG_Value.AXIS_X * FROM_MGAUSS_TO_UT50;
      data_in.mag[1] = MAG_Value.AXIS_Y * FROM_MGAUSS_TO_UT50;
      data_in.mag[2] = MAG_Value.AXIS_Z * FROM_MGAUSS_TO_UT50;

      /* Run Sensor Fusion algorithm */
      MotionFX_manager_run(pdata_in, pdata_out, MOTIONFX_ENGINE_DELTATIME);
      
//      floatToInt( data_out.quaternion_6X[0], &quat_0, 6 );
//      floatToInt( data_out.quaternion_6X[1], &quat_1, 6 );
//      floatToInt( data_out.quaternion_6X[2], &quat_2, 6 );
//      floatToInt( data_out.quaternion_6X[3], &quat_3, 6 );
//      sprintf( dataOut, "  %d.%06d:%d.%06d:%d.%06d:%d.%06d", (int)quat_0.out_int, (int)quat_0.out_dec,(int)quat_1.out_int, (int)quat_1.out_dec, (int)quat_2.out_int, (int)quat_2.out_dec, (int)quat_3.out_int, (int)quat_3.out_dec );
//      HAL_UART_Transmit( &huart2, ( uint8_t *)dataOut, strlen( dataOut ), 5000 );
//      
//      floatToInt( data_out.rotation_6X[0], &rot_0, 6 );
//      floatToInt( data_out.rotation_6X[1], &rot_1, 6 );
//      floatToInt( data_out.rotation_6X[2], &rot_2, 6 );
//      sprintf( dataOut, "  %d.%06d:%d.%06d:%d.%06d\r", (int)rot_0.out_int, (int)rot_0.out_dec, (int)rot_1.out_int, (int)rot_1.out_dec,(int)rot_2.out_int, (int)rot_2.out_dec );
//      HAL_UART_Transmit( &huart2, ( uint8_t *)dataOut, strlen( dataOut ), 5000 );
      
      floatToInt( data_out.quaternion_9X[0], &quat_0, 6 );
      floatToInt( data_out.quaternion_9X[1], &quat_1, 6 );
      floatToInt( data_out.quaternion_9X[2], &quat_2, 6 );
      floatToInt( data_out.quaternion_9X[3], &quat_3, 6 );
      sprintf( dataOut, "  %d.%06d:%d.%06d:%d.%06d:%d.%06d", (int)quat_0.out_int, (int)quat_0.out_dec,(int)quat_1.out_int, (int)quat_1.out_dec, (int)quat_2.out_int, (int)quat_2.out_dec, (int)quat_3.out_int, (int)quat_3.out_dec );
      HAL_UART_Transmit( &huart2, ( uint8_t *)dataOut, strlen( dataOut ), 5000 );
      
      floatToInt( data_out.rotation_9X[0], &rot_0, 6 );
      floatToInt( data_out.rotation_9X[1], &rot_1, 6 );
      floatToInt( data_out.rotation_9X[2], &rot_2, 6 );
      sprintf( dataOut, "  %d.%06d:%d.%06d:%d.%06d\r", (int)rot_0.out_int, (int)rot_0.out_dec, (int)rot_1.out_int, (int)rot_1.out_dec,(int)rot_2.out_int, (int)rot_2.out_dec );
      HAL_UART_Transmit( &huart2, ( uint8_t *)dataOut, strlen( dataOut ), 5000 );
      
    }
  }
}

/**
 * @brief  Splits a float into two integer values.
 * @param  in the float value as input
 * @param  out_value the pointer to the output integer structure
 * @param  dec_prec the decimal precision to be used
 * @retval None
 */
static void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec)
{
  if(in >= 0.0f)
  {
    out_value->sign = 0;
  }else
  {
    out_value->sign = 1;
    in = -in;
  }
  
  out_value->out_int = (int32_t)in;
  in = in - (float)(out_value->out_int);
  out_value->out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}

/**
  * @brief  Period elapsed callback
  * @param  htim pointer to a TIM_HandleTypeDef structure that contains
  *              the configuration information for TIM module.
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    sensor_read_request = 1;
  }
}
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
