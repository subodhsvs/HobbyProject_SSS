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

/* USER CODE BEGIN Includes */
#include "MotionFX_Manager.h"
#include "pid_controller.h"
#include "mapping_function.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define SAMPLE_FREQ                     ((uint8_t)100)  /* [Hz] */
#define SAMPLE_PERIOD                   ((uint8_t)10)   /* [ms] */
#define MOTIONFX_ENGINE_DELTATIME       0.01f

#define FROM_MG_TO_G                    0.001f
#define FROM_G_TO_MG                    1000.0f
#define FROM_MDPS_TO_DPS                0.001f
#define FROM_DPS_TO_MDPS                1000.0f
#define FROM_MGAUSS_TO_UT50             (0.1f/50.0f)
#define FROM_UT50_TO_MGAUSS             500.0f
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_CRC_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void Init_Sensors(void);
static void MX_TIM_FX_Init(void);
static void Accelero_Sensor_Handler(void);
static void Gyro_Sensor_Handler(void);
static void Magneto_Sensor_Handler(void);
static void Pressure_Sensor_Handler(void);
static void FX_Data_Handler(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* Public variables ----------------------------------------------------------*/
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
void *PRESSURE_handle    = NULL;

/* Global variables for sensor data */

/* Current readings for yaw, pitch and roll */
float Y_current;
float P_current;
float R_current;
float sensor_data_heading;

/* TODO : change setpoints here */
float Y_setpoint = 0.0F;
float P_setpoint = 0.0F;
float R_setpoint = 0.0F;

PID_VARS_INIT(Y_var);
PID_VARS_INIT(P_var);
PID_VARS_INIT(R_var);
MOTOR_VARS_INIT(motor_vars);


/* Private macro -------------------------------------------------------------*/
   
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef FX_TimHandle;
volatile static uint8_t sensor_read_request     = 0;
volatile static uint8_t magcal_request          = 0;

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  
  char lib_version[35];
  int lib_version_len;
  
  /* Customizing PID vars for Y, P and R (yaw, pitch & roll)*/
  
  Y_var.Kp = 0.0;
  Y_var.Ki = 0.0;
  Y_var.Kd = 0.0;
  Y_var.output_max = 0.0;
  Y_var.output_min = 0.0;

  P_var.Kp = 0.0;
  P_var.Ki = 0.0;
  P_var.Kd = 0.0;
  P_var.output_max = 0.0;
  P_var.output_min = 0.0;
  
  R_var.Kp = 0.0;
  R_var.Ki = 0.0;
  R_var.Kd = 0.0;
  R_var.output_max = 0.0;
  R_var.output_min = 0.0;

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
  MX_TIM1_Init();
  MX_TIM2_Init();
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

  /* Initialize Communication Peripheral for data log */
//  USARTConfig();

  /* RTC Initialization */
//  RTC_Config();
//  RTC_TimeStampConfig();

  /* Timer for FX algorithm synchronization Initialization */
  MX_TIM_FX_Init();
  
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
  }
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
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
    }

    if (sensor_read_request)
    {
      sensor_read_request = 0;

      /* Acquire data from enabled sensors and fill Msg stream */
      Accelero_Sensor_Handler();
      Gyro_Sensor_Handler();
      Magneto_Sensor_Handler();
      Pressure_Sensor_Handler();

      /* Sensor Fusion specific part */
      FX_Data_Handler();
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

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 32;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 25000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 32;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 25000;
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

  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 20000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
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

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

static void Init_Sensors(void)
{
  /* Initialize and Configure all sensors */
  BSP_ACCELERO_Init(ACCELERO_SENSORS_AUTO, &ACCELERO_handle);
  BSP_GYRO_Init(GYRO_SENSORS_AUTO, &GYRO_handle);
  BSP_MAGNETO_Init(MAGNETO_SENSORS_AUTO, &MAGNETO_handle);
//  BSP_HUMIDITY_Init(HUMIDITY_SENSORS_AUTO, &HUMIDITY_handle);
//  BSP_TEMPERATURE_Init(TEMPERATURE_SENSORS_AUTO, &TEMPERATURE_handle);
  BSP_PRESSURE_Init(PRESSURE_SENSORS_AUTO, &PRESSURE_handle);
}

/**
  * @brief  TIM_FX init function.
  * @param  None
  * @retval None
  * @details This function intialize the Timer used to syncronize the FX algorithm
  */
static void MX_TIM_FX_Init(void)
{
#define PERIOD_100HZ  ((uint8_t)20)

#if (defined (USE_STM32F4XX_NUCLEO))    /* 84 MHZ CPU clock */
#define PRESCALER_100HZ  ((uint16_t)41999)

#elif (defined (USE_STM32L4XX_NUCLEO))  /* 80 MHZ CPU clock */
#define PRESCALER_100HZ  ((uint16_t)39999)

#else
#error Not supported platform
#endif

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  FX_TimHandle.Instance = TIM_FX;
  FX_TimHandle.Init.Prescaler = PRESCALER_100HZ;
  FX_TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  FX_TimHandle.Init.Period = PERIOD_100HZ;
  FX_TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&FX_TimHandle);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&FX_TimHandle, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&FX_TimHandle, &sMasterConfig);
}

/**
  * @brief  Handles the ACC axes data getting/sending
  * @param  Msg - ACC part of the stream
  * @retval None
  */
static void Accelero_Sensor_Handler()
{
  uint8_t status = 0;

  if (Sensors_Enabled & ACCELEROMETER_SENSOR)
  {
    if (BSP_ACCELERO_IsInitialized(ACCELERO_handle, &status) == COMPONENT_OK && status == 1)
    {
      BSP_ACCELERO_Get_Axes(ACCELERO_handle, &ACC_Value);
    }
  }
}

/**
  * @brief  Handles the GYR axes data getting/sending
  * @param  Msg - GYR part of the stream
  * @retval None
  */
static void Gyro_Sensor_Handler()
{
  uint8_t status = 0;

  if (Sensors_Enabled & GYROSCOPE_SENSOR)
  {
    if (BSP_GYRO_IsInitialized(GYRO_handle, &status) == COMPONENT_OK && status == 1)
    {
      BSP_GYRO_Get_Axes(GYRO_handle, &GYR_Value);
    }
  }
}


/**
  * @brief  Handles the MAG axes data getting/sending
  * @param  Msg - MAG part of the stream
  * @retval None
  */
static void Magneto_Sensor_Handler()
{
  uint8_t status = 0;
  MFX_MagCal_input_t mag_data_in;
  MFX_MagCal_output_t mag_data_out;

  if (Sensors_Enabled & MAGNETIC_SENSOR)
  {
    if (BSP_MAGNETO_IsInitialized(MAGNETO_handle, &status) == COMPONENT_OK && status == 1)
    {
      BSP_MAGNETO_Get_Axes(MAGNETO_handle, &MAG_Value);

      if (mag_cal_status == 0)
      {
        mag_data_in.mag[0] = MAG_Value.AXIS_X * FROM_MGAUSS_TO_UT50;
        mag_data_in.mag[1] = MAG_Value.AXIS_Y * FROM_MGAUSS_TO_UT50;
        mag_data_in.mag[2] = MAG_Value.AXIS_Z * FROM_MGAUSS_TO_UT50;
        mag_data_in.time_stamp = mag_time_stamp;
        mag_time_stamp += SAMPLE_PERIOD;

        MotionFX_manager_MagCal_run(&mag_data_in, &mag_data_out);

        if (mag_data_out.cal_quality == MFX_MAGCALGOOD)
        {
          mag_cal_status = 1;

          MAG_Offset.AXIS_X = (int32_t) (mag_data_out.hi_bias[0] * FROM_UT50_TO_MGAUSS);
          MAG_Offset.AXIS_Y = (int32_t) (mag_data_out.hi_bias[1] * FROM_UT50_TO_MGAUSS);
          MAG_Offset.AXIS_Z = (int32_t) (mag_data_out.hi_bias[2] * FROM_UT50_TO_MGAUSS);

          /* Disable magnetometer calibration */
          MotionFX_manager_MagCal_stop(SAMPLE_PERIOD);

          /* Switch on the LED */
          BSP_LED_On(LED2);
        }
      }

      MAG_Value.AXIS_X = (int32_t) (MAG_Value.AXIS_X - MAG_Offset.AXIS_X);
      MAG_Value.AXIS_Y = (int32_t) (MAG_Value.AXIS_Y - MAG_Offset.AXIS_Y);
      MAG_Value.AXIS_Z = (int32_t) (MAG_Value.AXIS_Z - MAG_Offset.AXIS_Z);

    }
  }
}


/**
  * @brief  Handles the PRESS sensor data getting/sending.
  * @param  Msg - PRESS part of the stream
  * @retval None
  */
static void Pressure_Sensor_Handler()
{
  uint8_t status = 0;
  float PRESSURE_Value;

  if (Sensors_Enabled & PRESSURE_SENSOR)
  {
    if (BSP_PRESSURE_IsInitialized(PRESSURE_handle, &status) == COMPONENT_OK && status == 1)
    {
      BSP_PRESSURE_Get_Press(PRESSURE_handle, &PRESSURE_Value);
    }
  }
}



/**
  * @brief  Sensor Fusion data handler
  * @param  Msg - Sensor Fusion data part of the stream
  * @retval None
  */
static void FX_Data_Handler()
{
  
  MFX_input_t data_in;
  MFX_input_t *pdata_in = &data_in;
  MFX_output_t data_out;
  MFX_output_t *pdata_out = &data_out;

  if ((Sensors_Enabled & ACCELEROMETER_SENSOR) && (Sensors_Enabled & GYROSCOPE_SENSOR) && (Sensors_Enabled & MAGNETIC_SENSOR))
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

    {
#if MFX_NUM_AXES==3
       Y_current = pdata_out->rotation_6X[0];
       P_current = pdata_out->rotation_6X[1];
       R_current = pdata_out->rotation_6X[2];
       sensor_data_heading = pdata_out->heading_6X;
#else
#error "MFX_NUM_AXES!=3"
#endif
    }

  }
}


/**
  * @brief  Period elapsed callback
  * @param  htim pointer to a TIM_HandleTypeDef structure that contains
  *              the configuration information for TIM module.
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM_FX)
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
