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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdarg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
								//===== USER TYPES =====//
typedef enum {
    MOTOR_STATE_IDLE = 0,
    MOTOR_STATE_STARTUP,
    MOTOR_STATE_SENSORLESS,
	MOTOR_STATE_UNDEFINED
} MotorState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
								//===== USER DEFINES =====//
// ADC
#define adc_filter_len 		10
#define ADC_FULL_SCALE     	4095.0f
#define VDDA_VOLTS         	3.3f         // or measure via VREFINT for precision
#define BUS_DIV_RATIO       (18.0f / (169.0f + 18.0f))   // ≈ 0.01052
#define BEMF_DIV_RATIO	   	(2.2f / (10.0f + 2.2f))

// motor control
#define dmaPulse  		1
#define dmaPulseReload 	1023u
#define BLANKING_US 	60u
#define ZC_VALID_US 	5u
#define ZC_HYST 		0.05f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
								//===== USER GLOBAL VARS =====//
volatile MotorState_t motor_state = MOTOR_STATE_IDLE;
volatile MotorState_t motor_state_prev = MOTOR_STATE_UNDEFINED;
uint32_t poten_value = 0;
uint32_t poten_raw = 0;
uint8_t do_filter = 0;
uint16_t motorSpeed = 0;
uint16_t motorSpeedCurrent = 0;

uint32_t bus_raw = 0;
float bus_voltage = 0;

uint16_t bemf_a_raw = 0;
uint16_t bemf_b_raw = 0;
uint16_t bemf_c_raw = 0;

float bemf_a_voltage = 0;
float bemf_b_voltage = 0;
float bemf_c_voltage = 0;

uint16_t adc1_buffer[2] = {0};
uint16_t adc2_buffer[3] = {0};

// for info check:
//https://github.com/guser210/ESC/blob/master/Code/ESC_V1.1/cpp/src/main.cpp

uint32_t blanking_start = 0;
uint8_t blanking_active = 0;

volatile uint16_t dmaBuffer[dmaPulse] = {0};
volatile uint16_t resolution = dmaPulse * dmaPulseReload;
volatile uint8_t powerStep = 0;
/*
 * step		0	1	2	3	4	5
 * H		A 	B -	B	C	C	A
			  /   	  \
 * O		B 	A 	C 	B 	A	C
						  \
 * L		C	C 	A 	A 	B -	B
 * */

// define off phase rising array
// B will rise, A will fall, C will rise ...
const uint8_t risig[6] = {1,0,1,0,1,0};
// odLow port
GPIO_TypeDef *odLowPort[6] = {
		OD_C_GPIO_Port,
		OD_C_GPIO_Port,
		OD_A_GPIO_Port,
		OD_A_GPIO_Port,
		OD_B_GPIO_Port,
		OD_B_GPIO_Port
};
// odLow pin
const uint16_t odLowPin[6] = {
		OD_C_Pin,
		OD_C_Pin,
		OD_A_Pin,
		OD_A_Pin,
		OD_B_Pin,
		OD_B_Pin
};

// odHigh port
GPIO_TypeDef *odHighPort[6] = {
		OD_A_GPIO_Port,
		OD_B_GPIO_Port,
		OD_B_GPIO_Port,
		OD_C_GPIO_Port,
		OD_C_GPIO_Port,
		OD_A_GPIO_Port
};
// odHigh pin
const uint16_t odHighPin[6] = {
		OD_A_Pin,
		OD_B_Pin,
		OD_B_Pin,
		OD_C_Pin,
		OD_C_Pin,
		OD_A_Pin
};

// odOff port
GPIO_TypeDef *odOffPort[6] = {
		OD_B_GPIO_Port,
		OD_A_GPIO_Port,
		OD_C_GPIO_Port,
		OD_B_GPIO_Port,
		OD_A_GPIO_Port,
		OD_C_GPIO_Port
};
// odOff pin
const uint16_t odOffPin[6] = {
		OD_B_Pin,
		OD_A_Pin,
		OD_C_Pin,
		OD_B_Pin,
		OD_A_Pin,
		OD_C_Pin
};

// G4 reference manual : TIMx capture/compare enable register (TIMx_CCER)(x = 1, 8, 20)
// A CC1E = 0, B CC2E = 4, C CC3E = 8
// BACBAC
const uint32_t ccOff[6] = {
		TIM_CCER_CC2E,
		TIM_CCER_CC1E,
		TIM_CCER_CC3E,
		TIM_CCER_CC2E,
		TIM_CCER_CC1E,
		TIM_CCER_CC3E
};

// G4 reference manual : TIMx DMA/interrupt enable register (TIMx_DIER)(x = 1, 8, 20)
// A CC1DE = 9, B CC2DE = 10, C CC3DE = 11
// BACBAC
const uint32_t diOff[6] = {
		TIM_DIER_CC2DE,
		TIM_DIER_CC1DE,
		TIM_DIER_CC3DE,
		TIM_DIER_CC2DE,
		TIM_DIER_CC1DE,
		TIM_DIER_CC3DE
};

// G4 reference manual : TIMx capture/compare enable register (TIMx_CCER)(x = 1, 8, 20)
// A CC1E = 0, B CC2E = 4, C CC3E = 8
// BACBAC
const uint32_t ccHigh[6] = {
		TIM_CCER_CC1E,
		TIM_CCER_CC2E,
		TIM_CCER_CC2E,
		TIM_CCER_CC3E,
		TIM_CCER_CC3E,
		TIM_CCER_CC1E
};

// G4 reference manual : TIMx DMA/interrupt enable register (TIMx_DIER)(x = 1, 8, 20)
// A CC1DE = 9, B CC2DE = 10, C CC3DE = 11
// BACBAC
const uint32_t diHigh[6] = {
		TIM_DIER_CC1DE,
		TIM_DIER_CC2DE,
		TIM_DIER_CC2DE,
		TIM_DIER_CC3DE,
		TIM_DIER_CC3DE,
		TIM_DIER_CC1DE
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
								//===== USER FUNCTIONS =====//

int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}

void uart_printf(const char *fmt, ...)
{
    char buffer[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if ( hadc == &hadc1 )
	{
		// CH11: POTENTIOMETER
		// CH14: BEMFC
		poten_raw = adc1_buffer[0];
		bemf_c_raw = adc1_buffer[1];
		bemf_c_voltage =(bemf_c_raw/ADC_FULL_SCALE) * VDDA_VOLTS;
		bemf_c_voltage = bemf_c_voltage/BEMF_DIV_RATIO;
		// to avoid filtering in interrupt context.
		do_filter = 1;
	}
	else if( hadc == &hadc2 )
	{
		// CH1: BUS VOLTAGE
		// CH5: BEMFB
		// CH17: BEMFA
		bus_raw = adc2_buffer[0];
		bus_voltage = (bus_raw/ADC_FULL_SCALE) * VDDA_VOLTS;
		bus_voltage = bus_voltage/BUS_DIV_RATIO;

		bemf_b_raw = adc2_buffer[1];
		bemf_b_voltage =(bemf_b_raw/ADC_FULL_SCALE) * VDDA_VOLTS;
		bemf_b_voltage = bemf_b_voltage/BEMF_DIV_RATIO;

		bemf_a_raw = adc2_buffer[2];
		bemf_a_voltage =(bemf_a_raw/ADC_FULL_SCALE) * VDDA_VOLTS;
		bemf_a_voltage = bemf_a_voltage/BEMF_DIV_RATIO;
	}
}

// filter and normalize
void poten_filter_normalize()
{
  static uint32_t buff[adc_filter_len]= {0};
  static uint8_t i = 0;
  static uint32_t sum = 0;

	// filter
	sum -=buff[i];
	buff[i] = poten_raw;
	sum += buff[i];
	i = (i + 1) % adc_filter_len;
	uint32_t avg = sum/adc_filter_len;

	// normalize
	poten_value = avg * 1023 / 4095;
}

void setDutyCycle(uint16_t dc)
{
	if (dc > resolution) dc = resolution;

	dmaBuffer[0] = dc;
	TIM1->CCR1 = dc;
}


void commutate()
{
	__disable_irq();

	// got to next step
	powerStep = (powerStep + 1)%6;

	if (risig[powerStep])
	{
		/*
		 * step		0	1	2	3	4	5
		 * H		A 	B  	B	C	C	A
		 * O		B 	A 	C 	B 	A	C
		 * L		C	C 	A 	A 	B  	B
		 *
		 * 			STEP 2  is good example for rising (C on the way up)
		 * 			- C was low - turn it off
		 * 			- A was Off - turn it low
		 * */

		// enable low phase, that was off the previous step.
		odLowPort[powerStep]->BSRR = odLowPin[powerStep];

		// disable off phase, that was low the previous step.
		odOffPort[powerStep]->BRR  = odOffPin[powerStep];
	}
	else
	{
		/*
		 * step		0	1	2	3	4	5
		 * H		A 	B  	B	C	C	A
		 * O		B 	A 	C 	B 	A	C
		 * L		C	C 	A 	A 	B  	B
		 *
		 * 			STEP 3 is good example for falling (B on the way down)
		 * 			- B was high - turn it off
		 * 			- C was off  - turn it high
		 * */

		// disable off phase, that was high before.
		TIM1->CCER &= ~(ccOff[powerStep]);
		TIM1->DIER &= ~(diOff[powerStep]);

		// enable high phase, that was off before.
		TIM1->CCER |= (ccHigh[powerStep]); // enable capture/compare
		TIM1->DIER |= (diHigh[powerStep]); // enable dma
	}

	// set up delay of 5 us
	blanking_start = TIM7->CNT;
	blanking_active = 1;

	// reset tim6 for pumb motor.
	TIM6->CNT = 0;

	__enable_irq();
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if( (htim->Instance == TIM17) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) )
	{
		if( motorSpeedCurrent > 0 )
		{
				commutate();
				// stop one-shot, this is coming a short while after detecting zc
				HAL_TIM_OC_Stop_IT(&htim17, TIM_CHANNEL_1);
		}
	}
}

uint16_t startup_period = 10000;
uint16_t min_startup_period = 5000;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM6)
    {
		commutate(); // Step the motor

		// Gradually increase speed
		if (motorSpeedCurrent < 250)
		{
			motorSpeedCurrent += 5;
			setDutyCycle(motorSpeedCurrent);
		}

		// Decrease the period to speed up commutation
		if (startup_period > min_startup_period)
		{
			startup_period -= 5;
			__HAL_TIM_SET_AUTORELOAD(&htim6, startup_period);
		}
    }
}


uint32_t last_button_press = 0;
const uint32_t debounce_delay = 1000; // milliseconds
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    if (GPIO_Pin == BTN_Pin)
    {
        uint32_t now = HAL_GetTick();
        if ((now - last_button_press) > debounce_delay)
        {
            last_button_press = now;

            if (motor_state == MOTOR_STATE_IDLE)
                motor_state = MOTOR_STATE_STARTUP;
            else if (motor_state == MOTOR_STATE_STARTUP)
                motor_state = MOTOR_STATE_SENSORLESS;
            else
            	motor_state = MOTOR_STATE_IDLE;
        }
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  MX_TIM17_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  /*
   * TIM1 for pwm
   * TIM17 us timer, waiting after zc detection to commutate.
   * TIM7 170MHz, busy waits.
   * TIM6 pumb up motor.
   * */

  // start adc for potentiometer
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buffer, 2);
  // start adc for bus voltage
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buffer, 3);

  if( HAL_TIM_Base_Start(&htim7) != HAL_OK)
	  Error_Handler();
  if( HAL_TIM_Base_Start(&htim1) != HAL_OK)
	  Error_Handler();

  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;


  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_3);

  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*)dmaBuffer, dmaPulse);
  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t*)dmaBuffer, dmaPulse);
  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t*)dmaBuffer, dmaPulse);

  // set up tim X to pumb up the commutation. 10 ms timer.
  //HAL_TIM_Base_Start_IT(&htim6);


  // tim17 us timer to take time between zc.
  __HAL_TIM_SET_COUNTER(&htim17, 0);
  HAL_TIM_Base_Start(&htim17);  // start free-running timer

  MX_USART2_UART_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  float bemf = 0;
  float bemf_prev = 0;
  float bemf_filtered = 0;
  float half_bus = bus_voltage/2.0f;
  float half_bus_l = 0;
  float half_bus_h = 0;
  uint8_t zc_detected = 0;
  uint32_t zc_time_prev = 0;   // previous zero-crossing time
  uint32_t zc_period = 0;      // time between zero crossings
  uint8_t zc_pending = 0;
  uint32_t zc_timer_start = 0;
  while (1)
  {
	  switch (motor_state)
	  {
		case MOTOR_STATE_IDLE:
		{
			if (motor_state_prev != MOTOR_STATE_IDLE)
			{
				// stop
			    motorSpeedCurrent = 0;
			    setDutyCycle(motorSpeedCurrent);
			    HAL_TIM_Base_Stop_IT(&htim6);

				motor_state_prev = motor_state;
			}
			break;
		}
		case MOTOR_STATE_STARTUP:
		{
			if (motor_state_prev != MOTOR_STATE_STARTUP)
			{
				startup_period = 10000;
				motorSpeedCurrent = 100;
				setDutyCycle(motorSpeedCurrent);
			    __HAL_TIM_SET_AUTORELOAD(&htim6, startup_period);
				HAL_TIM_Base_Start_IT(&htim6);

				motor_state_prev = motor_state;
			}

			break;
		}
		case MOTOR_STATE_SENSORLESS:
		{
			if (motor_state_prev != MOTOR_STATE_SENSORLESS)
			{
				HAL_TIM_Base_Stop_IT(&htim6);
				motor_state_prev = motor_state;
			}
			if( do_filter )
				  {
					  poten_filter_normalize();
					  do_filter = 0;
					  motorSpeed = poten_value;

					  if (motorSpeed != motorSpeedCurrent)
					  {
						  if ( (motorSpeed - motorSpeedCurrent) > 20)
						  {
							  motorSpeedCurrent += 20;
						  }
						  else
						  {
							  motorSpeedCurrent = motorSpeed;
						  }
						  setDutyCycle(motorSpeedCurrent);
					  }
				  }
					char phase = 0;
				  switch(powerStep)
				  {
					  case 0:
						  bemf = bemf_b_voltage;
						  phase = 'b';
						  break;
					  case 1:
						  bemf = bemf_a_voltage;
						  phase = 'a';
						  break;
					  case 2:
						  bemf = bemf_c_voltage;
						  phase = 'c';
						  break;
					  case 3:
						  bemf = bemf_b_voltage;
						  phase = 'b';
						  break;
					  case 4:
						  bemf = bemf_a_voltage;
						  phase = 'a';
						  break;
					  case 5:
						  bemf = bemf_c_voltage;
						  phase = 'c';
						  break;
				  }

				  bemf_filtered = bemf;//0.7f * bemf_filtered + 0.3f * bemf;
				  half_bus = bus_voltage/2.0f;
				  half_bus_l = half_bus - ZC_HYST*bus_voltage;
				  half_bus_h = half_bus + ZC_HYST*bus_voltage;

				  uart_printf("%c - %u - %.2f - %.2f \r\n", phase, blanking_active, bemf_filtered,  half_bus );

				  // chill down for a bit after commutating.
				  if (blanking_active) {
				      uint32_t now = TIM7->CNT;
				      uint32_t elapsed = (now >= blanking_start) ?
				                         (now - blanking_start) :
				                         (0xFFFF - blanking_start + now + 1);
				      if (elapsed >= BLANKING_US * 170u)
				          blanking_active = 0;
				  }
				  if (blanking_active)
					  continue;

				  if (!zc_detected && !zc_pending)
				  {
				      if ((bemf_prev < half_bus_l && bemf_filtered >= half_bus_h) ||
				          (bemf_prev > half_bus_h && bemf_filtered <= half_bus_l))
				      {
				          // possible ZC, start validation window
				          zc_pending = 1;
				          zc_timer_start = TIM7->CNT;
				      }
				  }

				  if (zc_pending)
				  {
				      uint32_t elapsed = (TIM7->CNT >= zc_timer_start)
				                         ? (TIM7->CNT - zc_timer_start)
				                         : (0xFFFF - zc_timer_start + TIM7->CNT + 1);

				      if (elapsed >= ZC_VALID_US*170u)
				      {
				          // validation time expired, confirm stable
				          zc_detected = 1;
				          zc_pending = 0;
				      }
				      else
				      {
				          // if crossing reverted, cancel
				          if ((bemf_prev < half_bus_l && bemf_filtered < half_bus_l) ||
				              (bemf_prev > half_bus_h && bemf_filtered > half_bus_h))
				          {
				              zc_pending = 0;
				          }
				      }
				  }
				  bemf_prev = bemf_filtered;

				  if(zc_detected)
				  {
				      uint32_t zc_time = __HAL_TIM_GET_COUNTER(&htim17); // current time in µs

				      // compute time between last zero crossing and current one
				      if(zc_time_prev != 0)
				      {
				    	  // Compute period with overflow handling
				    	  zc_period = (zc_time >= zc_time_prev) ? (zc_time - zc_time_prev)
				    	                                          : (0xFFFF - zc_time_prev + zc_time + 1);
				      }
				      zc_time_prev = zc_time; // update for next ZC

				      // start commutation slightly after zero crossing
				      // for simplicity, we use half of the ZC period
				      if(zc_period > 0)
				      {
				          uint32_t pulse = zc_time + (zc_period/4);
				          __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, pulse & 0xFFFF);
				          HAL_TIM_OC_Start_IT(&htim17, TIM_CHANNEL_1); // enable OC interrupt
				          zc_detected = 0;
				      }
				  }
			break;
		}
		default:
			break;
	  }	// end of motor state switch.


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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
