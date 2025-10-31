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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define adc_filter_len 10

uint32_t poten_value = 0;
uint32_t poten_raw = 0;
uint8_t do_filter = 0;

uint32_t bus_raw = 0;
float bus_voltage = 0;


#define ADC_FULL_SCALE     4095.0f
#define VDDA_VOLTS         3.3f         // or measure via VREFINT for precision
#define DIV_RATIO          (18.0f / (169.0f + 18.0f))   // ≈ 0.01052


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if ( hadc == &hadc1 )
	{
		poten_raw = HAL_ADC_GetValue(&hadc1);
		// to avoid filtering in interrupt context.
		do_filter = 1;
	}
	else if( hadc == &hadc2 )
	{
		bus_raw = HAL_ADC_GetValue(&hadc2);
		bus_voltage = (bus_raw/ADC_FULL_SCALE) * VDDA_VOLTS;
		bus_voltage = bus_voltage/DIV_RATIO;
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

// for info check:
//https://github.com/guser210/ESC/blob/master/Code/ESC_V1.1/cpp/src/main.cpp


#define dmaPulse  1
const uint16_t dmaPulseReload = 1023;
volatile uint16_t dmaBuffer[dmaPulse] = {0};
volatile uint16_t resolution = dmaPulse * dmaPulseReload;

uint16_t motorSpeed = 0;
uint16_t motorSpeedCurrent = 0;

void setDutyCycle(uint16_t dc)
{
	if (dc > resolution) dc = resolution;

	dmaBuffer[0] = dc;
	TIM1->CCR1 = dc;
}

volatile uint32_t resetImrFlags = 0;
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

uint8_t do_commutate = 0;
void commutate()
{

	__disable_irq();
	// clear wake up interrupts

	// disable zero-crossing interrupts
	// TODO: disable zc interrupts

	// clear zc pending interrupts
	// TODO: clear pending interrupts

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

	// delay of 2 us
	TIM7->CNT = 0;
	// HCLK is 170 MHz, meaning 170 ticks a us.
	while(TIM7->CNT < 340);

	// enable wake up interrupts

	// reset tim17 for pumb motor.
	TIM17->CNT = 0;

	__enable_irq();
}

// TODO: zero-crossing detection.

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if( (htim == &htim17) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) )
	{
		if( motorSpeedCurrent > 0 )
		{
			if (do_commutate == 0)
			{
				do_commutate = 1;
			}
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
  /* USER CODE BEGIN 2 */
  // start adc for potentiometer
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_IT(&hadc1);
  // start adc for bus voltage
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADC_Start_IT(&hadc2);


  resetImrFlags = EXTI->IMR1;
  //TODO: reset the zc pins so it is turned off.

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

  // set up tim 17 to pumb up the commutation.
  // klc is 170MHz , 170000 ticks a ms, prescaler set to 170000/3
  __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 3*5);
  HAL_TIM_OC_Start_IT(&htim17, TIM_CHANNEL_1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
	  if ( do_commutate )
	  {
		  commutate();
		  do_commutate = 0;
	  }
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
