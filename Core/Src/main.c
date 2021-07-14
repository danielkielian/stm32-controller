/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <string.h>
#include "iir_first_order.h"
#include "sensors.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum Adc_Input_Order {
	CURRENT = 0,
	BATTERY,
	THROTTLE
};

typedef struct {
	float throttle;		//0-1000[-]
	float battery_voltage;	//0-18[V]
	float current;			//0-20[A]
	//uint8_t battery_level;	//0-100[%]
	//uint32_t rpm;			//[rpm]
} FilterData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLING_FREQUENCY	1000	//[Hz]
#define NBR_OF_SAMPLES_PER_PERIOD	100	//[-]


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adcData[2][THROTTLE - CURRENT + 1];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  IirFilter_t throttle_filter, battery_filter, current_filter;

  IirFilter_Init(&throttle_filter, SAMPLING_FREQUENCY, 0.2);
  IirFilter_Init(&battery_filter, SAMPLING_FREQUENCY, 0.2);
  IirFilter_Init(&current_filter, SAMPLING_FREQUENCY, 0.05);
  SensorsInit(&htim4.Instance->CNT, NBR_OF_SAMPLES_PER_PERIOD*(1000/SAMPLING_FREQUENCY));
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

  FilterData_t x_data;

  uint32_t sysCounter;
  uint32_t mainCounter = 0;
  uint8_t _send_data = 0;		//Flag which decides whether send raport

  uint8_t _pair = 1;			//Flag that points appropriate buffer for ADC data

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  sysCounter = HAL_GetTick();

	  //Simple program timer
	  if(mainCounter < sysCounter)
	  {
		  mainCounter = sysCounter + 1000/SAMPLING_FREQUENCY;

		  //START MEASURMENT
		  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adcData[_pair], 3);

		  //CALCULATIONS
		  _pair++;
		  _pair %= 2;
		  x_data.current = CalculateCurrent(&adcData[_pair][CURRENT]);
		  x_data.battery_voltage = CalculateBatteryVoltage(&adcData[_pair][BATTERY]);
		  x_data.throttle = CalculateThrottle(&adcData[_pair][THROTTLE]);

		  //FILTERING
		  IirFilter(&current_filter, &x_data.current);
		  IirFilter(&battery_filter, &x_data.battery_voltage);
		  IirFilter(&throttle_filter, &x_data.throttle);

		  //MOTOR CONTROL
		  if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin))	//PROTECTION - BUTTON HAS TO BE PRESSED
		  {
			  htim11.Instance->CCR1 = 1000 + (uint32_t)throttle_filter.y_k;
		  } else {
			  htim11.Instance->CCR1 = 1000;
		  }

		  //SENDING DATA REPORTS
		  _send_data++;
		  if(_send_data >= NBR_OF_SAMPLES_PER_PERIOD)
		  {
			  uint32_t rpm = GetRPM();
			  float power = 4 * battery_filter.y_k * current_filter.y_k;
			  uint8_t battery_perecentage = GetBatteryPercentage(&battery_filter.y_k);
			  char buff[100];
			  sprintf(buff,"RD:\t%d\t%4.1f\t%4.1f\t%ld\t%5.1f\t%d\r\n",
					  (uint16_t)throttle_filter.y_k, battery_filter.y_k, current_filter.y_k,
					  rpm, power, battery_perecentage);
			  CDC_Transmit_FS(buff, strlen(buff));
			  HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
			  _send_data = 0;
		  }
	  }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
