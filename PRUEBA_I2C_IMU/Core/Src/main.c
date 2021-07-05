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
#include "stdio.h"
#include "MPU_Libreria.h"
#include "lcd_i2c.h"
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}

float Ax1;
float tmp;

/* USER CODE END 0 */
/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	char buf[4];
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
	MX_TIM11_Init();
	MX_I2C1_Init();
	lcd_init();

	lcd_send_cmd (0x80|0x00);
	lcd_send_string("HELLO WORLD");

	lcd_send_cmd (0x80|0x40);
	lcd_send_string("LCD 20x4 DEMO");
	/* USER CODE BEGIN 2 */
	//MPU6050_Init();
	int8_t status;
	status = MPU6050_Init (MPU60X0_ADDRESS_0);//mpu60X0Init( MPU6050_ADDR );
	if( status < 0 ){
		HAL_UART_Transmit(&huart2, "IMU MPU6050 no inicializado, chequee las conexiones:\r\n\r\n", 60, HAL_MAX_DELAY);
//		printf( "IMU MPU6050 no inicializado, chequee las conexiones:\r\n\r\n" );
//		printf( "MPU6050 ---- EDU-CIAA-NXP\r\n\r\n" );
//		printf( "    VCC ---- 3.3V\r\n" );
//		printf( "    GND ---- GND\r\n" );
//		printf( "    SCL ---- SCL\r\n" );
//		printf( "    SDA ---- SDA\r\n" );
//		printf( "    AD0 ---- GND\r\n\r\n" );
//		printf( "Se detiene el programa.\r\n" );
		while(1);
	}
	HAL_GPIO_TogglePin(GPIOA, A_BOARD_Pin);
	HAL_UART_Transmit(&huart2, "IMU MPU6050 inicializado correctamente.\r\n\r\n", 47, HAL_MAX_DELAY);
	//printf("IMU MPU6050 inicializado correctamente.\r\n\r\n" );
	//printf("Hola mundo");
	HAL_Delay(1000);
	/* USER CODE END 2 */
	//printf("Hello world");
	/* Infinite loop */
	char str[10];
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		mpu60X0Read();
//		tmp = mpu60X0GetTemperature_C();
//		Ax1 = mpu60X0GetAccelX_mss();
		tmp = 2.34;
		Ax1 = 5.45;
		sprintf(str,"Ac %.2f", Ax1);
		lcd_send_cmd (0x80|0x14);
		lcd_send_string(str);
		sprintf(str,"Tmp %.2f", tmp);
		lcd_send_cmd (0x80|0x54);
		lcd_send_string(str);
		HAL_GPIO_TogglePin(GPIOA, A_BOARD_Pin);
//		sprintf(str,"%d \n", Ax1);
//		HAL_UART_Transmit(&huart2, str, 5, HAL_MAX_DELAY);
//		sprintf(str,"%d \n", tmp);
//		HAL_UART_Transmit(&huart2, str, 5, HAL_MAX_DELAY);
//		HAL_UART_Transmit(&huart2, "Hola\r\n", 8, HAL_MAX_DELAY);

		//printf(str);
		//printf("Acelerometro:  %.2f    [rad/s]\r\n",Ax1);
		//HAL_UART_Transmit(&huart2, "IMU MPU6050 inicializado correctamente.\r\n\r\n", 47, HAL_MAX_DELAY);
		//printf("Hello world\n");
//		printf( "Giroscopo:      (%f, %f, %f)   [rad/s]\r\n",mpu60X0GetGyroX_rads(),mpu60X0GetGyroY_rads(),mpu60X0GetGyroZ_rads());
//
//		printf( "Acelerometro:   (%f, %f, %f)   [m/s2]\r\n",
//				mpu60X0GetAccelX_mss(),
//				mpu60X0GetAccelY_mss(),
//				mpu60X0GetAccelZ_mss()
//		);
//
//		printf( "Temperatura:    %f   [C]\r\n\r\n",
//				mpu60X0GetTemperature_C()
//		);
		HAL_Delay(1000);
		//MPU6050_Read_Accel();
		//		MPU6050_Read_Gyro();
		//		printf("Hola mundo");
		//		printf( "Acelerometro:   (%f, %f, %f)   [m/s2]\r\n",
		//		              Ax,
		//		              Ay,
		//		              Az
		//		            );
		//		HAL_Delay(1000);
		//		for(uint8_t i = 0;i<16;i++){
		//			//HAL_I2C_Master_Transmit(hi2c, DevAddress, pData, Size, Timeout)
		//			HAL_I2C_Master_Transmit(&hi2c1, 0X40, i, 1, 1000);
		//			HAL_GPIO_TogglePin(GPIOA, A_BOARD_Pin);
		//			HAL_Delay(250);
		//
		//		}
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 84;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
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
 * @brief TIM11 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM11_Init(void)
{

	/* USER CODE BEGIN TIM11_Init 0 */

	/* USER CODE END TIM11_Init 0 */

	/* USER CODE BEGIN TIM11_Init 1 */

	/* USER CODE END TIM11_Init 1 */
	htim11.Instance = TIM11;
	htim11.Init.Prescaler = 840-1;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 100-1;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM11_Init 2 */

	/* USER CODE END TIM11_Init 2 */

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

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(A_BOARD_GPIO_Port, A_BOARD_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B0_Pin */
	GPIO_InitStruct.Pin = B0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B0_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : A_BOARD_Pin */
	GPIO_InitStruct.Pin = A_BOARD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(A_BOARD_GPIO_Port, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
