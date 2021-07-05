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
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#include "MPU_Libreria.h"
#include "ESP8266_HAL.h"
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
//I2C_HandleTypeDef hi2c1;
//
//
//
//UART_HandleTypeDef huart1;
//UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */
//Delay No Bloqueante
/*
 * -> void delay_ms(delay_t * delay,tick_t duration) : Función para crear el delay y la duración
 * -> bool_t delayRead(delay_t * delay) : Lee si el retardo se efectuó
 * */

tick_t tickCounter = 0;
void delay_ms(delay_t * delay,tick_t duration){
	delay->duration = duration/1;
	delay->running = 0;
}

bool_t delayRead(delay_t * delay){
	bool_t timeArrived = 0;

	if( !delay->running ) {
		delay->startTime = tickRead();
		delay->running = 1;
	}
	else {
		if ((tick_t)(tickRead()-delay->startTime) >= delay->duration ) {
			timeArrived = 1;
			delay->running = 0;
		}
	}

	return timeArrived;
}

//Delay Bloqueante
/*
 * -> void tickRead(void) : Retorna el valor tickCounter del Callback Timer 11
 * -> void tickWrite(uint16_t ticks) : Escribe el valor tick en tickCounter
 * -> void delay(uint16_t ticks) : Función retardo
 * */
uint16_t tickRead(void){
	return tickCounter;
}

void tickWrite(uint16_t ticks){
	tickCounter = ticks;
}

void delay(uint16_t ticks){
	uint16_t tiempo = 0;
	tiempo = tickRead();
	while((tickRead() - tiempo) <= ticks);
	tickWrite(0);
}

delay_t MEF_Normal;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define TIEMPO			    15000  // ms
uint32_t count = 0;
uint16_t Buf[2];
uint16_t fall[1];
int _write(int file, char *ptr, int len)
{
	/* Implement your write code here, this is used by puts and printf for example */
	int i=0;
	for(i=0 ; i<len ; i++)
		ITM_SendChar((*ptr++));
	return len;
}

float Ax1,Ay1,Az1;
float Gx1,Gy1,Gz1;
float tmp;
char str1[10];
char str2[10];
char str3[10];
char str4[10];
char str5[10];
char str6[10];
char str7[10];
char str8[10];
double accel_tot, pow_x, pow_y, pow_z;
double result, vmp, result_vmp;
double G6 = 6*MPU60X0_G;
double G2 = 2*MPU60X0_G;
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
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	MX_TIM11_Init();

	/* USER CODE BEGIN 2 */
	HAL_UART_Transmit(&huart2, "HOLA :D\r\n", 8, HAL_MAX_DELAY);
	ESP_Init("wels","123456789");
	int8_t status;
	status = MPU6050_Init (MPU60X0_ADDRESS_0);//mpu60X0Init( MPU6050_ADDR );
	if( status < 0 ){
		HAL_UART_Transmit(&huart2, "IMU MPU6050 no inicializado, chequee las conexiones:\r\n", 60, HAL_MAX_DELAY);
		while(1);
	}
	HAL_GPIO_TogglePin(GPIOA, 5);
	HAL_UART_Transmit(&huart2, "IMU MPU6050 inicializado correctamente.\r\n", 47, HAL_MAX_DELAY);
	HAL_Delay(1000);
	//delay_ms(&MEF_Normal,15000);

	Buf[0] = count;
	Buf[1] = 1;
	ESP_Send_Multi("K77JWY4UUUZPGWSO",2,Buf);
	//ESP_Send_Multi("K77JWY4UUUZPGWSO",1,fall);
	//HAL_TIM_Base_Start_IT(&htim11);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		mpu60X0Read();
		Ax1 = mpu60X0GetAccelX_mss();
		Ay1 = mpu60X0GetAccelY_mss();
		Az1 = mpu60X0GetAccelZ_mss();
		pow_x = pow(Ax1,2);
		pow_y = pow(Ay1,2);
		pow_z = pow(Az1,2);
		accel_tot = pow_x + pow_y + pow_z;
		result = sqrt(accel_tot);
		vmp = pow_x + pow_z;
		result_vmp = sqrt(vmp);
		if(result_vmp >= G2)
		{
			Buf[1] = 1;
			//fall[0] = 1;
			count = 0;
			Buf[0] = count;
			ESP_Send_Multi("K77JWY4UUUZPGWSO",2,Buf);
			//sprintf(str1,"ACTU: %1f\n", result);
			//ESP_Send_Multi("K77JWY4UUUZPGWSO",2,fall);
			HAL_UART_Transmit(&huart2, "FALL\r\n", 10, HAL_MAX_DELAY);
			result = 0;
			fall[0] = 1;
			HAL_Delay(15000);
		}
//		if(result_vmp >= G2)
//		{
//			sprintf(str1,"ACTU: %1f\n", result_vmp);
//			HAL_UART_Transmit(&huart2, str1, 20, HAL_MAX_DELAY);
//			HAL_UART_Transmit(&huart2, "FALL 2G\r\n", 8, HAL_MAX_DELAY);
//			result_vmp = 0;
//			HAL_Delay(1000);
//		}
			//HAL_UART_Transmit(&huart2, str1, 12, HAL_MAX_DELAY);
			//HAL_GPIO_TogglePin(GPIOA, 5);
		//if(delayRead(&MEF_Normal)){
		else{
			//Si cumple tiempo de retardo Normal, envia el mensaje "Modo Normal"
			//fall[0] = 1;
			Buf[1] = 0;
			//sprintf(str1,"ACTU: %1f\n", result);
			//ESP_Send_Multi("K77JWY4UUUZPGWSO",2,fall);
			Buf[0] = count;
			count++;
			ESP_Send_Multi("K77JWY4UUUZPGWSO",2,Buf);
			HAL_UART_Transmit(&huart2, "NORMAL\r\n", 10, HAL_MAX_DELAY);
			HAL_Delay(100);
		}
		HAL_GPIO_TogglePin(GPIOA, LED_Pin);
		HAL_Delay(100);
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
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

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
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B0_Pin */
	GPIO_InitStruct.Pin = B0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B0_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim11){
		tickCounter++;
	}
}
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
