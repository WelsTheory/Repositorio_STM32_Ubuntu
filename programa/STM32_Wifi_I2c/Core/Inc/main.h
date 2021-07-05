/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
//Variables bool_t y tick_t
typedef uint8_t bool_t;
typedef uint16_t tick_t;

//Declaración de estructuras delay_t
typedef struct{
	tick_t startTime;
	tick_t duration;
	bool_t running;
}delay_t;

UART_HandleTypeDef huart2;

//Delay No Bloqueante
/*
 * -> void delay_ms(delay_t * delay,tick_t duration) : Función para crear el delay y la duración
 * -> bool_t delayRead(delay_t * delay) : Lee si el retardo se efectuó
 * */
void delay_ms(delay_t * delay,tick_t duration);
bool_t delayRead(delay_t * delay);

//Delay Bloqueante
/*
 * -> void tickRead(void) : Retorna el valor tickCounter del Callback Timer 11
 * -> void tickWrite(uint16_t ticks) : Escribe el valor tick en tickCounter
 * -> void delay(uint16_t ticks) : Función retardo
 * */
uint16_t tickRead(void);
void tickWrite(uint16_t ticks);
void delay(uint16_t ticks);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B0_Pin GPIO_PIN_13
#define B0_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
