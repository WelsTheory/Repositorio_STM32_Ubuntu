
#include "sapi.h"
#include "Integral.h"

UART_HandleTypeDef huart2;

int main(void)
{
	boardInit();
	Func_Inicial();
	HAL_GPIO_TogglePin(GPIOA, 5);
	HAL_UART_Transmit(&huart2, "\nbienvenido\n", 10, HAL_MAX_DELAY);
	HAL_Delay(100);
	Func_Trapezoidal();
	while (1)
	{

	}

}

