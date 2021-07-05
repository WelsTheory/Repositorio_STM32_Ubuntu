#include <stdio.h>
#include "MPU_Libreria.h"
#include "Calculo_Libreria.h"
#include "Integral.h"
#include "main.h"

DataX_T X;
Calculos_t Cal;
uint32_t N_muestras = 0;

char datos[10];

extern UART_HandleTypeDef huart2;

int32_t Calculo_Init(void)
{
	mpu60X0Read();
	Ax1 = mpu60X0GetAccelX_mss();
	Ay1 = mpu60X0GetAccelY_mss();
	Az1 = mpu60X0GetAccelZ_mss();
	//sprintf(datos,"%.2f\n",Ax1);
	//HAL_UART_Transmit(&huart2, datos, 10, 100);
	//HAL_UART_Transmit(&huart2, "Hola\n", 6, 100);
	if(Func_Calculo_6G() >= G6)
	{
		Cal.result = 0.0;
		Cal.result_vmp = 0.0;
		return -1;
	}
	if(Func_Calculo_2G() > G2)
	{
		Cal.result_vmp = 0.0;
		return -2;
	}
	X.n[N_muestras] = Func_Calculo_6G();
	N_muestras++;
	if(Integration_Simpson_Method(X.n[N_muestras],0) > 1.5)
	{
		Integration_Simpson_Method(X.n[N_muestras],1);
		N_muestras = 0;
		return -3;
	}
	if(N_muestras >= 96)
	{
		N_muestras = 0;
		Integration_Simpson_Method(X.n[N_muestras],1);
		return 1;
	}
	return 1;
}

float Func_Calculo_6G(void)
{
	pow_x = pow(Ax1,2);
	pow_y = pow(Ay1,2);
	pow_z = pow(Az1,2);
	accel_tot = pow_x + pow_y + pow_z;
	Cal.result = sqrt(accel_tot);
	return Cal.result;
}

float Func_Calculo_2G(void)
{
	vmp_dato = pow_x + pow_z;
	Cal.result_vmp = sqrt(vmp_dato);
	return Cal.result_vmp;
}



