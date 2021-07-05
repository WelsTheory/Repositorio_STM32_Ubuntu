/*
 * Integral.c
 *
 *  Created on: May 7, 2021
 *      Author: wels
 */
#include "Integral.h"
#include "main.h"
#include "stdint.h"


/* File inclusion */
#include "WL_numeric_integration.h"



/* Function definition */

float Integration_Trapezoid_Method(float y_val, uint8_t reset){

    static float f_x[2] = {0.0, 0.0};
    static float total_area = 0.0;
    float delta_area = 0.0;

    if(reset == 0){

        /* Actualizar los valores almacenados de y */
        f_x[1] = f_x[0];
        f_x[0] = y_val;

        /* Calcular el delta de area */
        delta_area = ((f_x[0] + f_x[1]) * DELTA_X) / 2.0;

        /* Actualizar la integral */
        total_area += delta_area;

    }else{

        f_x[1] = 0.0;
        f_x[0] = 0.0;
        total_area = 0.0;

    }

    return total_area;
}


float Integration_Midpoint_Method(float y_val, uint8_t reset){

}


float Integration_Simpson_Method(float y_val, uint8_t reset){

}

//uint32_t index_muestras = 0;
//uint32_t index;
//
//DataX_T X;
//MEFEstado_t estados;
//
//extern UART_HandleTypeDef huart2;
//
//char datos[20];
//
//float resultado;
//
//void Func_Inicial(void)
//{
//	for(uint32_t i = 0; i < Muestras_Total; i ++)
//	{
//		X.n[i] = 3;
//	}
//	estados = IMPRIMIR;
//}
//
//void Func_Trapezoidal(void)
//{
//
//	switch(estados)
//	{
//	case LECTURA_DATOS:
//		while(1);
//		break;
//	case IMPRIMIR:
//		for(uint32_t i = 0; i < Muestras_Total; i ++)
//		{
//			if(i == 0)
//			{
//				resultado = X.n[i];
//			}
//			else if(i == 9)
//			{
//				resultado = resultado + X.n[i];
//				resultado = coef*resultado;
//				sprintf(datos,"DATOS: %.2f\r\n", resultado);
//				HAL_UART_Transmit(&huart2, datos, 20, HAL_MAX_DELAY);
//				//printf("El resultado es %.2f \n\n", resultado);
//				estados = LECTURA_DATOS;
//			}
//			else
//			{
//				resultado = resultado + 2*X.n[i];
//			}
//		}
//		break;
//	}
//}

//int Func_Trapezoidal(void)
//{
//	switch()
//	if(index_muestras >= Primer_dato && index_muestras < Muestras_Total)
//	{
//		if(Func_Integral() >= 1.5)
//		{
//			Cal.result_vmp = 0.0;
//			Cal.result = 0.0;
//			Cal.integral_result = 0.0;
//			return -3;
//		}
//		else
//		{
//			index_muestras++;
//			return 1;
//		}
//	}
//	else{
//
//		for(index = 0; index < Muestras_Total - 1; index++)
//		{
//			X.n[index] = X.n[index+1];
//		}
//
//		index_muestras = Muestras_Total - 1;
//	}
//
//}
//
//float Func_Integral(void)
//{
//	uint32_t N_muestras;
//	for(N_muestras = 0; N_muestras < Muestras_Total; N_muestras++)
//	{
//		if(N_muestras == 0)
//		{
//			integral += X.n[0];
//		}
//		integral += 2*X.n[N_muestras];
//	}
//	integral = coef*integral;
//	return Cal.integral_result;
//}

