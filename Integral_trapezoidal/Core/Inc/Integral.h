/*
 * Integral.h
 *
 *  Created on: May 7, 2021
 *      Author: wels
 */

#ifndef WL_NUMERIC_INTEGRATION_H_
#define WL_NUMERIC_INTEGRATION_H_

/* File inclusion */
#include <stdint.h>

/* Macro definition */
#define DELTA_X     0.1


/* Function declaration */
float Integration_Trapezoid_Method(float y_val, uint8_t reset);
float Integration_Midpoint_Method(float y_val, uint8_t reset);
float Integration_Simpson_Method(float y_val, uint8_t reset);


#endif
//
//#ifndef INC_INTEGRAL_H_
//#define INC_INTEGRAL_H_
//
//// Estructura MEFEstado
//
//
//typedef enum{
//	LECTURA_DATOS,
//	IMPRIMIR,
//	IDLE_state
//}MEFEstado_t;
//
//typedef struct
//{
//	float result;
//	float integral_result;
//	float result_vmp
//}Calculos_t;
//
//#define Muestras	10
//#define valor_a		0
//#define valor_b		1.5
//#define h_dato 		(valor_a-valor_b)/Muestras
//#define coef  		h_dato/2
//#define G6			6*9.80
//#define	G2 			2*9.80
//
//#define Muestras_Total	10
//#define Primer_dato		0
//
//typedef struct{
//	float n[Muestras_Total];
//}DataX_T;
//
//
//void Func_Inicial(void);
//void Func_Trapezoidal(void);
//
//#endif /* INC_INTEGRAL_H_ */
