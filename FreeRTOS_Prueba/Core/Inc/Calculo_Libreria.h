/*
 * Calculo_Libreria.h
 *
 *  Created on: Jul 4, 2021
 *      Author: wels
 */

#ifndef INC_CALCULO_LIBRERIA_H_
#define INC_CALCULO_LIBRERIA_H_

#include <math.h>

typedef struct{
	float n[94];
}DataX_T;

typedef struct
{
	float result;
	float integral_result;
	float result_vmp;
}Calculos_t;

#define Muestras	94
#define h_dato 		0.0159
#define coef  		h_dato/2
#define G6			6*MPU60X0_G
#define	G2 			2*MPU60X0_G

float Ax1,Ay1,Az1;
float accel_tot, pow_x, pow_y, pow_z;
float integral,vmp_dato;

//FUNCIONES CALCULAR
int32_t Calculo_Init(void);
float Func_Calculo_6G(void);
float Func_Calculo_2G(void);

#endif /* INC_CALCULO_LIBRERIA_H_ */
