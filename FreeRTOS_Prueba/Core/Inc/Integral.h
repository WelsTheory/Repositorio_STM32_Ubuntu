/*
 * Integral.h
 *
 *  Created on: Jul 4, 2021
 *      Author: wels
 */

#ifndef INC_INTEGRAL_H_
#define INC_INTEGRAL_H_

/* File inclusion */
#include <stdint.h>

/* Macro definition */
#define DELTA_X     0.008


/* Function declaration */
float Integration_Trapezoid_Method(float y_val, uint8_t reset);
float Integration_Midpoint_Method(float y_val, uint8_t reset);
float Integration_Simpson_Method(float y_val, uint8_t reset);



#endif /* INC_INTEGRAL_H_ */
