/*
 * sapi.h
 *
 *  Created on: Jul 4, 2021
 *      Author: wels
 */

#ifndef INC_SAPI_H_
#define INC_SAPI_H_

#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>

//Estados LOW y True
#ifndef LOW
#define LOW 	0
#endif

//Estados lógicos TRUE y FALSE
#ifndef TRUE
#define TRUE	1
#endif


#ifndef	FALSE
#define FALSE	0
#endif

//Variables bool_t y tick_t
typedef uint8_t bool_t;
typedef uint16_t tick_t;


void boardInit(void);



#endif /* INC_SAPI_H_ */
