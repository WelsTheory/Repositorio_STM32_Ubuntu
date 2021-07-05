/*
 * sapi.h
 *
 *  Created on: May 7, 2021
 *      Author: wels
 */

#ifndef INC_SAPI_H_
#define INC_SAPI_H_

/*
 * sapi.h
 *
 *  Created on: Nov 11, 2020
 *      Author: wels
 */

//LIBRERIAS
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

//Declaración de estructuras delay_t
typedef struct{
	tick_t startTime;
	tick_t duration;
	bool_t running;
}delay_t;

//Tiempos para modo de operación
#define TIEMPO_NORMAL     		    500    // ms
#define TIEMPO_LENTO			    2000   // ms
#define TIEMPO_RAPIDO			    100    // ms

//Tiempo para rebote pulsadores
#define TIEMPO_BOTON			    40    // ms

//Variables para retardos
delay_t MEF_Normal;
delay_t MEF_Lento;
delay_t MEF_Rapido;
delay_t MEF_Boton;

void Delays_NB(void);

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

// Función de inicialización
void boardInit(void);

#endif /* INC_SAPI_H_ */
