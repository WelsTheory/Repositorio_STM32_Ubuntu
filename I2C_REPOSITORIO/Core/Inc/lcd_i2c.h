/*
 * lcd_i2c.h
 *
 *  Created on: 24 mar. 2021
 *      Author: Wels
 */

#ifndef INC_LCD_I2C_H_
#define INC_LCD_I2C_H_

#include "stm32f4xx_hal.h"

void lcd_init (void);   // initialize lcd

void lcd_send_cmd (char cmd);  // send command to the lcd

void lcd_send_data (char data);  // send data to the lcd

void lcd_send_string (char *str);  // send string to the lcd

void lcd_clear (void);

#endif /* INC_LCD_I2C_H_ */
