/*
 * ESPDataLogger.h
 *
 *  Created on: May 26, 2020
 *      Author: controllerstech
 */

#ifndef INC_ESPDATALOGGER_H_
#define INC_ESPDATALOGGER_H_

void ESP_Init (char *SSID, char *PASSWD);
void ESP_Send_Data (char *APIkey, int Field_num, uint16_t value);
void ESP_Send_Multi (char *APIkey, int numberoffileds, uint16_t value[]);

#endif /* INC_ESPDATALOGGER_H_ */
