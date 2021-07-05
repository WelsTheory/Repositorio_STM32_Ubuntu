/*
 * ESPDataLogger.c
 *
 *  Created on: May 26, 2020
 *      Author: controllerstech
 */

#include "UartRingbuffer.h"
#include "ESP8266_HAL.h"
#include "stdio.h"
#include "string.h"


void bufclr (char *buf)
{
	int len = strlen (buf);
	for (int i=0; i<len; i++) buf[i] = '\0';
}


void ESP_Init (char *SSID, char *PASSWD)
{
	char data[80];

	Ringbuf_init();

	Uart_sendstring("AT+RST\r\n");
	HAL_Delay(1000);

	Uart_flush();

	/********* AT **********/
	Uart_sendstring("AT\r\n");
	while(!(Wait_for("OK\r\n")));

	Uart_flush();


	/********* AT+CWMODE=1 **********/
	Uart_sendstring("AT+CWMODE=1\r\n");
	while (!(Wait_for("OK\r\n")));

	Uart_flush();


	/********* AT+CWJAP="SSID","PASSWD" **********/
	sprintf (data, "AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, PASSWD);
	Uart_sendstring(data);
	while (!(Wait_for("GOT IP\r\n")));

	Uart_flush();

	/********* AT+CIPMUX=0 **********/
	Uart_sendstring("AT+CIPMUX=0\r\n");
	while (!(Wait_for("OK\r\n")));

	Uart_flush();

	return 1;

}

void ESP_Send_Data (char *APIkey, int Field_num, uint16_t value)
{
	char local_buf[100] = {0};
	char local_buf2[30] = {0};

	Uart_sendstring("AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n");
	while (!(Wait_for("OK\r\n")));

	sprintf (local_buf, "GET /update?api_key=%s&field%d=%u\r\n", APIkey, Field_num, value);
	int len = strlen (local_buf);

	sprintf (local_buf2, "AT+CIPSEND=%d\r\n", len);
	Uart_sendstring(local_buf2);
	while (!(Wait_for(">")));

	Uart_sendstring (local_buf);
	while (!(Wait_for("SEND OK\r\n")));

	while (!(Wait_for("CLOSED")));

	bufclr(local_buf);
	bufclr(local_buf2);

	Ringbuf_init();

}

void ESP_Send_Multi (char *APIkey, int numberoffileds, uint16_t value[])
{
	char local_buf[500] = {0};
	char local_buf2[30] = {0};
	char field_buf[200] = {0};


	Uart_sendstring("AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n");
	while (!(Wait_for("OK\r\n")));

	sprintf (local_buf, "GET /update?api_key=%s", APIkey);
	for (int i=0; i<numberoffileds; i++)
	{
		sprintf(field_buf, "&field%d=%u",i+1, value[i]);
		strcat (local_buf, field_buf);
	}

	strcat(local_buf, "\r\n");
	int len = strlen (local_buf);

	sprintf (local_buf2, "AT+CIPSEND=%d\r\n", len);
	Uart_sendstring(local_buf2);
	while (!(Wait_for(">")));

	Uart_sendstring (local_buf);
	while (!(Wait_for("SEND OK\r\n")));

	while (!(Wait_for("CLOSED")));

	bufclr(local_buf);
	bufclr(local_buf2);

	Ringbuf_init();

}
