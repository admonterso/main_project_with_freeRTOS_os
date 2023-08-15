/*
 * quectel_commands_shortand.c
 *
 *  Created on: Aug 8, 2023
 *      Author: admonterso
 */
#include "stdio.h"
#include "stm32f1xx_hal.h"
#include "quectel_commands_shortand.h"
#include "main.h"


#define backServer liftos08765546789
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

uint8_t MQTT_PUB[50];

void MQTTPubToTopic(int length){
	//uint8_t status = 0;
	sprintf((char*) MQTT_PUB, "AT+QMTPUBEX=0,0,0,0,\""STR(backServer)"\",%d\r\n", (length));


		HAL_UART_Transmit(&huart1, MQTT_PUB, sizeof MQTT_PUB / sizeof MQTT_PUB[0], 100);

		//HAL_UART_Receive(&huart1, RXBuffer, 10, 100);

		//status = checkCommand(RXBuffer, (uint8_t*)">");
		HAL_Delay(30);

}
void send_data_to_MQTT(int length, uint8_t* data){
	MQTTPubToTopic(length);
	HAL_Delay(50);

	HAL_UART_Transmit(&huart1, data, length, 100);
}


