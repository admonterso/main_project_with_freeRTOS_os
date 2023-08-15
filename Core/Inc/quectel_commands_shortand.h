/*
 * quectel_commands_shortand.h
 *
 *  Created on: Aug 8, 2023
 *      Author: admonterso
 */

#ifndef INC_QUECTEL_COMMANDS_SHORTAND_H_
#define INC_QUECTEL_COMMANDS_SHORTAND_H_

void MQTTPubToTopic(int length);
void send_data_to_MQTT(int length, uint8_t* data);
#endif /* INC_QUECTEL_COMMANDS_SHORTAND_H_ */
