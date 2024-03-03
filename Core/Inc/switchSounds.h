/*
 * switchSounds.h
 *
 *  Created on: Apr 30, 2023
 *      Author: Dell
 */

#ifndef SRC_SWITCHSOUNDS_H_
#define SRC_SWITCHSOUNDS_H_

#include "stm32f1xx_hal.h"

void AppruveSound();
void RelaySwitch();
void ErrorSound();
void CardReadSound();

int takeStatus(uint8_t* data, int length);
void insert(uint8_t* main);
void takeData(uint8_t* data, int length, uint8_t* dataToDisplay);
uint8_t check_if_terminal_is_free(uint8_t* data, int length);
#endif /* SRC_SWITCHSOUNDS_H_ */
