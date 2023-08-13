/*
 * extra_functions_for_work.c
 *
 *  Created on: Aug 8, 2023
 *      Author: admonterso
 */
#include "extra_functions_for_work.h"
#include "main.h"
#include <stdlib.h>

char* convertNumberToCharArray(uint64_t number) {
    // Count the number of digits in the number
    uint64_t temp = number;
    int numDigits = 1;
    while (temp /= 10) {
        numDigits++;
    }

    // Allocate memory for the character array (+1 for null-terminator)
    char* buffer = (char*)malloc((numDigits + 1) * sizeof(char));
    if (buffer == NULL) {
        // Error in memory allocation
        return NULL;
    }

    // Convert each digit to its corresponding character representation
    int i = numDigits - 1;
    while (number != 0) {
        buffer[i--] = '0' + (number % 10);
        number /= 10;
    }

    buffer[numDigits] = '\0'; // Null-terminate the character array

    return buffer;
}

void jumpToAddress(uint32_t ADDRESSTOGO){
	  uint32_t addresstojump;

	  addresstojump = *((volatile uint32_t*)(ADDRESSTOGO + 4));
	  HAL_DeInit();
	  HAL_RCC_DeInit();
	  void (*GoToApp)(void);
	  GoToApp = (void (*) (void))addresstojump;
	  __disable_irq();
	  __set_MSP(*((volatile uint32_t*)ADDRESSTOGO));
	  GoToApp();
}
