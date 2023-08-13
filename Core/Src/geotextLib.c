/*
 * geotextLib.c
 *
 *  Created on: May 6, 2023
 *      Author: Dell
 */
#include "main.h"
#include "GEOtextLib.h"
#include "liquidcrystal_i2c.h"
#include "stdio.h"
#include <string.h>
#include <stdlib.h>

uint8_t nGeo[] = {
	  0x0E,
	  0x10,
	  0x10,
	  0x10,
	  0x1C,
	  0x12,
	  0x12,
	  0x0C
};

uint8_t sGeo[] = {
	  0x10,
	  0x10,
	  0x10,
	  0x10,
	  0x14,
	  0x12,
	  0x12,
	  0x0C
};
uint8_t xGeo[] = {
		  0x10,
		  0x10,
		  0x14,
		  0x18,
		  0x1C,
		  0x12,
		  0x12,
		  0x0C
};

uint8_t uGeo[] = {
		  0x0A,
		  0x15,
		  0x15,
		  0x01,
		  0x01,
		  0x01,
		  0x09,
		  0x06
};
uint8_t dGeo[] = {
		  0x0E,
		  0x11,
		  0x19,
		  0x16,
		  0x10,
		  0x0C,
		  0x02,
		  0x01
};

uint8_t TGeo[] = {
		  0x04,
		  0x0E,
		  0x15,
		  0x15,
		  0x11,
		  0x11,
		  0x11,
		  0x0E
};
uint8_t aGeo[] = {
		0x04,
		  0x02,
		  0x01,
		  0x01,
		  0x01,
		  0x09,
		  0x09,
		  0x06
};

uint8_t SGeo[] = {
		  0x0A,
		  0x15,
		  0x11,
		  0x01,
		  0x07,
		  0x09,
		  0x09,
		  0x06
};
uint8_t bGeo[] = {
		  0B10000,
		  0B11100,
		  0B10100,
		  0B00100,
		  0B01110,
		  0B10001,
		  0B10001,
		  0B01110
};
uint8_t iGeo[] = {
		  0x0E,
		  0x11,
		  0x11,
		  0x11,
		  0x11,
		  0x11,
		  0x11,
		  0x0A
};
uint8_t lGeo[] = {
		  0x0E,
		  0x11,
		  0x11,
		  0x11,
		  0x10,
		  0x0C,
		  0x02,
		  0x01
};
uint8_t cGeo[] = {
		  0x11,
		  0x11,
		  0x12,
		  0x14,
		  0x12,
		  0x11,
		  0x11,
		  0x0E
};
uint8_t oGeo[] = {
		  0x0A,
		  0x15,
		  0x15,
		  0x15,
		  0x11,
		  0x11,
		  0x11,
		  0x0A
};
uint8_t tGeo[] = {
		  0x0A,
		  0x15,
		  0x15,
		  0x15,
		  0x15,
		  0x15,
		  0x15,
		  0x09
};
uint8_t mGeo[] = {
		0x06,
		  0x09,
		  0x09,
		  0x01,
		  0x07,
		  0x09,
		  0x09,
		  0x06

};
uint8_t zGeo[] = {
		0x08,
		  0x14,
		  0x14,
		  0x0C,
		  0x06,
		  0x05,
		  0x05,
		  0x02

};
uint8_t vGeo[] = {
		0x06,
		  0x09,
		  0x01,
		  0x02,
		  0x01,
		  0x01,
		  0x09,
		  0x06

};
uint8_t rGeo[] = {
		0x02,
		  0x04,
		  0x08,
		  0x1A,
		  0x15,
		  0x15,
		  0x11,
		  0x0A

};
uint8_t eGeo[] = {
		0x06,
		  0x09,
		  0x09,
		  0x01,
		  0x01,
		  0x09,
		  0x09,
		  0x06

};
uint8_t wGeo[] = {
  0x0A,
  0x15,
  0x10,
  0x10,
  0x1E,
  0x11,
  0x11,
  0x0E
};
/**
  * @brief  The application entry point.
  * @retval int
  */
void printItvirteba(uint8_t row, uint8_t column){
	HD44780_Clear();
	HD44780_CreateSpecialChar(1, iGeo); // i
	HD44780_SetCursor(column, row);
	HD44780_PrintSpecialChar(1);
	HD44780_CreateSpecialChar(2, TGeo); //t
	HD44780_SetCursor(column+1, row);
	HD44780_PrintSpecialChar(2);
	HD44780_CreateSpecialChar(3, vGeo);//v
	HD44780_SetCursor(column+2, row);
	HD44780_PrintSpecialChar(3);
	HD44780_SetCursor(column+3, row);
	HD44780_PrintSpecialChar(1); // i
	HD44780_CreateSpecialChar(5, rGeo); //r
	HD44780_SetCursor(column+4, row);
	HD44780_PrintSpecialChar(5);
	HD44780_CreateSpecialChar(6, tGeo); // t
	HD44780_SetCursor(column+5, row);
	HD44780_PrintSpecialChar(6);
	HD44780_CreateSpecialChar(7, eGeo); //e
	HD44780_SetCursor(column+6, row);
	HD44780_PrintSpecialChar(7);
	HD44780_CreateSpecialChar(8, bGeo); //b
	HD44780_SetCursor(column+7, row);
	HD44780_PrintSpecialChar(8);
	HD44780_CreateSpecialChar(4, aGeo); //a
	HD44780_SetCursor(column+8, row);
	HD44780_PrintSpecialChar(4);
	HD44780_PrintStr("!");
}

void printMzadVar(uint8_t row, uint8_t column){
	HD44780_Clear();
	HD44780_CreateSpecialChar(1, mGeo);
	HD44780_SetCursor(column, row);
	HD44780_PrintSpecialChar(1);
	HD44780_CreateSpecialChar(2, zGeo);
	HD44780_SetCursor(column+1, row);
	HD44780_PrintSpecialChar(2);
	HD44780_CreateSpecialChar(3, aGeo);
	HD44780_SetCursor(column+2, row);
	HD44780_PrintSpecialChar(3);
	HD44780_CreateSpecialChar(4, dGeo);
	HD44780_SetCursor(column+3, row);
	HD44780_PrintSpecialChar(4);
	HD44780_CreateSpecialChar(5, vGeo);
	HD44780_SetCursor(column+5, row);
	HD44780_PrintSpecialChar(5);
	HD44780_SetCursor(column+6, row);
	HD44780_PrintSpecialChar(3);
	HD44780_CreateSpecialChar(6, rGeo);
	HD44780_SetCursor(column+7, row);
	HD44780_PrintSpecialChar(6);
	HD44780_SetCursor(column+8, row);
	HD44780_PrintStr("!");
	HD44780_SetCursor(6, 1);
	HD44780_PrintStr("-_-");
}
void printUcxoBaratia(uint8_t row, uint8_t column){
		HD44780_Clear();
		HD44780_CreateSpecialChar(1, sGeo);
		HD44780_SetCursor(column, row);
		HD44780_PrintSpecialChar(1);

		HD44780_CreateSpecialChar(2, xGeo);
		HD44780_SetCursor(column+1, row);
		HD44780_PrintSpecialChar(2);

		HD44780_CreateSpecialChar(3, vGeo);
		HD44780_SetCursor(column+2, row);
		HD44780_PrintSpecialChar(3);

		HD44780_CreateSpecialChar(4, aGeo);
		HD44780_SetCursor(column+3, row);
		HD44780_PrintSpecialChar(4);

		HD44780_CreateSpecialChar(5, bGeo);
		HD44780_SetCursor(column+5, row);
		HD44780_PrintSpecialChar(5);

		HD44780_SetCursor(column+6, row);
		HD44780_PrintSpecialChar(4);

		HD44780_CreateSpecialChar(6, rGeo);
		HD44780_SetCursor(column+7, row);
		HD44780_PrintSpecialChar(6);

		HD44780_SetCursor(column+8, row);
		HD44780_PrintSpecialChar(4);

		HD44780_CreateSpecialChar(7, tGeo);
		HD44780_SetCursor(column+9, row);
		HD44780_PrintSpecialChar(7);

		HD44780_CreateSpecialChar(8, iGeo);
		HD44780_SetCursor(column+10, row);
		HD44780_PrintSpecialChar(8);

		HD44780_SetCursor(column+11, row);
		HD44780_PrintSpecialChar(4);
		HD44780_PrintStr("!");

}

void printBlansiAraa(uint8_t row, uint8_t column){

	HD44780_Clear();
	HD44780_CreateSpecialChar(1, bGeo); //b
	HD44780_SetCursor(column, row);
	HD44780_PrintSpecialChar(1);

	HD44780_CreateSpecialChar(2, aGeo); //a
	HD44780_SetCursor(column+1, row);
	HD44780_PrintSpecialChar(2);

	HD44780_CreateSpecialChar(3, lGeo); // l
	HD44780_SetCursor(column+2, row);
	HD44780_PrintSpecialChar(3);


	HD44780_SetCursor(column+3, row); //a
	HD44780_PrintSpecialChar(2);

	HD44780_CreateSpecialChar(4, nGeo); //n
	HD44780_SetCursor(column+4, row);
	HD44780_PrintSpecialChar(4);

	HD44780_CreateSpecialChar(5, sGeo); //s
	HD44780_SetCursor(column+5, row);
	HD44780_PrintSpecialChar(5);

	HD44780_CreateSpecialChar(6, iGeo); //i
	HD44780_SetCursor(column+6, row);
	HD44780_PrintSpecialChar(6);

	HD44780_SetCursor(column+8, row); //a
	HD44780_PrintSpecialChar(2);

	HD44780_CreateSpecialChar(7, rGeo);
	HD44780_SetCursor(column+9, row);
	HD44780_PrintSpecialChar(7);


	HD44780_SetCursor(column+10, row);//a
	HD44780_PrintSpecialChar(2);

	HD44780_PrintSpecialChar(2);
	HD44780_PrintStr("!");
}

void printMiadetBarati(uint8_t row, uint8_t column){
		HD44780_Clear();
		HD44780_CreateSpecialChar(1, mGeo);
		HD44780_SetCursor(column, row);
		HD44780_PrintSpecialChar(1);
		HD44780_CreateSpecialChar(2, iGeo);
		HD44780_SetCursor(column+1, row);
		HD44780_PrintSpecialChar(2);
		HD44780_CreateSpecialChar(3, aGeo);
		HD44780_SetCursor(column+2, row);
		HD44780_PrintSpecialChar(3);
		HD44780_CreateSpecialChar(4, dGeo);
		HD44780_SetCursor(column+3, row);
		HD44780_PrintSpecialChar(4);
		HD44780_CreateSpecialChar(5, eGeo);
		HD44780_SetCursor(column+4, row);
		HD44780_PrintSpecialChar(5);
		HD44780_CreateSpecialChar(6, tGeo);
		HD44780_SetCursor(column+5, row);
		HD44780_PrintSpecialChar(6);
		HD44780_CreateSpecialChar(7, bGeo);
		HD44780_SetCursor(column+7, row);
		HD44780_PrintSpecialChar(7);
		HD44780_SetCursor(column+8, row);
		HD44780_PrintSpecialChar(3);
		HD44780_CreateSpecialChar(8, rGeo);
		HD44780_SetCursor(column+9, row);
		HD44780_PrintSpecialChar(8);
		HD44780_SetCursor(column+10, row);
		HD44780_PrintSpecialChar(3);
		HD44780_SetCursor(column+11, row);
		HD44780_PrintSpecialChar(6);
		HD44780_SetCursor(column+12, row);
		HD44780_PrintSpecialChar(2);
		HD44780_SetCursor(column+2, row+1);
		HD44780_PrintStr("0.10 GEL");


}

void prinWarmateba(uint8_t row, uint8_t column){
		HD44780_Clear();
		HD44780_CreateSpecialChar(1, wGeo);
		HD44780_SetCursor(column, row);
		HD44780_PrintSpecialChar(1);
		HD44780_CreateSpecialChar(2, aGeo);
		HD44780_SetCursor(column+1, row);
		HD44780_PrintSpecialChar(2);
		HD44780_CreateSpecialChar(3, rGeo);
		HD44780_SetCursor(column+2, row);
		HD44780_PrintSpecialChar(3);
		HD44780_CreateSpecialChar(4, mGeo);
		HD44780_SetCursor(column+3, row);
		HD44780_PrintSpecialChar(4);
		HD44780_PrintSpecialChar(2); // a
		HD44780_CreateSpecialChar(5, tGeo); //t
		HD44780_SetCursor(column+5, row);
		HD44780_PrintSpecialChar(5);
		HD44780_CreateSpecialChar(6, eGeo); //e
		HD44780_SetCursor(column+6, row);
		HD44780_PrintSpecialChar(6);
		HD44780_CreateSpecialChar(7, bGeo); //b
		HD44780_SetCursor(column+7, row);
		HD44780_PrintSpecialChar(7);
		HD44780_PrintSpecialChar(2);
		HD44780_PrintStr("!");

}
void printShecdoma(uint8_t row, uint8_t column){
	HD44780_Clear();
	HD44780_CreateSpecialChar(1, SGeo);
	HD44780_SetCursor(column, row);
	HD44780_PrintSpecialChar(1);
	HD44780_CreateSpecialChar(2, eGeo);
	HD44780_SetCursor(column+1, row);
	HD44780_PrintSpecialChar(2);
	HD44780_CreateSpecialChar(3, cGeo);
	HD44780_SetCursor(column+2, row);
	HD44780_PrintSpecialChar(3);
	HD44780_CreateSpecialChar(4, dGeo);
	HD44780_SetCursor(column+3, row);
	HD44780_PrintSpecialChar(4);
	HD44780_CreateSpecialChar(5, oGeo);
	HD44780_SetCursor(column+4, row);
	HD44780_PrintSpecialChar(5);
	HD44780_CreateSpecialChar(6, mGeo);
	HD44780_SetCursor(column+5, row);
	HD44780_PrintSpecialChar(6);
	HD44780_CreateSpecialChar(7, aGeo);
	HD44780_SetCursor(column+6, row);
	HD44780_PrintSpecialChar(7);
	HD44780_SetCursor(column+7, row);
	HD44780_PrintStr("!");
}

void printDaicadet(uint8_t row, uint8_t column){
	HD44780_Clear();
	HD44780_CreateSpecialChar(1, dGeo);
	HD44780_SetCursor(column, row);
	HD44780_PrintSpecialChar(1);
	HD44780_CreateSpecialChar(2, aGeo);
	HD44780_SetCursor(column+1, row);
	HD44780_PrintSpecialChar(2);
	HD44780_CreateSpecialChar(3, iGeo);
	HD44780_SetCursor(column+2, row);
	HD44780_PrintSpecialChar(3);
	HD44780_CreateSpecialChar(4, cGeo);
	HD44780_SetCursor(column+3, row);
	HD44780_PrintSpecialChar(4);
	HD44780_SetCursor(column+4, row);
	HD44780_PrintSpecialChar(2);
	HD44780_SetCursor(column+5, row);
	HD44780_PrintSpecialChar(1);
	HD44780_CreateSpecialChar(5, eGeo);
	HD44780_SetCursor(column+6, row);
	HD44780_PrintSpecialChar(5);
	HD44780_CreateSpecialChar(6, tGeo);
	HD44780_SetCursor(column+7, row);
	HD44780_PrintSpecialChar(6);
	HD44780_PrintStr("!");
}
void printVemzadebi(uint8_t row, uint8_t column){
	HD44780_Clear();
	HD44780_CreateSpecialChar(1, vGeo);
	HD44780_SetCursor(column, row);
	HD44780_PrintSpecialChar(1);
	HD44780_CreateSpecialChar(2, eGeo);
	HD44780_SetCursor(column+1, row);
	HD44780_PrintSpecialChar(2);
	HD44780_CreateSpecialChar(3, mGeo);
	HD44780_SetCursor(column+2, row);
	HD44780_PrintSpecialChar(3);
	HD44780_CreateSpecialChar(4, zGeo);
	HD44780_SetCursor(column+3, row);
	HD44780_PrintSpecialChar(4);
	HD44780_CreateSpecialChar(5, aGeo);
	HD44780_SetCursor(column+4, row);
	HD44780_PrintSpecialChar(5);
	HD44780_CreateSpecialChar(6, dGeo);
	HD44780_SetCursor(column+5, row);
	HD44780_PrintSpecialChar(6);
	HD44780_SetCursor(column+6, row);
	HD44780_PrintSpecialChar(2);
	HD44780_CreateSpecialChar(7, bGeo);
	HD44780_SetCursor(column+7, row);
	HD44780_PrintSpecialChar(7);
	HD44780_CreateSpecialChar(8, iGeo);
	HD44780_SetCursor(column+8, row);
	HD44780_PrintSpecialChar(8);
	HD44780_PrintStr("!");
}

void printBalansi(uint8_t row, uint8_t column){
	HD44780_Clear();
	HD44780_CreateSpecialChar(1, bGeo);
	HD44780_SetCursor(column, row);
	HD44780_PrintSpecialChar(1);

	HD44780_CreateSpecialChar(2, aGeo);
	HD44780_SetCursor(column+1, row);
	HD44780_PrintSpecialChar(2);

	HD44780_CreateSpecialChar(3, lGeo);
	HD44780_SetCursor(column+2, row);
	HD44780_PrintSpecialChar(3);

	HD44780_PrintSpecialChar(2);
	HD44780_SetCursor(column+3, row);

	HD44780_CreateSpecialChar(5, nGeo);
	HD44780_SetCursor(column+4, row);
	HD44780_PrintSpecialChar(5);

	HD44780_CreateSpecialChar(6, sGeo);
	HD44780_SetCursor(column+5, row);
	HD44780_PrintSpecialChar(6);

	HD44780_CreateSpecialChar(7, iGeo);
	HD44780_SetCursor(column+6, row);
	HD44780_PrintSpecialChar(7);

	HD44780_PrintStr(": ");
}
