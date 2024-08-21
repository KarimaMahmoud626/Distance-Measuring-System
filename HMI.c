/*
 * HMI.c
 *
 *  Created on: Jun 24, 2024
 *      Author: zakwajbl
 */

#include"lcd.h"
#include"ultrasonic_snsr.h"
#include<avr/io.h>

int main(void)
{
	uint16 distance = 0;
	Ultrasonic_init();
	LCD_init();
	LCD_displayString("Distance= ");
	LCD_displayStringRowColumn(0,13,"cm");
	SREG |= (1<<7);
	while(1){
		LCD_moveCursor(0,10);
		distance = Ultrasonic_readDistance();
		if(distance >= 100)
		{
			LCD_integerToString(distance);
		}
		else
		{
			LCD_integerToString(distance);
			/* In case the digital value is two or one digits print space in the next digit place */
			LCD_displayCharacter(' ');
		}

	}
}
