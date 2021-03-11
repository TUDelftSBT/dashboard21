/*
 * LEDS.c
 * de functies om de LEDS op de PCB aan te zetten
 *
 *  Created on: Feb 11, 2021
 *      Author: thomas Kamminga
 *
 */

#include "main.h"

void Toggle_Green_LED(uint8_t ToggleNum){  // Turn on green led if 1 is given or off with 0
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, ToggleNum);
}

void Toggle_CB_TRIP_LED(uint8_t ToggleNum){  // Turn on CB led if 1 is given or off with 0
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, ToggleNum);
}

void Toggle_Yellow_LED(uint8_t ToggleNum){  // Turn on yellow led if 1 is given or off with 0
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, ToggleNum);
}

void Toggle_Red_LED(uint8_t ToggleNum){  // Turn on red led if 1 is given or off with 0
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, ToggleNum);
}

void Blink_Green_LED(uint8_t delay){
	Toggle_Green_LED(1);
	HAL_Delay(delay);
	Toggle_Green_LED(0);
}
