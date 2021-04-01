 /*
 * knoppen_input.c
 *
 *  Created on: 10 Feb 2021
 *      Author: Thomas Kamminga
 */
#include "main.h"

uint8_t ButtonHydrogen(void) // 1 if Hydrogen button selected 0 else
{
	return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
}

uint8_t ButtonBatteryRESET(void) // 1 if BatteryRESET button selected 0 else
{
	return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
}

uint8_t ButtonCB_TRIP(void) // 1 if CB TRIP button selected 0 else
{
	return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
}

uint8_t ButtonChargeON(void) // 1 if battery charge button selected 0 else
{
	return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
}

uint8_t ButtonMotor(void) // 1 if Bilge ON button selected 0 else
{
	return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
}

uint8_t read_buttons(void){
	uint8_t button_status = 0;

	button_status += ButtonHydrogen();
	button_status = button_status <<1;

	button_status += ButtonBatteryRESET();
	button_status = button_status <<1;

	button_status += ButtonCB_TRIP();
	button_status = button_status <<1;

	button_status += ButtonChargeON();
	button_status = button_status <<1;

	button_status += ButtonMotor();

	return button_status;
}

uint8_t read_bulgepump_buttons(void){

	uint8_t button_status = 0;

	button_status += HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10); //Bilge port
	button_status = button_status <<1;

	button_status += HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11); //Bilge motor
	button_status = button_status <<1;

	button_status += HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12); //Bilge crew
	button_status = button_status <<1;

	button_status += HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2);  //Bilge battery
	button_status = button_status <<1;

	button_status += HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);  //Bilge starboard

	return  button_status;
}

