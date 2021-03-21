 /*
 * knoppen_input.c
 *
 *  Created on: 10 Feb 2021
 *      Author: Thomas Kamminga
 */
#include "main.h"

uint8_t ButtonHarbor(void) // 1 if Harbor button selected 0 else
{
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t ButtonHydrogen(void) // 1 if Hydrogen button selected 0 else
{
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14))
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

uint8_t ButtonBatteryRESET(void) // 1 if BatteryRESET button selected 0 else
{
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t ButtonCB_TRIP(void) // 1 if CB TRIP button selected 0 else
{
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t ButtonBilge_ON(void) // 1 if Bilge ON button selected 0 else
{
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


uint8_t ButtonAuto(void) // 1 if auto button selected 0 else
{
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15))
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

uint8_t ButtonOFF(void) // 1 if Bilge ON button selected 0 else
{
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t read_buttons(void){
	uint8_t button_status = 0;

	button_status += ButtonHarbor();
	button_status = button_status <<1;

	button_status += ButtonHydrogen();
	button_status = button_status <<1;

	button_status += ButtonBatteryRESET();
	button_status = button_status <<1;

	button_status += ButtonCB_TRIP();
	button_status = button_status <<1;

	button_status += ButtonBilge_ON();
	button_status = button_status <<1;

	button_status += ButtonAuto();
	button_status = button_status <<1;

	button_status += ButtonOFF();
	button_status = button_status <<1;

	return button_status;
}

