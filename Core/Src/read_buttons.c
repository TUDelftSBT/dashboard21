/*
 * read_buttons.c
 *
 *  Created on: Feb 25, 2021
 *      Author: Thomas Kamminga
 *
 *     	Make a 8-bit message that for every bit has the status of a button
 */

#include "main.h"



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


