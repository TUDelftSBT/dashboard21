/*
 * can.c
 *
 *  Created on: 26 Feb 2021
 *      Author: thoma
 */

#include "main.h"
#include "can.h"
const uint32_t can_msg_tx_dashboard_Buttons_id = 100; // dashboard id

const uint32_t can_msg_rx_hydrogenAlarm_id = 1600; // hydrogen alarm id, dataframe and handle
uint8_t can_msg_rx_hydrogenAlarm_data[0];
uint8_t rr_hydrogenAlarm_handle = 0;

const uint32_t can_msg_rx_EMS_id = 300;	// EMS id dataframe and handle
uint8_t can_msg_rx_EMS_data[0];
uint8_t rr_EMS_handle = 0;


CAN_TxHeaderTypeDef CAN1TxHeader;
CAN_RxHeaderTypeDef CAN1RxHeader;
uint8_t CAN1RxData[8];



void CAN1ReceiveMsg(void)
{
	if (CAN1RxHeader.StdId == can_msg_rx_hydrogenAlarm_id)	//if the message in the buffer is from the hydrogen alarm store it in the proper list
	{
		can_msg_rx_hydrogenAlarm_data[0] = CAN1RxData[0];
		rr_hydrogenAlarm_handle = 1;
	}

	else if (CAN1RxHeader.StdId == can_msg_rx_EMS_id) //if the message in the buffer is from the EMS store it in the proper list
	{
		can_msg_rx_EMS_data[0] = CAN1RxData[0];
		rr_EMS_handle = 1;
	}
}



