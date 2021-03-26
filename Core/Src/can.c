/*
 * can.c
 *
 *  Created on: 26 Feb 2021
 *      Author: thoma
 */

#include "main.h"
#include "can.h"

uint8_t can_msg_rx_hydrogenAlarm_data[1];
uint8_t rr_hydrogenAlarm_handle = 0;

uint8_t can_msg_rx_WC_data[2];
uint8_t rr_WC_handle = 0;

uint8_t rr_can_send = 0;


CAN_TxHeaderTypeDef CAN1TxHeader;
CAN_RxHeaderTypeDef CAN1RxHeader;
uint8_t CAN1RxData[8];

uint8_t CheckSendMessage(uint8_t changedButtons){
	uint8_t SendOrNot = changedButtons || rr_can_send;
	rr_can_send = 0;
	return SendOrNot;
}

void CAN1ReceiveMsg(void)
{
	if (CAN1RxHeader.StdId == can_msg_rx_hydrogenAlarm_id)	//if the message in the buffer is from the hydrogen alarm store it in the proper list
	{
		can_msg_rx_hydrogenAlarm_data[0] = CAN1RxData[0];
		rr_hydrogenAlarm_handle = 1;
	}

	else if (CAN1RxHeader.StdId == can_msg_rx_WC_id) //if the message in the buffer is from the EMS store it in the proper list
	{
		can_msg_rx_WC_data[0] = CAN1RxData[0];
		can_msg_rx_WC_data[1] = CAN1RxData[1];
		rr_WC_handle = 1;
	}
}



