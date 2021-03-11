/*
 * can.h
 *
 *  Created on: 26 Feb 2021
 *      Author: thoma
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_





#endif /* INC_CAN_H_ */

extern uint8_t rr_hydrogenAlarm_handle;
extern uint8_t can_msg_rx_hydrogenAlarm_data[0];

extern uint8_t rr_EMS_handle;
extern uint8_t can_msg_rx_EMS_data[0];

extern CAN_TxHeaderTypeDef CAN1TxHeader;
extern CAN_RxHeaderTypeDef CAN1RxHeader;

extern uint8_t CAN1RxData[8];

void CAN1ReceiveMsg(void);
int isNthBitSet(uint8_t, int);
