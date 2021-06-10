/*
 * can.h
 *
 *  Created on: 26 Feb 2021
 *      Author: thoma
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_





#endif /* INC_CAN_H_ */

static const uint32_t can_msg_tx_dashboard_Buttons_id = 500; // dashboard id
static const uint32_t can_msg_rx_hydrogenAlarm_id = 1600; // hydrogen alarm id, dataframe and handle
static const uint32_t can_msg_rx_hydrogenAlarm2_id = 1608; // hydrogen alarm id, dataframe and handle
static const uint32_t can_msg_rx_WC_id = 200;	// WC id dataframe and handle

extern uint8_t rr_can_send;

extern uint8_t rr_hydrogenAlarm_handle;
extern uint8_t can_msg_rx_hydrogenAlarm_data[8];
extern uint8_t rr_hydrogenAlarm_timeout;
extern uint32_t HydrogenTimeLastMessage;

extern uint8_t can_msg_rx_hydrogenAlarm2_data[8];
extern uint8_t rr_hydrogenAlarm2_handle;
extern uint8_t rr_hydrogenAlarm2_timeout;
extern uint32_t HydrogenTimeLastMessage2;

extern uint8_t rr_WC_handle;
extern uint8_t can_msg_rx_WC_data[1];

extern CAN_TxHeaderTypeDef CAN1TxHeader;
extern CAN_RxHeaderTypeDef CAN1RxHeader;

extern uint8_t CAN1RxData[8];

void CAN1ReceiveMsg(uint32_t count);
uint8_t CheckSendMessage(uint8_t changedButtons);
