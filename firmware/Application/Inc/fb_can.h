#include "can.h"

#ifndef _FB_CAN_
#define _FB_CAN_

#define MAX_DUTY 				1000.0f
#define MAX_DUTY_1 				0.001f
#define MAX_DUTY_HALF			500.0f
#define MAX_DUTY_HALF_1			0.002f

#define MOTOR_ID				1
#define CAN_STD_ID_TX			0x0011
#define CAN_EXT_ID_TX			(MOTOR_ID)
#define CAN_STD_ID_RX			0x0012
#define CAN_EXT_ID_RX			(MOTOR_ID)
#define CAN_STD_ID_RX_LOOPBACK	(CAN_STD_ID_TX)
#define CAN_EXT_ID_RX_LOOPBACK	(CAN_EXT_ID_TX)

typedef struct data_bus_tx_can_s
{
	int16_t _rpm_d10:16; 			/* positive and negative numbers (-32768 32767)*/
	uint16_t _voltage_x2:10;		/* only positive numbers  (0 1023)*/
	int16_t _current_x5:11;			/* positive and negative numbers (-1024 1023)*/
	int16_t _duty_x1000:11;			/* positive and negative numbers (-1024 1023) */
	int8_t _fetTemp;				/* positive and negative numbers (-128 127) */
	uint8_t _faultCode;				/* only positive numbers (0 255) */
} data_bus_tx_can_t;

typedef struct data_bus_rx_can_s
{
    int16_t dutyList[4];
} data_bus_rx_can_t;

extern CAN_TxHeaderTypeDef   	TxHeader;
extern CAN_RxHeaderTypeDef   	RxHeader;
extern uint8_t               	TxData[8];
extern uint8_t               	RxData[8];
extern uint32_t             	TxMailbox;
extern CAN_FilterTypeDef		canfilterconfig;
extern uint32_t               	can_bus_fault;

extern data_bus_tx_can_t data_bus_tx;
extern data_bus_rx_can_t data_bus_rx;

void fb_canbus_init(void);
void fb_canbus_process(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
//void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);

#endif
