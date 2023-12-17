#include "fb_can.h"
#include "fb_state_machine.h"
#include "stm32f7xx_hal_can.h"

CAN_TxHeaderTypeDef   	TxHeader;
CAN_RxHeaderTypeDef   	RxHeader;
uint8_t               	TxData[8];
uint8_t               	RxData[8];
uint32_t              	TxMailbox = CAN_TX_MAILBOX0;
CAN_FilterTypeDef 		canfilterconfig;
uint32_t               	can_bus_fault = 0;

data_bus_tx_can_t data_bus_tx;
data_bus_rx_can_t data_bus_rx;

void fb_canbus_init(void)
{
	canfilterconfig.FilterBank = 18;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.FilterIdHigh = CAN_STD_ID_RX_LOOPBACK << 5; // loopback
	canfilterconfig.FilterIdLow = 0x0000; // loopback
	canfilterconfig.FilterMaskIdHigh = CAN_STD_ID_RX_LOOPBACK << 5;  // loopback
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterActivation = ENABLE;
	canfilterconfig.SlaveStartFilterBank = 20;

	if(HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig) != HAL_OK)
	{
	  /* Reception Error */
		can_bus_fault |= 0x0001;
	}

	/* Configure Transmission process */
	TxHeader.StdId = CAN_STD_ID_TX;
	TxHeader.ExtId = MOTOR_ID;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
//	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;

	if(HAL_CAN_Start(&hcan1) != HAL_OK)
	{
	  /* Reception Error */
		can_bus_fault |= 0x0002;
	}
	if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
	  /* Reception Error */
		can_bus_fault |= 0x0004;
	}
}

void fb_canbus_process(void)
{
//	data_bus_tx._rpm_d10 = param.omega_hat_pu * MOTOR_NOMINAL_SPEED * 10;
//	data_bus_tx._voltage_x2 = param.motor_voltage_u_pu * MOTOR_NOMINAL_VOLTAGE * 2;
//	data_bus_tx._current_x5 = param.inverter_current_q * MOTOR_NOMINAL_CURRENT;
//	data_bus_tx._duty_x1000 = param.inverter_current_q * MAX_DUTY;
//	data_bus_tx._fetTemp = param.temperature_igbt_inverter_u * INVERTER_NOMINAL_TEMPERATURE;
//	data_bus_tx._faultCode = param.global_fault_state_description;

//	TxData[0] = (int8_t)(((data_bus_tx._rpm_d10)>>8)&(0xFF));
//	TxData[1] = (int8_t)((data_bus_tx._rpm_d10)&(0xFF));
//	TxData[2] = (int8_t)(((data_bus_tx._voltage_x2)>>2)&(0xFF));
//	TxData[3] = (int8_t)(((data_bus_tx._voltage_x2)&(0x3)<<6)) + (int8_t)(((data_bus_tx._current_x5)>>5)&(0x3F));
//	TxData[4] = (int8_t)(((data_bus_tx._current_x5)&(0x1F)<<3)) + (int8_t)(((data_bus_tx._duty_x1000)>>8)&(0x7));
//	TxData[5] = (int8_t)((data_bus_tx._duty_x1000)&(0xFF));
//	TxData[6] = (int8_t)((data_bus_tx._faultCode)&(0xFF));
//	TxData[7] = (int8_t)((data_bus_tx._faultCode)&(0xFF));

//	data_bus_tx._rpm_d10 = param.torque_ref * MAX_DUTY;
//	data_bus_tx._voltage_x2 = param.motor_voltage_u_pu * MOTOR_NOMINAL_VOLTAGE * 2;
//	data_bus_tx._current_x5 = param.inverter_current_q * MOTOR_NOMINAL_CURRENT;
//	data_bus_tx._duty_x1000 = param.inverter_current_q * MAX_DUTY;
//	data_bus_tx._fetTemp = param.temperature_igbt_inverter_u * INVERTER_NOMINAL_TEMPERATURE;
//	data_bus_tx._faultCode = param.global_fault_state_description;
//
//	TxData[0] = (int8_t)(((data_bus_tx._rpm_d10)>>8)&(0xFF));
//	TxData[1] = (int8_t)((data_bus_tx._rpm_d10)&(0xFF));
//	TxData[2] = (int8_t)(((data_bus_tx._voltage_x2)>>2)&(0xFF));
//	TxData[3] = (int8_t)(((data_bus_tx._voltage_x2)&(0x3)<<6)) + (int8_t)(((data_bus_tx._current_x5)>>5)&(0x3F));
//	TxData[4] = (int8_t)(((data_bus_tx._current_x5)&(0x1F)<<3)) + (int8_t)(((data_bus_tx._duty_x1000)>>8)&(0x7));
//	TxData[5] = (int8_t)((data_bus_tx._duty_x1000)&(0xFF));
//	TxData[6] = (int8_t)((data_bus_tx._faultCode)&(0xFF));
//	TxData[7] = (int8_t)((data_bus_tx._faultCode)&(0xFF));

	TxData[0] = (int8_t)(param.torque_ref*100);
	TxData[1] = (int8_t)(param.duty_reference);
	TxData[2] = (int8_t)(param.inverter_fault_state_description);
	TxData[3] = (int8_t)(param.global_state);
	TxData[4] = (int8_t)(param.global_fault_state_description);
	TxData[5] = (int8_t)(param.inverter_ctrl_mode);
	TxData[6] = (int8_t)(7);
	TxData[7] = (int8_t)(8);

	if (!HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox))
		{
			if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
			{
			  /* Reception Error */
			  can_bus_fault |= 0x0008;
			}
		}

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
	/* Get RX message */
	if(HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{
		/* Reception Error */
		can_bus_fault |= 0x0016;
	}

//	if ((RxHeader.StdId == CAN_STD_ID_TX) && (RxHeader.IDE == CAN_ID_EXT) && (RxHeader.DLC == 8) && (RxHeader.ExtId == MOTOR_ID))
	if ((RxHeader.StdId == CAN_STD_ID_TX) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 8))
	{
		data_bus_rx.dutyList[0] = (RxData[0] << 8) + RxData[1];

		if (data_bus_rx.dutyList[0] > MAX_DUTY)
		{
			data_bus_rx.dutyList[0] = MAX_DUTY;
		}

		if (data_bus_rx.dutyList[0] < -MAX_DUTY)
		{
			data_bus_rx.dutyList[0] = -MAX_DUTY;
		}

		param.duty_reference = (float)(data_bus_rx.dutyList[0]) * MAX_DUTY_1;
	}
}


