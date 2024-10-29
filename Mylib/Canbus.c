#include "Canbus.h"

static uint16_t encoder_tx_delay;
static uint16_t IMU_tx_delay;
static uint32_t TxMailbox[3];
static uint16_t encoder_feedback;
static uint16_t IMU_feedback;

//Dùng khi MCU có 1 CAN connection và chi dùng filteridhigh
//Khai báo filter
void CAN_Single_FilterMaskIDHigh_Config(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig, uint8_t filter_bank, 
															uint8_t fifo, uint16_t filteridhigh, uint16_t filtermaskidhigh)
{
	sFilterConfig->FilterActivation 		= CAN_FILTER_ENABLE;
	sFilterConfig->FilterBank						= filter_bank;
	sFilterConfig->FilterFIFOAssignment	= fifo;									//CAN_RX_FIFO0 || CAN_RX_FIFO1
	sFilterConfig->FilterIdHigh					= filteridhigh;
	sFilterConfig->FilterIdLow					= 0x0000;
	sFilterConfig->FilterMaskIdHigh			=	filtermaskidhigh;
	sFilterConfig->FilterMaskIdLow			= 0x0000;
	sFilterConfig->FilterMode						= CAN_FILTERMODE_IDMASK;
	sFilterConfig->FilterScale					= CAN_FILTERSCALE_32BIT;
	sFilterConfig->SlaveStartFilterBank	= 0;
	
	HAL_CAN_ConfigFilter(hcan, sFilterConfig);
}
// Khoi tao ID cho TxHeader
void CAN_TxHeader_Init(CAN_TxHeaderTypeDef *TxHeader, uint32_t DLC, uint32_t StdId)
{
	TxHeader->DLC 								= DLC;
	TxHeader->ExtId								= 0;
	TxHeader->IDE									= CAN_ID_STD;
  TxHeader->RTR									= CAN_RTR_DATA;
	TxHeader->StdId								= StdId;
	TxHeader->TransmitGlobalTime	= DISABLE;
}

//Bat dâu thiêt bi
void CAN_Start(CAN_HandleTypeDef *hcan, uint8_t device ,CAN_TxHeaderTypeDef *TxHeader, uint16_t Tx_Delay)
{
	uint8_t data[3];
	//chon khoi dông IMU/Encoder
	data[0] = device;
	//chon chu kì truyên data
	data[1] = Tx_Delay>>8 & 0xFF;
	data[2] = Tx_Delay & 0xFF;
	if (device == CAN_START_ENCODER)
	{
		encoder_tx_delay = Tx_Delay;
	}
	else if (device == CAN_START_IMU)
	{
		IMU_tx_delay = Tx_Delay;
	}
	HAL_CAN_AddTxMessage(hcan, TxHeader, data, &TxMailbox[0]);
}
//Reser thiet bi
void CAN_Reset(CAN_HandleTypeDef *hcan, uint8_t device, CAN_TxHeaderTypeDef *TxHeader)
{
	HAL_CAN_AddTxMessage(hcan, TxHeader, &device, &TxMailbox[0]);
}

//Yêu cau slave gui data theo chu kì 
void CAN_RequestData(CAN_HandleTypeDef *hcan,CAN_TxHeaderTypeDef *TxHeader, uint8_t device ,uint16_t tx_delay)
{
	uint8_t data[3];
	data[0] = device;
	//chuyen tu uint16_t sang 2 uint8_t
	data[1] = tx_delay>>8 & 0xFF;
	data[2] = tx_delay & 0xFF;
	//Gui chu kì truyên data cho các thiêt bi
	if (device == CAN_ENCODER_TXDELAY)
	{
		encoder_tx_delay =  tx_delay;
	}
	else if (device == CAN_IMU_TXDELAY)
	{
		IMU_tx_delay = tx_delay;
	}
	HAL_CAN_AddTxMessage(hcan, TxHeader, data, &TxMailbox[0]);
}

//Hoat dông nhu flag bat dâu
uint8_t CAN_Encoder_TxStart_Init(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])
{
	uint8_t flag = 0;
	HAL_CAN_GetRxMessage(hcan, RxFifo, pHeader, aData);
	if (aData[0] == CAN_START_ENCODER)
	{
		encoder_tx_delay = (uint16_t)(((uint16_t)aData[1]<<8) | aData[2]);
		flag = 1;
	}
	if (flag) {return 1;}
	return 0;
}

uint8_t CAN_IMU_TxStart_Init(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])
{
	uint8_t flag = 0;
	HAL_CAN_GetRxMessage(hcan, RxFifo, pHeader, aData);
	if (aData[0] == CAN_START_IMU)
	{
		IMU_tx_delay = (uint16_t)(((uint16_t)aData[1]<<8) | aData[2]);
		flag = 1;
	}
	if (flag) {return 1;}
	return 0;
}

//Nhân chu kì gui data
void CAN_IMU_TxDelay(uint8_t aData[])
{
	if (aData[0] == CAN_IMU_TXDELAY)
	{
		IMU_tx_delay = (uint16_t)(((uint16_t)aData[1]<<8) | aData[2]); //Chuyên 2 uint8_t thành uint16_t
	}
}

void CAN_Encoder_TxDelay(uint8_t aData[])
{
	if (aData[0] == CAN_ENCODER_TXDELAY)
	{
		encoder_tx_delay = (uint16_t)(((uint16_t)aData[1]<<8) | aData[2]);
	}
}

//Nhân data tu Encoder/IMU
void CAN_Encoder_Recieve(uint8_t aData[], Float_to_Uint8_t *x_position, Float_to_Uint8_t *y_position)
{
	if (aData[0] == CAN_ENCODER_POSITION)
	{
		if (aData[5] == 'X')
		{
			x_position->byte[0] =	aData[1];
			x_position->byte[1]	=	aData[2];
			x_position->byte[2]	=	aData[3];
			x_position->byte[3]	=	aData[4];
		}
		else if (aData[5] == 'Y')
		{
			y_position->byte[0] =	aData[1];
			y_position->byte[1]	=	aData[2];
			y_position->byte[2]	=	aData[3];
			y_position->byte[3]	=	aData[4];
		}
	}
}

void CAN_IMU_Recieve(uint8_t aData[], Angle_ReadTypeDef *angle)
{
	if (aData[0] == CAN_IMU_ANGLE)
	{
		angle->x = ((float)((short)aData[2] << 8| aData[1])/32768.0)*180.0; //chuyen tu uint8_t sang float
		angle->y = ((float)((short)aData[4] << 8| aData[3])/32768.0)*180.0;
		angle->z = ((float)((short)aData[6] << 8| aData[5])/32768.0)*180.0;
	}
}

//Gui data cua Encoder/IMU
void CAN_Encoder_Transmit(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *TxHeader, Float_to_Uint8_t x_position, Float_to_Uint8_t y_position)
{
	static uint32_t time;
	static uint8_t flag;
	uint8_t data[6];
	if ((HAL_GetTick()- time) >= encoder_tx_delay) //Gui theo chu kì duoc yêu cau boi master
	{	
		//Luân phiên gui vi tri truc X và Y
		if (flag)
		{
			data[0] = CAN_ENCODER_POSITION;
			data[1]	= x_position.byte[0];
			data[2]	= x_position.byte[1];
			data[3]	= x_position.byte[2];
			data[4]	= x_position.byte[3];
			data[5]	= 'X';
			HAL_CAN_AddTxMessage(hcan, TxHeader, data, &TxMailbox[0]);
			flag = 0;
		}
		else
		{
			data[0] = CAN_ENCODER_POSITION;
			data[1]	= y_position.byte[0];
			data[2]	= y_position.byte[1];
			data[3]	= y_position.byte[2];
			data[4]	= y_position.byte[3];
			data[5]	= 'Y';
			HAL_CAN_AddTxMessage(hcan, TxHeader, data, &TxMailbox[0]);
			flag = 1;
		}
		time = HAL_GetTick();
	}
}

void CAN_IMU_Transmit(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *TxHeader, uint8_t aData[])
{
	static uint32_t time;
	uint8_t data[7];
	if ((HAL_GetTick()- time) >= IMU_tx_delay) //Gui theo chu kì duoc yêu cau boi master
	{
		data[0] = CAN_IMU_ANGLE;
		data[1]	= aData[0];
		data[2]	= aData[1];
		data[3]	= aData[2];
		data[4]	= aData[3];
		data[5]	= aData[4];
		data[6]	= aData[5];
		if (data[1] + data[2] + data[3] + data[4] + data[5] + data[6]
				== aData[0] + aData[1] + aData[2] + aData[3] + aData[4]+ aData[5]) //checksum truoc khi gui
		{HAL_CAN_AddTxMessage(hcan, TxHeader, data, &TxMailbox[1]);}
	}
}

//Feedback chu kì truyên data cho master
void CAN_Encoder_Feedback(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *TxHeader, uint8_t RxData[])
{
	uint32_t time;
	uint8_t aData[3];
	if((HAL_GetTick()-time)>=200)
	{
		aData[0] = CAN_ENCODER_FEEDBACK;
		aData[1] = (encoder_tx_delay >> 8) & 0xFF;
		aData[2] = encoder_tx_delay & 0xFF;
		RxData[0] = 0;
		HAL_CAN_AddTxMessage(hcan, TxHeader, aData, &TxMailbox[0]);
		time = HAL_GetTick();
	}
}

void CAN_IMU_Feedback(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *TxHeader, uint8_t RxData[])
{
	uint32_t time;
	uint8_t aData[3];
	if((HAL_GetTick()-time)>=200)
	{
		aData[0] = CAN_IMU_FEEDBACK;
		aData[1] = (IMU_tx_delay >> 8) & 0xFF;
		aData[2] = IMU_tx_delay & 0xFF;
		RxData[0] = 0;
		HAL_CAN_AddTxMessage(hcan, TxHeader, aData, &TxMailbox[0]);
		time = HAL_GetTick();
	}
}

//Nhân thông tin feedback
uint16_t CAN_Encoder_RxFeedback(uint8_t aData[])
{
	if (aData[0] == CAN_ENCODER_FEEDBACK)
	{
		encoder_feedback = (uint16_t)(((uint16_t)aData[1]<<8) | aData[2]);
	}
	return encoder_feedback;
}

uint16_t CAN_IMU_RxFeedback(uint8_t aData[])
{
	if (aData[0] == CAN_IMU_FEEDBACK)
	{
		IMU_feedback = (uint16_t)(((uint16_t)aData[1]<<8) | aData[2]);
	}
	return IMU_feedback;
}

