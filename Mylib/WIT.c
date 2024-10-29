/*
 * WIT.c
 *
 *  Created on: Aug 11, 2024
 *      Author: Admin
 */
#include "WIT.h"

static uint8_t unlock_cmd[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
static uint8_t reset_zero_cmd[5] = {0xFF, 0xAA, 0x01, 0x04, 0x00};
static uint8_t save_cmd[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
static uint8_t flag;

static uint8_t buff[11];
static uint8_t recieve_flag = 0;
static uint8_t data_len = 0;
static uint8_t uart_flag = 0;
char checksum;


//-------------------------Tính góc---------------------------------------------
void WIT_Data_Process(Angle_ReadTypeDef *angle, uint8_t aData[])
{
	  if (uart_flag)
	  {
		  //-------------Tính checksum-------------------------------------------
		  checksum = 0x55 + 0x53 + buff[2] + buff[3] + buff[4]
					+ buff[5] + buff[6] + buff[7] + buff[8] + buff[9];
		  //------Kiểm tra checksum và byte thứ 2-------------------------------
		  if (buff[1] == 0x53 && checksum == buff[10])
		  {
			  angle->x = ((float)((short)buff[3] << 8| buff[2])/32768.0)*180.0;
			  angle->y = ((float)((short)buff[5] << 8| buff[4])/32768.0)*180.0;
			  angle->z = ((float)((short)buff[7] << 8| buff[6])/32768.0)*180.0;
				aData[0] = buff[2];
				aData[1] = buff[3];
				aData[2] = buff[4];
				aData[3] = buff[5];
				aData[4] = buff[6];
				aData[5] = buff[7];
		  }
		  uart_flag = 0;
		  recieve_flag = 0;
		  data_len = 0;

	  }
}
//-----------Nhận dữ liệu đầu vào-------------------------------------------
void WIT_Data_In(uint8_t data)
{
	//-------Kiểm tra byte đầu tiên-----------------------------------------
	if (data == 0x55 && data_len == 0)
	{
		recieve_flag = 1;
	}
//-----------Điền các byte con lại vào mảng buff---------------------------
	if (recieve_flag)
	{
		buff[data_len] = data;
		data_len++;

	}
	if (data_len > 10)
	{
		uart_flag = 1;
	}
}

//--------------Reset Z về 0 độ-----------------------------------------------------
void WIT_Reset_Zero(UART_HandleTypeDef *huart)
{
	  static uint32_t time;
	  
	  if (HAL_GetTick() - time > 100 && flag != 3)
	  {
		  switch (flag)
		  {
		  case 0:
			  HAL_UART_Transmit(huart, unlock_cmd, sizeof(unlock_cmd),100);
			  flag++;
			  time = HAL_GetTick();
			  break;
		  case 1:
			  HAL_UART_Transmit(huart, reset_zero_cmd, sizeof(reset_zero_cmd),100);
			  flag++;
			  time = HAL_GetTick();
			  break;
		  case 2:
			  HAL_UART_Transmit(huart, save_cmd, sizeof(save_cmd), 100);
			  flag++;
			  time = HAL_GetTick();
		  }
	  }
}

void WIT_Reset_Flag(void)
{
	flag = 0;
}
