/*
 * WIT.h
 *
 *  Created on: Aug 11, 2024
 *      Author: Admin
 */

#ifndef WIT_H_
#define WIT_H_
#include "main.h"

//L?nh g?i d?n c?m bi?n


typedef struct
{
	float x;
	float y;
	float z;
}Angle_ReadTypeDef;
void WIT_Data_In(uint8_t data);
void WIT_Data_Process(Angle_ReadTypeDef *angle, uint8_t aData[]);
void WIT_Reset_Zero(UART_HandleTypeDef *huart);
void WIT_Reset_Flag(void);


#endif /* WIT_H_ */



