
#include "EncoderPosition.h"

uint8_t offset_flag = 0;


//Cap nhat các giá tri cho encoder qua timer
void Encoder_Update_Data(Encoder_HandleTypeDef *encoder, TIM_HandleTypeDef *htim, uint16_t resolution)
{
	//Gan các giá tri cua timer vào encoder
	encoder->current_direction = (htim->Instance->CR1 & TIM_CR1_DIR) >> 4;
	encoder->current_CNT_value = htim->Instance->CNT;
	encoder->resolution = resolution;
}

void Encoder_Round_Counter(Encoder_HandleTypeDef *encoder) //Ðêm sô vong quay cua encoder qua xung Z
{
	if (encoder->current_direction != 0)
	{
		encoder->round_counter--;
	}
	else
	{
		encoder->round_counter++;
	}	
	//skip 1 xung Z nêu encoder quay nguoc chiêu dang quay
	if(encoder->last_direction != encoder->current_direction)
	{
		if (encoder->current_direction != 0)
		{
			encoder->round_counter++;
		}
		else
		{
			encoder->round_counter--;
		}
	}
	encoder->last_direction = encoder->current_direction;
}	

uint8_t Encoder_Offset_Detect(Encoder_HandleTypeDef *encoder) //Tim dô lech so voi vi tri dat robot
{
	if (!offset_flag)
	{
		encoder->offset = encoder->pulse; //Luu so xung lech so voi vi tri dat robot
		encoder->last_direction = encoder->current_direction;
		offset_flag++;
		return 0;
	}
	else
	{
		return 1; 
	}
}


void Encoder_Pulse_Counter(Encoder_HandleTypeDef *encoder)	//Ðêm sô xung
{
	if (encoder->last_CNT_value > encoder->current_CNT_value)
	{
		if (encoder->current_direction != 0) //CNT dem xuong
		{
			if ((encoder->last_CNT_value - encoder->current_CNT_value) >= 4)
			{
				encoder->pulse--;
				encoder->last_CNT_value = encoder->current_CNT_value;
			}
		}
		else	//CNT dem lên
		{
			if ((encoder->last_CNT_value - encoder->current_CNT_value) >= 4)
			{
				encoder->pulse++;
				encoder->last_CNT_value = encoder->current_CNT_value;
			}
		}
	}
	if (encoder->last_CNT_value < encoder->current_CNT_value)
	{
		if (encoder->current_direction != 0) //CNT dem xuong
		{
			if ((encoder->current_CNT_value - encoder->last_CNT_value) >= 4)
			{
				encoder->pulse--;
				encoder->last_CNT_value = encoder->current_CNT_value;
			}
		}
		else	//CNT dem lên
		{
			if ((encoder->current_CNT_value - encoder->last_CNT_value) >= 4)
			{
				encoder->pulse++;
				encoder->last_CNT_value = encoder->current_CNT_value;
			}
		}
	}
}




void Encoder_Pulse_Calibration(Encoder_HandleTypeDef *encoder) //Hiêu chinh sai sô sô xung 
{
	Encoder_Round_Counter(encoder); //Ðêm so vong qua xung Z
	encoder->pulse = encoder->round_counter * 1000 + encoder->offset;
}

void Encoder_Position(Encoder_HandleTypeDef *encoder, TIM_HandleTypeDef *htim, uint16_t resolution, float wheel_diameater) //Tính vi trí
{
	Encoder_Update_Data(encoder, htim, resolution);
	Encoder_Pulse_Counter(encoder);
	encoder->position = PI*wheel_diameater*encoder->pulse/encoder->resolution; //Tinh quang duong

}

void Encoder_Reset(Encoder_HandleTypeDef *encoder, TIM_HandleTypeDef *htim)//Reset Encoder + CNT
{
	htim->Instance->CNT = 0;
	encoder->current_CNT_value = encoder->current_direction = encoder->last_CNT_value
	= encoder->last_direction = encoder->offset = encoder->position = encoder->pulse
	= encoder->resolution = encoder->round_counter = 0;
	offset_flag = 0;
}	


