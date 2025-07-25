/*
 * motor_encoder.c
 *
 *  Created on: Jul 14, 2025
 *      Author: Acer
 */

#include "motor_encoder.h"

encoderConfig encoder = {
		.velocity = 0,
		.position = 0,
		.prev_encoder_raw = 0,
};

void update_encoder(TIM_HandleTypeDef *htim)
{
	uint32_t temp_counter = __HAL_TIM_GET_COUNTER(htim);
	static uint8_t first_time = 0;

	if(!first_time)
	{
		encoder.velocity = 0;
		first_time = 1;
	}
	else
	{
		if(temp_counter == encoder.prev_encoder_raw)
		{
			encoder.velocity = 0;
		}
		else if(temp_counter > encoder.prev_encoder_raw)
		{
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			{
				encoder.velocity = -encoder.prev_encoder_raw - (__HAL_TIM_GET_AUTORELOAD(htim) - temp_counter);
			}
			else
			{
				encoder.velocity = temp_counter - encoder.prev_encoder_raw;
			}
		}
		else
		{
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			{
				encoder.velocity = temp_counter - encoder.prev_encoder_raw;
			}
			else
			{
				encoder.velocity = temp_counter + (__HAL_TIM_GET_AUTORELOAD(htim) - encoder.prev_encoder_raw);
			}
		}
	}

	encoder.position += encoder.velocity;
	encoder.prev_encoder_raw = temp_counter;

}

void reset_encoder()
{
	encoder.velocity = 0;
	encoder.position = 0;
	encoder.prev_encoder_raw = 0;
}
