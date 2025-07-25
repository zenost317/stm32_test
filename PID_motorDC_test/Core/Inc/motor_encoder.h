/*
 * motor_encoder.h
 *
 *  Created on: Jul 14, 2025
 *      Author: Acer
 */

#ifndef INC_MOTOR_ENCODER_H_
#define INC_MOTOR_ENCODER_H_

#include "stdint.h"
#include "main.h"

typedef struct _encoder{
	int16_t velocity;
	int16_t position;
	uint32_t prev_encoder_raw;
}encoderConfig;

void update_encoder(TIM_HandleTypeDef *htim);
void reset_encoder();

#endif /* INC_MOTOR_ENCODER_H_ */
