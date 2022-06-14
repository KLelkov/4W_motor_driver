#define POSITION_MAX 10250
#define POSITION_MIN -10250


struct linear_motor
{
	uint8_t id;
	uint8_t dir_q;
	uint8_t inverse_q;
	int32_t current_position;
	int32_t target_position;
};

typedef struct linear_motor Linear_Motor ;

void linear_motor_init(Linear_Motor *str, uint8_t motor_num, uint8_t inverse)
{
	str->id = motor_num;
	str->inverse_q = inverse;
	str->current_position = 0;
	str->target_position = 0;
	str->dir_q = 0;
}


void linear_motor_set_target(Linear_Motor *str, int32_t target)
{
	if (target > POSITION_MAX)
	{
		target = POSITION_MAX;
	}
	else if (target < POSITION_MIN)
	{
		target = POSITION_MIN;
	}
	str->target_position = target;
}


int32_t linear_motor_get_position(Linear_Motor *str)
{
	return str->current_position;
}


void linear_motor_set_direction(Linear_Motor *str, uint8_t dir)
{
	str->dir_q = dir;
}

/**
  * @brief  Sends a fixed number of PWM pulses to the target linear motor
  * @param  str linear_motor structure handle
  * @param  timer_handle timer handle associated with linear motors
  * @param  counter_handle counter pointer to track number of pulses sent via interupt callback
  * @retval if(errors_occurred)
  */
uint8_t linear_motor_pulse(Linear_Motor *str, TIM_HandleTypeDef* timer_handle, uint32_t* counter_handle)
{
	if (str->id == 1 && TIM_CHANNEL_STATE_GET(timer_handle, TIM_CHANNEL_1) == HAL_TIM_CHANNEL_STATE_BUSY)
	{
		return 1; // error occurred
	}
	else if (str->id == 2 && TIM_CHANNEL_STATE_GET(timer_handle, TIM_CHANNEL_2) == HAL_TIM_CHANNEL_STATE_BUSY)
	{
		return 1; // error occurred
	}
	uint32_t shift = abs(str->target_position - str->current_position);
	if (shift == 0)
	{
		return 1;
	}
	if (str->target_position - str->current_position > 0)
	{
		str->dir_q = 1;
	}
	else
	{
		str->dir_q = 0;
	}
	if (str->id == 1)
	{
		*counter_handle = shift;
		// TODO: SET DIRECTION GPIO
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); // Set LOW
		if (abs(str->dir_q - str->inverse_q) == 1)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET); // Set HIGH
		}
		else
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); // Set LOW
		}
		HAL_TIM_PWM_Start_IT(timer_handle, TIM_CHANNEL_1);
	}
	else if (str->id == 2)
	{
		*counter_handle = shift;
		if (abs(str->dir_q - str->inverse_q) == 1)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); // Set HIGH
		}
		else
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); // Set LOW
		}
		HAL_TIM_PWM_Start_IT(timer_handle, TIM_CHANNEL_2);
	}
	str->current_position = str->target_position;
	return 0;
}


uint32_t linear_motor_calibrate(Linear_Motor *str, TIM_HandleTypeDef* timer_handle, uint32_t* counter_handle)
{
	uint32_t stepSize = 100;
	uint32_t distanceTraveled = 0;
	uint8_t edgeReached1 = 0;
	uint8_t edgeReached2 = 0;
	uint32_t hard_limit = 20000;
	uint32_t cnt = 0;
	if (str->id == 1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); // Set HIGH
	}
	else if (str->id == 2)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); // Set HIGH
	}
	while (edgeReached1 == 0)
	{
		*counter_handle = stepSize;
		cnt++;
		if (str->id == 1)
		{
			while((TIM_CHANNEL_STATE_GET(timer_handle, TIM_CHANNEL_1) == HAL_TIM_CHANNEL_STATE_BUSY))
			{

			}
			HAL_TIM_PWM_Start_IT(timer_handle, TIM_CHANNEL_1);
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 0)
			{
				edgeReached1 = 1;
			}
			else
			{
				distanceTraveled += stepSize;
			}
			// TODO: add timeout
		}
//		else if (str->id == 2)
//		{
//			while((TIM_CHANNEL_STATE_GET(timer_handle, TIM_CHANNEL_2) == HAL_TIM_CHANNEL_STATE_BUSY))
//			{
//
//			}
//			HAL_TIM_PWM_Start_IT(timer_handle, TIM_CHANNEL_2);
//			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0)
//			{
//				edgeReached1 = 1;
//			}
//			else
//			{
//				distanceTraveled += stepSize;
//			}
//			// TODO: add timeout
//		}
		if (distanceTraveled > hard_limit)
		{
			edgeReached1 = 1;
			return 0;
		}

	}
	if (str->id == 1)
	{
		// Change direction of the front motor
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET); // Set LOW
		*counter_handle = 9600;
		HAL_TIM_PWM_Start_IT(timer_handle, TIM_CHANNEL_1);
		str->current_position = 0;
		return distanceTraveled;
	}
//	else if (str->id == 2)
//	{
//		// Change direction of the rear motor
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); // Set LOW
//		*counter_handle = 9600;
//		HAL_TIM_PWM_Start_IT(timer_handle, TIM_CHANNEL_2);
//		str->current_position = 0;
//		return distanceTraveled;
//	}
	return 0;

}

