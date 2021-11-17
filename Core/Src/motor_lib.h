#define FORWARD "Forward"    // forward string
#define BACKWARD "Backward"    // backward string
#define MAX_C_LPU 0.5


int sign(int x) {
  if (x>0) return 1;
  else if (x<0) return -1;
  else return 0;
}

struct motor_wheel
{
	uint8_t id;
	uint8_t dir_q;
	uint8_t break_q;
	uint8_t inverse_q;
	float duty_cycle; // Duty cycle [-1.0, 1.0]
};

typedef struct motor_wheel Motor_Wheel ;

void motor_wheel_init(Motor_Wheel *str, uint8_t motor_num)
{
	str->id = motor_num;
	str->inverse_q = 0;
	str->break_q = 0;
	str->dir_q = 0;
	str->duty_cycle = 0.0;
	// BREAK PIN
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); // Set HIGH
	// DIRECTION PIN
	if (str->id == 1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // Set LOW
	}
	else if (str->id == 2)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); // Set LOW
		str->inverse_q = 1;
	}
	else if (str->id == 3)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // Set LOW
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); // Set HIGH
		str->inverse_q = 1;
	}
}


void motorPWM_pulse(TIM_HandleTypeDef* timer_handle, Motor_Wheel *str, int LPU) // PWM Velocity Control
{

	uint8_t throttle = abs(LPU);//fabs(LPU) * 100;
	if (abs(LPU) > 100)
	{
		return;
	}

	if (str->id == 1)
		__HAL_TIM_SET_COMPARE(timer_handle, TIM_CHANNEL_1, throttle);
	else if (str->id == 2)
		__HAL_TIM_SET_COMPARE(timer_handle, TIM_CHANNEL_2, throttle);
	else if (str->id == 3)
		__HAL_TIM_SET_COMPARE(timer_handle, TIM_CHANNEL_3, throttle);
	else if (str->id == 4)
		__HAL_TIM_SET_COMPARE(timer_handle, TIM_CHANNEL_4, throttle);
};

void set_duty_cycle(Motor_Wheel *str, float value)
{
  str->duty_cycle = value;
  if (fabs(value) > MAX_C_LPU)
	  str->duty_cycle = MAX_C_LPU*sign(value);
}

float get_duty_cycle(Motor_Wheel *str)
{
	return str->duty_cycle;
}

void motorPWM_default(TIM_HandleTypeDef* timer_handle, Motor_Wheel *str) // Send set duty cycle tothe motor
{
  if (str->duty_cycle >= 0)
	motor_DIR(str, FORWARD);
  else
	motor_DIR(str, BACKWARD);
  uint8_t throttle = fabs(str->duty_cycle) * 100;
  if (str->id == 1)
	  __HAL_TIM_SET_COMPARE(timer_handle, TIM_CHANNEL_1, throttle);
  else if (str->id == 2)
	  __HAL_TIM_SET_COMPARE(timer_handle, TIM_CHANNEL_2, throttle);
  else if (str->id == 3)
	  __HAL_TIM_SET_COMPARE(timer_handle, TIM_CHANNEL_3, throttle);
  else if (str->id == 4)
	  __HAL_TIM_SET_COMPARE(timer_handle, TIM_CHANNEL_4, throttle);
};

void motor_DIR(Motor_Wheel *str, const uint8_t Direction) // Send direction change command
{

	str->dir_q = Direction;
	// 0 = Forward
	// 1 = backward


  // DIRECTION PIN
  if (str->dir_q == 1) // Backward
  {
	 if (str->id == 1 || str->id == 3) // LEFT MOTORS
	  	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // Set LOW
	 else // RIGHT MOTORS
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET); // Set HIGH
  }
  else // Forward
  {
	 if (str->id == 1 || str->id == 3) // LEFT MOTORS
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // Set HIGH
	 else // RIGHT MOTORS
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); // Set LOW
  }
}

void motor_Reverse(Motor_Wheel *str) // Send direction change command
{
	str->inverse_q = abs(str->inverse_q - 1);
	//if (abs(str->dir_q - str->inverse_q) == 1)
}

void motor_break(Motor_Wheel *str, uint8_t br) // Send break signal
{
  str->break_q = br;
  // BREAK PIN
  if (str->break_q == 1)
  {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET); // Set HIGH
	  str->duty_cycle = 0.0;
  }
  else
  {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); // Set LOW
  }
}
