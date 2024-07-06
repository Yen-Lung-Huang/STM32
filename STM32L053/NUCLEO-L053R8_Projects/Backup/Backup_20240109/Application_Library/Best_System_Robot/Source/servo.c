#include "servo.h"
//extern PWM_TypeDef servo[];

/* Math--------------------------------------------------------*/
float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* Servo Configuration--------------------------------------------------------*/
PWM_TypeDef pwm_constructor(PWM_TypeDef pwm_struct)
{
#if defined(__STM32L0xx_HAL_H)
	HAL_TIM_PWM_Start(pwm_struct.timer,pwm_struct.channel);
#elif defined(__STM32F4xx_HAL_H)
	pwm_struct.complementary?
	HAL_TIMEx_PWMN_Start(pwm_struct.timer,pwm_struct.channel):
	HAL_TIM_PWM_Start(pwm_struct.timer,pwm_struct.channel);
#endif
	return pwm_struct;
}


//Robot arm servos initialization.
PWM_TypeDef servo[3];
void servos_init(void)
{
	servo[S1] = pwm_constructor((PWM_TypeDef){.timer=&htim21, .channel=TIM_CHANNEL_1, .pwm_min=26,\
																	.pwm_max=123, .physical_min=0, .physical_max=180, .offset=0,.pwm_value=0,\
																	.reverse=false, .complementary=false, .latch=false});
	servo[S2] = pwm_constructor((PWM_TypeDef){.timer=&htim21, .channel=TIM_CHANNEL_2, .pwm_min=26,\
																	.pwm_max=123, .physical_min=0, .physical_max=180, .offset=0,.pwm_value=0,\
																	.reverse=false, .complementary=false, .latch=false});
	servo[S3] = pwm_constructor((PWM_TypeDef){.timer=&htim2, .channel=TIM_CHANNEL_4, .pwm_min=0,\
																	.pwm_max=123, .physical_min=0, .physical_max=180, .offset=0,.pwm_value=0,\
																	.reverse=false, .complementary=false, .latch=false});
}


volatile uint32_t* timer_ch2ccr(TIM_HandleTypeDef* timer, uint32_t channel)
{
	switch(channel){
		case TIM_CHANNEL_1:
			return &(timer->Instance->CCR1);
		case TIM_CHANNEL_2:
			return &(timer->Instance->CCR2);
		case TIM_CHANNEL_3:
			return &(timer->Instance->CCR3);
		case TIM_CHANNEL_4:
			return &(timer->Instance->CCR4);
	}
	return &(timer->Instance->CCR1); // A random return value for default, not nesseary.
}


/* Servo Control--------------------------------------------------------*/
uint16_t reverse_pwm(PWM_TypeDef* servo, uint16_t pwm_value)
{
	return servo->pwm_min + servo->pwm_max - pwm_value;
}


float reverse_physical(PWM_TypeDef* servo, float physical_value)
{
	return servo->physical_min + servo->physical_max - physical_value;
}



// 定義一個宏，表示 PWM 的最小占空比
#define PWM_DUTY_CYCLE_MIN 20

void pwm_set(PWM_TypeDef* servo, uint16_t pwm_value, bool limit) // 使用固定的兩個參數來定義函數
{
	// 儲存不受限制的 PWM 值
	servo->pwm_value = pwm_value; 
	
	// 判斷 limit 是否為 true
	if(limit == true){ // 如果 limit 為 true，執行以下代碼
		// 取得實際的 PWM 值
		pwm_value = get_limit_pwm(servo); 
		
		// 限制 pwm_value 最小值為 PWM_DUTY_CYCLE_MIN
		if(pwm_value < PWM_DUTY_CYCLE_MIN){
			pwm_value = PWM_DUTY_CYCLE_MIN;
		}
	}
	
	// 設定 PWM 輸出
	*(timer_ch2ccr(servo->timer,servo->channel)) = pwm_value;
	// printf("limit= %d, pwm_value= %d\r\n",limit,pwm_value); // Print for debug.

	// 如果 latch 為 true，控制繼電器開關
	if(servo->latch == true){
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); // for Relay
	}
}


// Define a function to set the PWM value according to the physical angle and the limit with fixed arguments
void pwm_physical_set(PWM_TypeDef *servo, float physical_value, bool limit)
{
  // Check the validity of the arguments
  if (servo == NULL) return; // Invalid pointer
  
  // Check the limit parameter
  if (limit) {
    // Limit the physical value to the physical range
    if (physical_value < servo->physical_min) physical_value = servo->physical_min;
    if (physical_value > servo->physical_max) physical_value = servo->physical_max;
  }

  // Calculate the PWM value for the physical value
  uint16_t pwm_value = map(physical_value, servo->physical_min, servo->physical_max, servo->pwm_min, servo->pwm_max);
  pwm_set(servo, pwm_value, limit);

  // Check if the servo is latched and the PWM value is zero
  if (servo->latch && servo->pwm_value == 0) {
    // Reset the GPIO pin for the relay
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); // for Relay
  }
}


// Define a function to control the servo angle with fixed arguments
void servo_control(PWM_TypeDef *servo, float servo_input, bool mode, bool limit) 
{
  // Check the validity of the arguments
  if (servo == NULL) return; // Invalid pointer

  // Check the mode parameter
  if (mode == PWM) { // If the mode is PWM
    // Set the PWM value
    pwm_set(servo, (int)servo_input, limit); 
  } else if (mode == ANGLE) { // If the mode is ANGLE
    // Set the PWM value according to the servo angle
    pwm_physical_set(servo, servo_input, limit); 
  }
}


void pwm_stop(PWM_TypeDef* servo)
{
	pwm_set(servo, 0, false);
}

void all_pwm_stop(void)
{
	for(int i=0; i<=11; i++){
		//servo_pwm_set(&servo[i], 0, false);
	}
}


/* Servo Motor Returns Infomation--------------------------------------------------------*/

float get_pwm_physical(PWM_TypeDef* servo)
{
	return map(servo->pwm_value,servo->pwm_min,servo->pwm_max,servo->physical_min,servo->physical_max);
}


// 這個函數接收一個 servo 物件的指標，並回傳它的實際使用的 PWM 占空比
uint16_t get_limit_pwm(PWM_TypeDef* servo)
{
    uint16_t limit_pwm = servo->pwm_value; // 先將 pwm_value 複製給 limit_pwm
    if (servo->reverse == true) // 如果 reverse 為 true，則進行反轉的運算
    {
        limit_pwm = reverse_pwm(servo,limit_pwm);
    }
    if (servo->offset != 0) // 如果 offset 不為零，則加上 offset
    {
        limit_pwm = limit_pwm + servo->offset;
    }
    if(limit_pwm < servo->pwm_min){ // 如果 limit_pwm 小於 pwm_min，則設為 pwm_min
        limit_pwm = servo->pwm_min;
    }
    if(limit_pwm > servo->pwm_max){ // 如果 limit_pwm 大於 pwm_max，則設為 pwm_max
        limit_pwm = servo->pwm_max;
    }
    return limit_pwm; // 回傳 limit_pwm
}

