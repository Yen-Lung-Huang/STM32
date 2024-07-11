/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MOTOR_SHIELD_H
#define MOTOR_SHIELD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Include */
#include <math.h>
#include "servo.h"
#include "74HC595.h"
#include "timing_delays.h"
#include "motor_types.h"

/* MACRO DEFINITIONS */
#define DC_MOTOR_MIN 0
#define DC_MOTOR_MAX 999
#define MAX_ACCELERATION 1000
#define MAX_ADJUSTMENT_FACTOR 0.8f
#define MIN_ADJUSTMENT_FACTOR 0.3f
#define STOP_ACCELERATION_FACTOR 1.5f  // Increase deceleration when stopping
#define USE_BACK_EMF_COMPENSATION 0    // Set to 1 to enable back EMF compensation


// Define a structure for EN pin
typedef struct {
	My_GPIO_TypeDef GPIO; // GPIO for EN pin
	PWM_TypeDef PWM;      // PWM for EN pin
	bool enable_pwm;      // Enable PWM mode or not
    NonBlockingDelay_TypeDef soft_control_delay; // Non-blocking delay for soft control
} DC_Motor_EN_TypeDef;

// Define a structure for Encoder
typedef struct {
    My_GPIO_TypeDef A;  // GPIO for Encoder A pin
    My_GPIO_TypeDef B;  // GPIO for Encoder B pin
    int32_t count;      // Encoder count
} Encoder_TypeDef;

// Define a structure for DC Motor
typedef struct {
    My_GPIO_TypeDef IN1;           // GPIO for IN1 pin
    My_GPIO_TypeDef IN2;           // GPIO for IN2 pin
    DC_Motor_EN_TypeDef EN;        // Enable signal for DC motor
    MotorController controller;    // Motor controller
    bool reverse;                  // Reverse the motor direction
    Encoder_TypeDef encoder;       // Encoder structure
    bool has_encoder;              // Flag indicating if encoder exists
} DC_Motor_TypeDef;

// Define a structure for Motor_Shield_V1
typedef struct {
	HC595 hc595;
	DC_Motor_TypeDef M1; // DC motor 1
	DC_Motor_TypeDef M2; // DC motor 2
	DC_Motor_TypeDef M3; // DC motor 3
	DC_Motor_TypeDef M4; // DC motor 4
	PWM_TypeDef S1_PWM;  // PWM signal for servo 1
	PWM_TypeDef S2_PWM;  // PWM signal for servo 2
} Motor_Shield_V1;

// Define a structure for Motor_Shield_l29xx
typedef struct {
	DC_Motor_TypeDef M1; // DC motor 1
	DC_Motor_TypeDef M2; // DC motor 2
	DC_Motor_TypeDef M3; // DC motor 3
	DC_Motor_TypeDef M4; // DC motor 4
} Motor_Shield_L29XX;

// Define a structure for L293D、l298P、L298N
typedef struct {
	DC_Motor_TypeDef M1; // DC motor 1
	DC_Motor_TypeDef M2; // DC motor 2
} L29XX;


// Declare global objects
extern Motor_Shield_V1 motor_shield_v1;
extern Motor_Shield_L29XX motor_shield_l29xx;

// // Define an enum type to indicate the type of motor shield
// enum Motor_Shield_Type {MS_V1, MS_L29XX};

// // Define an enum type variable to store the motor names and values
// enum Motor_Shield_Motor {M1, M2, M3, M4};

// // Define an enum type variable to store the servo names and values
// // enum motor_shield_v1_servo {S1, S2};


// Function prototypes
void ms_init(Motor_Shield_Type type, bool enable_pwm);
void init_motor_shield_v1();
void init_motor_shield_l29xx();
void ms_pwm_init(Motor_Shield_Type type);
void ms_gpio_init(Motor_Shield_Type type);
void ms_encoder_init(Motor_Shield_Type type);
DC_Motor_TypeDef *get_dc_motor(void *motor_shield, Motor_Shield_Type type, uint8_t dc_motor_number);
uint8_t get_motor_bit(uint8_t dc_motor_number, uint8_t bit_index);
void set_motor_speed(void *motor_shield, Motor_Shield_Type type, uint8_t dc_motor_number, int target_speed);
void ms_motor_control(void *motor_shield, Motor_Shield_Type type, uint8_t dc_motor_number, float motor_input);
void ms_v1_servo_control(Motor_Shield_V1 *motor_shield, uint8_t servo_number, float servo_input, bool mode);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_SHIELD_H */

/*****END OF FILE*****/
