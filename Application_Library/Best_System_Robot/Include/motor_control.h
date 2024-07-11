#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_types.h"
#include "motor_shield.h"

extern const MotorThresholds WHEEL_THRESHOLDS[];

// Function prototypes
void adjust_thresholds(DC_Motor_TypeDef *motor, bool success);
void soft_motor_control(void *motor_shield, Motor_Shield_Type type, uint8_t dc_motor_number, int target_speed);
void adjust_motor_parameters(DC_Motor_TypeDef *motor, MovementMode mode);
void set_wheel_modes(MovementMode mode);
void set_wheel_speeds(int m1, int m2, int m3, int m4);
void set_wheel_speeds_with_mode(int m1, int m2, int m3, int m4, MovementMode mode);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */