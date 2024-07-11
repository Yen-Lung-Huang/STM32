#include "move.h"

// Define default parameters for each mode
const MotorThresholds WHEEL_THRESHOLDS[] = {
    [MOVE] = {
        .static_friction_threshold = 500,
        .low_speed_threshold = 375,
        .success_count = 0,
        .failure_count = 0,
        .kp = 1.0f,
        .ki = 0.1f,
        .kd = 0.01f
    },
    [TURN] = {
        .static_friction_threshold = 600,
        .low_speed_threshold = 450,
        .success_count = 0,
        .failure_count = 0,
        .kp = 1.2f,
        .ki = 0.12f,
        .kd = 0.015f
    },
    [STRAFE] = {
        .static_friction_threshold = 600,
        .low_speed_threshold = 450,
        .success_count = 0,
        .failure_count = 0,
        .kp = 1.2f,
        .ki = 0.12f,
        .kd = 0.015f
    },
    [DIAGONAL] = {
        .static_friction_threshold = 600,
        .low_speed_threshold = 450,
        .success_count = 0,
        .failure_count = 0,
        .kp = 1.2f,
        .ki = 0.12f,
        .kd = 0.015f
    }
};

void adjust_motor_parameters(DC_Motor_TypeDef *motor, MovementMode mode)
{
    motor->controller.thresholds = WHEEL_THRESHOLDS[mode];
}

void set_wheel_modes(MovementMode mode)
{
    // Adjust parameters for each motor according to the mode
    adjust_motor_parameters(&motor_shield_l29xx.M1, mode);
    adjust_motor_parameters(&motor_shield_l29xx.M2, mode);
    adjust_motor_parameters(&motor_shield_l29xx.M3, mode);
    adjust_motor_parameters(&motor_shield_l29xx.M4, mode);
}

void set_wheel_speeds(int m1, int m2, int m3, int m4) // target speeds
{
    // set motor speeds
    set_motor_speed(&motor_shield_l29xx, MS_L29XX, M1, m1);
    set_motor_speed(&motor_shield_l29xx, MS_L29XX, M2, m2);
    set_motor_speed(&motor_shield_l29xx, MS_L29XX, M3, m3);
    set_motor_speed(&motor_shield_l29xx, MS_L29XX, M4, m4);

    // may need to call other functions to actually execute motor control
    // e.g. ms_motor_control(&motor_shield_l29xx, MS_L29XX, M1, m1);
    // ame as M2, M3, M4
}

void set_wheel_speeds_with_mode(int m1, int m2, int m3, int m4, MovementMode mode)
{
    set_wheel_modes(mode); // Set the motor mode and adjust motor parameters
    set_wheel_speeds(m1, m2, m3, m4); // Set wheel speeds
}