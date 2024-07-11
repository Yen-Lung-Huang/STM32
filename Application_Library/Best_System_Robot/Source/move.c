#include "move.h"



void adjust_motor_parameters(DC_Motor_TypeDef *motor, MovementMode mode)
{
    MotorThresholds *motor_thresholds = &motor->controller.thresholds;
    
    switch (mode) {
    case MOVE:
        motor_thresholds->static_friction_threshold = 500;
        motor_thresholds->low_speed_threshold = 375;
        motor_thresholds->kp = 1.0f;
        motor_thresholds->ki = 0.1f;
        motor_thresholds->kd = 0.01f;
        break;
    case TURN:
        motor_thresholds->static_friction_threshold = 600;
        motor_thresholds->low_speed_threshold = 450;
        motor_thresholds->kp = 1.2f;
        motor_thresholds->ki = 0.12f;
        motor_thresholds->kd = 0.015f;
        break;
    case STRAFE:
        motor_thresholds->static_friction_threshold = 600;
        motor_thresholds->low_speed_threshold = 450;
        motor_thresholds->kp = 1.2f;
        motor_thresholds->ki = 0.12f;
        motor_thresholds->kd = 0.015f;
        break;
    case DIAGONAL:
        motor_thresholds->static_friction_threshold = 600;
        motor_thresholds->low_speed_threshold = 450;
        motor_thresholds->kp = 1.2f;
        motor_thresholds->ki = 0.12f;
        motor_thresholds->kd = 0.015f;
        break;
    }
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