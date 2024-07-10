#include "move.h"

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

void adjust_motor_parameters(DC_Motor_TypeDef *motor, MovementMode mode)
{
    switch (mode) {
    case MOVE_FORWARD:
        motor->thresholds.static_friction_threshold = 500;
        motor->thresholds.low_speed_threshold = 375;
        motor->thresholds.kp = 1.0f;
        motor->thresholds.ki = 0.1f;
        motor->thresholds.kd = 0.01f;
        break;
    case TURN:
        motor->thresholds.static_friction_threshold = 600;
        motor->thresholds.low_speed_threshold = 450;
        motor->thresholds.kp = 1.2f;
        motor->thresholds.ki = 0.12f;
        motor->thresholds.kd = 0.015f;
        break;
    case STRAFE:
        motor->thresholds.static_friction_threshold = 600;
        motor->thresholds.low_speed_threshold = 450;
        motor->thresholds.kp = 1.2f;
        motor->thresholds.ki = 0.12f;
        motor->thresholds.kd = 0.015f;
        break;
    case DIAGONAL:
        motor->thresholds.static_friction_threshold = 600;
        motor->thresholds.low_speed_threshold = 450;
        motor->thresholds.kp = 1.2f;
        motor->thresholds.ki = 0.12f;
        motor->thresholds.kd = 0.015f;
        break;
    }
}