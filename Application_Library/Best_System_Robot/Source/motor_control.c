#include "motor_control.h"

#define MAX_ACCELERATION 1000
#define MAX_ADJUSTMENT_FACTOR 0.8f
#define MIN_ADJUSTMENT_FACTOR 0.3f
#define STOP_ACCELERATION_FACTOR 1.5f  // Increase deceleration when stopping
#define USE_BACK_EMF_COMPENSATION 0    // Set to 1 to enable back EMF compensation

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
        .low_speed_threshold = 600,
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


/* Soft Motor Control ---------------------------------------------------------------*/

// Define a function to adjust the motor thresholds
void adjust_thresholds(DC_Motor_TypeDef *motor, bool success)
{
    MotorThresholds *motor_thresholds = &motor->controller.thresholds;
    const MotorThresholds *initial_thresholds = &WHEEL_THRESHOLDS[motor->controller.mode];

    if (success) {
        motor_thresholds->success_count++;
        motor_thresholds->failure_count = 0;
        if (motor_thresholds->success_count > 5) {
            motor_thresholds->static_friction_threshold -= 10;
            motor_thresholds->low_speed_threshold -= 5;
            motor_thresholds->success_count = 0;
        }
    } else {
        motor_thresholds->failure_count++;
        motor_thresholds->success_count = 0;
        if (motor_thresholds->failure_count > 3) {
            motor_thresholds->static_friction_threshold += 10;
            motor_thresholds->low_speed_threshold += 5;
            motor_thresholds->failure_count = 0;
        }
    }

    // Ensure thresholds do not go below initial values
    motor_thresholds->static_friction_threshold = (motor_thresholds->static_friction_threshold < initial_thresholds->static_friction_threshold) ? initial_thresholds->static_friction_threshold : motor_thresholds->static_friction_threshold;
    motor_thresholds->low_speed_threshold = (motor_thresholds->low_speed_threshold < initial_thresholds->low_speed_threshold) ? initial_thresholds->low_speed_threshold : motor_thresholds->low_speed_threshold;
}


// Define a function to control the motor with one input
void soft_motor_control(void *motor_shield, Motor_Shield_Type type, uint8_t dc_motor_number, int target_speed)
{
    DC_Motor_TypeDef *motor = get_dc_motor(motor_shield, type, dc_motor_number);
    if (motor == NULL) {
        printf("Error: Motor not found\n");
        return;
    }

    MotorController *motor_ctrl = &motor->controller;
    MotorThresholds *motor_thresholds = &motor_ctrl->thresholds;
    target_speed = (target_speed > DC_MOTOR_MAX) ? DC_MOTOR_MAX : (target_speed < -DC_MOTOR_MAX ? -DC_MOTOR_MAX : target_speed);
    motor_ctrl->target_speed = target_speed;

    uint32_t current_time = HAL_GetTick();

    if (motor->EN.soft_control_delay.IsExpired(&motor->EN.soft_control_delay)) {
        float dt = (current_time - motor_ctrl->last_update_time) / 1000.0f;
        int speed_diff = motor_ctrl->target_speed - motor_ctrl->current_speed;

        // PID control
        motor_ctrl->integral_error += speed_diff * dt;
        float derivative = (speed_diff - motor_ctrl->previous_error) / dt;
        float pid_output = motor_thresholds->kp * speed_diff + motor_thresholds->ki * motor_ctrl->integral_error + motor_thresholds->kd * derivative;

        if (target_speed == 0) {
            // Faster stopping logic
            float stop_deceleration = MAX_ACCELERATION * STOP_ACCELERATION_FACTOR * dt;
            if (abs(motor_ctrl->current_speed) > motor_thresholds->low_speed_threshold) {
                motor_ctrl->current_speed -= (motor_ctrl->current_speed > 0) ? stop_deceleration : -stop_deceleration;
                if (abs(motor_ctrl->current_speed) < motor_thresholds->low_speed_threshold) {
                    motor_ctrl->current_speed = 0;
                }
            } else {
                motor_ctrl->current_speed = 0;
            }
            motor_ctrl->static_friction_overcome = false;
        } else {
            int effective_target_speed = (abs(target_speed) < motor_thresholds->low_speed_threshold) ?
                                         (target_speed > 0 ? motor_thresholds->low_speed_threshold : -motor_thresholds->low_speed_threshold) : target_speed;

            if (motor_ctrl->current_speed * effective_target_speed < 0) {
                // Handle direction change
                float direction_change_deceleration = MAX_ACCELERATION * dt;
                motor_ctrl->current_speed -= (motor_ctrl->current_speed > 0) ? direction_change_deceleration : -direction_change_deceleration;
                if (abs(motor_ctrl->current_speed) < motor_thresholds->low_speed_threshold) {
                    motor_ctrl->current_speed = 0;
                    motor_ctrl->static_friction_overcome = false;
                }
            } else {
                if (!motor_ctrl->static_friction_overcome || abs(motor_ctrl->current_speed) < motor_thresholds->low_speed_threshold) {
                    motor_ctrl->current_speed = (effective_target_speed > 0) ? motor_thresholds->static_friction_threshold : -motor_thresholds->static_friction_threshold;
                    motor_ctrl->static_friction_overcome = true;
                } else {
                    float speed_ratio = (float)abs(motor_ctrl->current_speed) / DC_MOTOR_MAX;
                    float adjustment_factor = MAX_ADJUSTMENT_FACTOR - speed_ratio * (MAX_ADJUSTMENT_FACTOR - MIN_ADJUSTMENT_FACTOR);

                    adjustment_factor *= (abs(effective_target_speed) > abs(motor_ctrl->current_speed)) ? 1.2f : 0.8f;

                    int speed_change = (int)((pid_output + effective_target_speed - motor_ctrl->current_speed) * adjustment_factor);
                    int max_speed_change = MAX_ACCELERATION * dt;

                    speed_change = (abs(speed_change) > max_speed_change) ?
                                   ((speed_change > 0) ? max_speed_change : -max_speed_change) : speed_change;

                    motor_ctrl->current_speed += speed_change;

                    if ((speed_diff > 0 && motor_ctrl->current_speed > effective_target_speed) ||
                            (speed_diff < 0 && motor_ctrl->current_speed < effective_target_speed)) {
                        motor_ctrl->current_speed = effective_target_speed;
                    }
                }
            }
        }

        float motor_input = (float)motor_ctrl->current_speed / DC_MOTOR_MAX;

#if USE_BACK_EMF_COMPENSATION
        float back_emf = motor_thresholds->k_back_emf * motor->current_speed;
        motor_input += back_emf;
#endif

        motor_input = (motor_input > 1.0f) ? 1.0f : (motor_input < -1.0f ? -1.0f : motor_input);

        ms_motor_control(motor_shield, type, dc_motor_number, motor_input);

        motor_ctrl->previous_error = speed_diff;
        motor_ctrl->last_update_time = current_time;
        motor->EN.soft_control_delay.Start(&motor->EN.soft_control_delay, 1);

        // Update performance (this is where we would ideally use encoder feedback)
        bool moved_as_expected = abs(motor_ctrl->current_speed - target_speed) < motor_thresholds->low_speed_threshold;
        adjust_thresholds(motor, moved_as_expected);
    }
}


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