/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Include */
#include <stdbool.h>
#include <stdint.h>


// Define a structure for Motor Thresholds and PID parameters
typedef struct {
    int static_friction_threshold;
    int low_speed_threshold;
    int success_count;
    int failure_count;
    float kp;
    float ki;
    float kd;
} MotorThresholds;

// Define a structure for Motor Controller
typedef struct {
    int target_speed;              // Range -999 to 999, negative value means reverse
    int current_speed;             // Current speed of the motor
    bool static_friction_overcome; // Flag indicating if static friction has been overcome
    float integral_error;          // Integral error for PID control
    float previous_error;          // Previous error for PID control
    uint32_t last_update_time;     // Time of the last update for dt calculation
    MotorThresholds thresholds;    // Motor thresholds and PID parameters
} MotorController;

/* FUNCTION (Prototype) DEFINITIONS */
//e.g. void hello_world(void)...


#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROLLER_H */

/*****END OF FILE*****/
