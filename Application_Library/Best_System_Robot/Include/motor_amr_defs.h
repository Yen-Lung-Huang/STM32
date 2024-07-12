#ifndef MOTOR_AMR_DEFS_H
#define MOTOR_AMR_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOVE,
    TURN,
    STRAFE,
    DIAGONAL
} MovementMode;

typedef struct {
    int static_friction_threshold;
    int low_speed_threshold;
    int success_count;
    int failure_count;
    float kp;
    float ki;
    float kd;
    float k_back_emf;  // Added this from your original code
} MotorThresholds;

typedef struct {
    int target_speed;
    int current_speed;
    bool static_friction_overcome;
    float integral_error;
    float previous_error;
    uint32_t last_update_time;
    MotorThresholds thresholds;
    MovementMode mode;
} MotorController;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_AMR_DEFS_H */