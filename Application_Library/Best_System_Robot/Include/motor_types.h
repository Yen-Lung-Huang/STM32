#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

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

typedef enum {
    MS_V1,
    MS_L29XX
} Motor_Shield_Type;

typedef enum {
    M1,
    M2,
    M3,
    M4
} Motor_Shield_Motor;

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

// // Forward declarations
// typedef struct My_GPIO_TypeDef My_GPIO_TypeDef;
// typedef struct PWM_TypeDef PWM_TypeDef;
// typedef struct DC_Motor_TypeDef DC_Motor_TypeDef;
// typedef struct Motor_Shield_V1 Motor_Shield_V1;
// typedef struct Motor_Shield_L29XX Motor_Shield_L29XX;
// typedef struct NonBlockingDelay_TypeDef NonBlockingDelay_TypeDef;
// typedef struct HC595 HC595;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_TYPES_H */