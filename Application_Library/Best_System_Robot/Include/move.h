/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MOVE_H
#define MOVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_shield.h"

typedef enum {
    MOVE,
    TURN,
    STRAFE,
    DIAGONAL
} MovementMode;

extern const MotorThresholds WHEEL_THRESHOLDS[];

void adjust_motor_parameters(DC_Motor_TypeDef *motor, MovementMode mode);
void set_wheel_modes(MovementMode mode);
void set_wheel_speeds(int m1, int m2, int m3, int m4);
void set_wheel_speeds_with_mode(int m1, int m2, int m3, int m4, MovementMode mode);



#ifdef __cplusplus
}
#endif

#endif /* MOVE_H */

/*****END OF FILE*****/
