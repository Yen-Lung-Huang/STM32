/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MOVE_H
#define MOVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_shield_v1.h"

typedef enum {
    MOVE_FORWARD,
    TURN,
    STRAFE,
    DIAGONAL
} MovementMode;

void set_wheel_speeds(int m1, int m2, int m3, int m4);

//void wheels_degree_set(float front_left, float front_right, float rear_left, float rear_right);
//void wheels_speed_set(float left_speed, float right_speed);
//void wheels_stop(void);
//void mobile_pwm_stop(void);
//void turn_set(bool turn);
//void move(float speed);
//void turn(float speed);

#ifdef __cplusplus
}
#endif

#endif /* MOVE_H */

/*****END OF FILE*****/
