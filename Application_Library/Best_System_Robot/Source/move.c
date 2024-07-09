#include "move.h"

void set_wheel_speeds(int m1, int m2, int m3, int m4) // target speeds
{
    get_dc_motor(&motor_shield_l29xx, MS_L29XX, M1)->target_speed = m1;
    get_dc_motor(&motor_shield_l29xx, MS_L29XX, M2)->target_speed = m2;
    get_dc_motor(&motor_shield_l29xx, MS_L29XX, M3)->target_speed = m3;
    get_dc_motor(&motor_shield_l29xx, MS_L29XX, M4)->target_speed = m4;

    // may need to call other functions to actually execute motor control
    // e.g. ms_motor_control(&motor_shield_l29xx, MS_L29XX, M1, m1);
    // ame as M2, M3, M4
}


// move forward or backward
void move(int speed) {
    set_motor_speeds(speed, speed, speed, speed);
}


// turn left or right
void turn(int speed) {
    set_motor_speeds(speed, -speed, speed, -speed);
}


// move strafing 
void strafe(int speed) {
    set_motor_speeds(speed, -speed, -speed, speed);
}


// move diagonally
void diagonal_move(int speed, int direction) {
    // direction: 1 for forward-right, -1 for forward-left
    if (direction > 0) {
        set_motor_speeds(speed, 0, 0, speed);  // right front
    } else {
        set_motor_speeds(0, speed, speed, 0);  // left front
    }
}