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
