#include "robotic_arm.h"

/* Global NonBlockingDelay Variables */
NonBlockingDelay_TypeDef myDelay = INIT_NON_BLOCKING_DELAY();

/* Robotic Arm Configuration---------------------------------------*/
RoboticArmState_TypeDef roboticArmState = STATE_INIT;
bool defect_result_received = false;
bool defect_result = false;

void UpdateRoboticArmState(void)
{
    switch (roboticArmState) {
    case STATE_IDLE:
        HandleIdleState();
        break;
    case STATE_INIT:
        HandleInitState();
        break;
    case STATE_GRAB_SHUTTLECOCK:
        HandleGrabShuttlecockState();
        break;
    case STATE_MOVE_TO_SCAN:
        HandleMoveToScanState();
        break;
    case STATE_WAIT_FOR_SCAN:
        HandleWaitForScanState();
        break;
    case STATE_SORT_AND_DROP:
        HandleSortAndDropState();
        break;
    case STATE_STORE_SHUTTLECOCK:
        HandleStoreShuttlecockState();
        break;
    case STATE_STOP:
        HandleStopState();
        break;
    default:
        break;
    }
}

void HandleIdleState(void)
{
    // Wait for further instructions
}

void HandleInitState(void)
{
    static enum {
        INIT_SERVOS,
        MOVE_TO_START,
        WAIT_FOR_STABILITY
    } initSubState = INIT_SERVOS;

    switch (initSubState) {
        case INIT_SERVOS:
            servo_control(&servo[S1], -3, ANGLE, true);
            servo_control(&servo[S2], 0, ANGLE, true);
            servo_control(&servo[S3], 30, ANGLE, true);
            if (is_pwm_at_angle(&servo[S1], -3) && is_pwm_at_angle(&servo[S2], 0) && is_pwm_at_angle(&servo[S3], 30)) {
                initSubState = MOVE_TO_START;
            }
            break;

        case MOVE_TO_START:
            if (!Button_IsPressed(&button[B2])) {
                ms_motor_control(&motor_shield_v1, MS_V1, M1, -1000);
            } else {
                ms_motor_control(&motor_shield_v1, MS_V1, M1, 0);
                initSubState = WAIT_FOR_STABILITY;
                myDelay.Start(&myDelay, 200);
            }
            break;

        case WAIT_FOR_STABILITY:
            if (myDelay.IsExpired(&myDelay)) {
                roboticArmState = STATE_IDLE;
                initSubState = INIT_SERVOS;  // Reset for next time
            }
            break;
    }
}

void HandleGrabShuttlecockState(void)
{
    static enum {
        MOVE_TO_GRAB,
        CLOSE_GRIPPER,
        CONFIRM_GRIP
    } grabSubState = MOVE_TO_GRAB;

    switch (grabSubState) {
        case MOVE_TO_GRAB:
            if (!Button_IsPressed(&button[B1])) {
                ms_motor_control(&motor_shield_v1, MS_V1, M1, 1000);
            } else {
                ms_motor_control(&motor_shield_v1, MS_V1, M1, 0);
                myDelay.Start(&myDelay, 200);
                grabSubState = CLOSE_GRIPPER;
            }
            break;

        case CLOSE_GRIPPER:
            if (myDelay.IsExpired(&myDelay)) {
                servo_control(&servo[S3], 0, ANGLE, true);
                myDelay.Start(&myDelay, 500);
                grabSubState = CONFIRM_GRIP;
            }
            break;

        case CONFIRM_GRIP:
            if (myDelay.IsExpired(&myDelay) && is_pwm_at_angle(&servo[S3], 0)) {
                roboticArmState = STATE_MOVE_TO_SCAN;
                grabSubState = MOVE_TO_GRAB;  // Reset for next time
            }
            break;
    }
}

void HandleMoveToScanState(void)
{
    static enum {
        MOVE_TO_SCAN_POSITION,
        ADJUST_ARM
    } scanSubState = MOVE_TO_SCAN_POSITION;

    switch (scanSubState) {
        case MOVE_TO_SCAN_POSITION:
            if (!Button_IsPressed(&button[B2])) {
                ms_motor_control(&motor_shield_v1, MS_V1, M1, -1000);
            } else {
                ms_motor_control(&motor_shield_v1, MS_V1, M1, 0);
                scanSubState = ADJUST_ARM;
            }
            break;

        case ADJUST_ARM:
            servo_control(&servo[S1], -15, ANGLE, true);
            servo_control(&servo[S2], 10, ANGLE, true);
            servo_control(&servo[S3], 1, ANGLE, true);
            if (is_pwm_at_angle(&servo[S1], -15) && is_pwm_at_angle(&servo[S2], 10) && is_pwm_at_angle(&servo[S3], 1)) {
                roboticArmState = STATE_WAIT_FOR_SCAN;
                scanSubState = MOVE_TO_SCAN_POSITION;  // Reset for next time
            }
            break;
    }
}

void HandleWaitForScanState(void)
{
    if (!myDelay.active) {
        defect_result_received = false;
        printf("{\"request\":\"scan_defect\"}");
        myDelay.Start(&myDelay, 1000);
    } else if (myDelay.IsExpired(&myDelay)) {
        roboticArmState = STATE_SORT_AND_DROP;
    }
}

void HandleSortAndDropState(void)
{
    static enum {
        MOVE_HOLDER,
        ROTATE_S2_90,
        MOVE_M1_ROTATE_S2_180,
        ADJUST_S1,
        RELEASE_SHUTTLECOCK,
        RESET_S1
    } sortDropSubState = MOVE_HOLDER;

    switch (sortDropSubState) {
        case MOVE_HOLDER:
            if (!Button_IsPressed(defect_result ? &button[B4] : &button[B3])) {
                ms_motor_control(&motor_shield_v1, MS_V1, M2, defect_result ? -1000 : 1000);
            } else {
                ms_motor_control(&motor_shield_v1, MS_V1, M2, 0);
                servo_control(&servo[S1], 0, ANGLE, true);
                myDelay.Start(&myDelay, 200);
                sortDropSubState = ROTATE_S2_90;
            }
            break;

        case ROTATE_S2_90:
            if (myDelay.IsExpired(&myDelay)) {
                servo_control(&servo[S2], 90, ANGLE, true);
                if (is_pwm_at_angle(&servo[S2], 90)) {
                    sortDropSubState = MOVE_M1_ROTATE_S2_180;
                }
            }
            break;

        case MOVE_M1_ROTATE_S2_180:
            ms_motor_control(&motor_shield_v1, MS_V1, M1, 1000);
            if (Button_IsPressed(&button[B1])) {
                ms_motor_control(&motor_shield_v1, MS_V1, M1, 0);
                servo_control(&servo[S2], 180, ANGLE, true);
                if (is_pwm_at_angle(&servo[S2], 180)) {
                    sortDropSubState = ADJUST_S1;
                }
            }
            break;

        case ADJUST_S1:
            servo_control(&servo[S1], defect_result ? -85 : 65, ANGLE, true);
            if (is_pwm_at_angle(&servo[S1], defect_result ? -85 : 65)) {
                myDelay.Start(&myDelay, 500);
                sortDropSubState = RELEASE_SHUTTLECOCK;
            }
            break;

        case RELEASE_SHUTTLECOCK:
            if (myDelay.IsExpired(&myDelay)) {
                servo_control(&servo[S3], 10, ANGLE, true);
                myDelay.Start(&myDelay, 500);
                sortDropSubState = RESET_S1;
            }
            break;

        case RESET_S1:
            if (myDelay.IsExpired(&myDelay)) {
                servo_control(&servo[S1], 0, ANGLE, true);
                if (is_pwm_at_angle(&servo[S1], 0)) {
                    roboticArmState = STATE_STORE_SHUTTLECOCK;
                    sortDropSubState = MOVE_HOLDER;  // Reset for next time
                }
            }
            break;
    }
}

void HandleStoreShuttlecockState(void)
{
    static enum {
        CHECK_BUCKET,
        PUSH_SHUTTLECOCK,
        ROTATE_WHEEL
    } storeSubState = CHECK_BUCKET;
    static int check_count = 0;

    switch (storeSubState) {
        case CHECK_BUCKET:
            if (!CheckBucketFull(defect_result)) {
                storeSubState = PUSH_SHUTTLECOCK;
            } else {
                storeSubState = ROTATE_WHEEL;
                check_count++;
            }
            break;

        case PUSH_SHUTTLECOCK:
            ms_motor_control(&motor_shield_v1, MS_V1, M2, defect_result ? 1000 : -1000);
            if (Button_IsPressed(defect_result ? &button[B3] : &button[B4])) {
                ms_motor_control(&motor_shield_v1, MS_V1, M2, 0);
                check_count = 0;
                roboticArmState = STATE_INIT;
                storeSubState = CHECK_BUCKET;  // Reset for next time
            }
            break;

        case ROTATE_WHEEL:
            RotateWheel(defect_result);
            myDelay.Start(&myDelay, 1000);
            if (myDelay.IsExpired(&myDelay)) {
                if (check_count >= 5) {
                    roboticArmState = STATE_STOP;
                } else {
                    storeSubState = CHECK_BUCKET;
                }
            }
            break;
    }
}

void HandleStopState(void)
{
    all_pwm_stop();
    ms_motor_control(&motor_shield_v1, MS_V1, M1, 0);
    ms_motor_control(&motor_shield_v1, MS_V1, M2, 0);
    ms_motor_control(&motor_shield_v1, MS_V1, M3, 0);
    ms_motor_control(&motor_shield_v1, MS_V1, M4, 0);
    
    roboticArmState = STATE_IDLE;
}

bool CheckBucketFull(bool defect_result)
{
    float distance = defect_result ? hc_sr04[1].distance : hc_sr04[2].distance;
    return distance < 5.0;
}

void RotateWheel(bool defect_result)
{
    if (defect_result) {
        if (!Button_IsPressed(&button[B5])) {
            ms_motor_control(&motor_shield_v1, MS_V1, M3, 1000);
        } else {
            ms_motor_control(&motor_shield_v1, MS_V1, M3, 0);
        }
    } else {
        if (!Button_IsPressed(&button[B6])) {
            ms_motor_control(&motor_shield_v1, MS_V1, M4, 1000);
        } else {
            ms_motor_control(&motor_shield_v1, MS_V1, M4, 0);
        }
    }
}