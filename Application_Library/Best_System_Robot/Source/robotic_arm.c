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

enum { // State machine for grabbing shuttlecock and for initial checking
    MOVE_TO_GRAB,
    CLOSE_GRIPPER
} grabSubState = MOVE_TO_GRAB;

void HandleInitState(void)
{
    static enum {
        INIT_SERVOS,
        MOVE_TO_START
    } initSubState = INIT_SERVOS;

    switch (initSubState) {
    case INIT_SERVOS:
        servo_control(&servo[S1], -3, ANGLE, true);
        servo_control(&servo[S2], 0, ANGLE, true);
        servo_control(&servo[S3], 40, ANGLE, true);
        if (is_pwm_at_angle(&servo[S1], -3) && is_pwm_at_angle(&servo[S2], 0) && is_pwm_at_angle(&servo[S3], 40)) {
            initSubState = MOVE_TO_START;
        }
        break;

    case MOVE_TO_START:
        if (!Button_IsPressed(&button[B2])) {
            ms_motor_control(&motor_shield_v1, MS_V1, M1, -1000);
        } else {
            ms_motor_control(&motor_shield_v1, MS_V1, M1, 0);
            if(!myDelay.active) {
                myDelay.Start(&myDelay, 200);
            } else if (myDelay.IsExpired(&myDelay)) {
                roboticArmState = STATE_IDLE;
                initSubState = INIT_SERVOS;  // Reset for next time

                // Modify the grabSubState of HandleGrabShuttlecockState
                // extern enum { MOVE_TO_GRAB, CLOSE_GRIPPER } grabSubState;
                grabSubState = MOVE_TO_GRAB;
            }
        }
        break;
    }
}

void HandleGrabShuttlecockState(void)
{
    // static enum {
    //     MOVE_TO_GRAB,
    //     CLOSE_GRIPPER
    // } grabSubState = MOVE_TO_GRAB;

    switch (grabSubState) {
    case MOVE_TO_GRAB:
        if (!Button_IsPressed(&button[B1])) {
            ms_motor_control(&motor_shield_v1, MS_V1, M1, 1000);
        } else {
            ms_motor_control(&motor_shield_v1, MS_V1, M1, 0);
            grabSubState = CLOSE_GRIPPER;
        }
        break;

    case CLOSE_GRIPPER:
        if(!myDelay.active) {
            servo_control(&servo[S3], 0, ANGLE, true);
            myDelay.Start(&myDelay, 500);
        } else if (myDelay.IsExpired(&myDelay)) {
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
        myDelay.Start(&myDelay, 0);
    } else if (myDelay.IsExpired(&myDelay)) {
        roboticArmState = STATE_SORT_AND_DROP;
    }
}

void HandleSortAndDropState(void)
{
    static enum {
        MOVE_HOLDER_AND_ROTATE,
        ADJUST_S1,
        RELEASE_SHUTTLECOCK,
        RESET_S1
    } sortDropSubState = MOVE_HOLDER_AND_ROTATE;
    static NonBlockingDelay_TypeDef rotateDelay = INIT_NON_BLOCKING_DELAY();

    if (defect_result_received) {
        switch (sortDropSubState) {
        case MOVE_HOLDER_AND_ROTATE:
            // Move holder
            if (!Button_IsPressed(defect_result ? &button[B4] : &button[B3])) {
                ms_motor_control(&motor_shield_v1, MS_V1, M2, defect_result ? -1000 : 1000);
            } else {
                ms_motor_control(&motor_shield_v1, MS_V1, M2, 0);
            }

            // Rotate S2 to 90 degrees
            servo_control(&servo[S2], 90, ANGLE, true);

            // Move M1 and rotate S2 to 180 degrees
            if (is_pwm_at_angle(&servo[S2], 90)) {
                ms_motor_control(&motor_shield_v1, MS_V1, M1, 1000);
                if (Button_IsPressed(&button[B1])) {
                    ms_motor_control(&motor_shield_v1, MS_V1, M1, 0);
                    servo_control(&servo[S2], 180, ANGLE, true);
                }
            }

            // Check if all movements are complete
            if (Button_IsPressed(defect_result ? &button[B4] : &button[B3]) &&
                    Button_IsPressed(&button[B1]) &&
                    is_pwm_at_angle(&servo[S2], 180)) {
                sortDropSubState = ADJUST_S1;
            }
            break;

        case ADJUST_S1:
            servo_control(&servo[S1], defect_result ? -85 : 65, ANGLE, true);
            if (is_pwm_at_angle(&servo[S1], defect_result ? -85 : 65)) {
                if (!rotateDelay.active) {
                    rotateDelay.Start(&rotateDelay, 500);
                } else if (rotateDelay.IsExpired(&rotateDelay)) {
                    sortDropSubState = RELEASE_SHUTTLECOCK;
                }
            }
            break;

        case RELEASE_SHUTTLECOCK:
            servo_control(&servo[S3], 15, ANGLE, true);
            if (!rotateDelay.active) {
                rotateDelay.Start(&rotateDelay, 500);
            } else if (rotateDelay.IsExpired(&rotateDelay)) {
                sortDropSubState = RESET_S1;
            }
            break;

        case RESET_S1:
            servo_control(&servo[S1], 0, ANGLE, true);
            if (is_pwm_at_angle(&servo[S1], 0)) {
                roboticArmState = STATE_STORE_SHUTTLECOCK;
                sortDropSubState = MOVE_HOLDER_AND_ROTATE;  // Reset for next time
            }
            break;
        }
    }
}

void HandleStoreShuttlecockState(void)
{
    static enum {
        PUSH_SHUTTLECOCK,
        CHECK_BUCKET,
        ROTATE_WHEEL
    } storeSubState = PUSH_SHUTTLECOCK;
    static int check_count = 0;
    static bool skip_check = true;

    switch (storeSubState) {
    case PUSH_SHUTTLECOCK:
        // Push shuttlecock
        ms_motor_control(&motor_shield_v1, MS_V1, M2, defect_result ? 1000 : -1000);

        // Simultaneously execute STATE_INIT
        HandleInitState();

        if (Button_IsPressed(defect_result ? &button[B3] : &button[B4])) {
            ms_motor_control(&motor_shield_v1, MS_V1, M2, 0);
            check_count = 0;
            if (skip_check) {
                roboticArmState = STATE_GRAB_SHUTTLECOCK;
            } else {
                storeSubState = CHECK_BUCKET;
            }
        }
        break;

    case CHECK_BUCKET:
        if (!CheckBucketFull(defect_result) || skip_check) {
            roboticArmState = STATE_GRAB_SHUTTLECOCK;
            storeSubState = PUSH_SHUTTLECOCK;  // Reset for next time
        } else {
            storeSubState = ROTATE_WHEEL;
            check_count++;
        }
        break;

    case ROTATE_WHEEL:
        RotateRevolver(defect_result);
        static NonBlockingDelay_TypeDef wheelDelay = INIT_NON_BLOCKING_DELAY();
        if (!wheelDelay.active) {
            wheelDelay.Start(&wheelDelay, 1000);
        } else if (wheelDelay.IsExpired(&wheelDelay)) {
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

void RotateRevolver(bool defect_result)
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