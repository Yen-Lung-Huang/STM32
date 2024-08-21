#include "robotic_arm.h"

/* Sensor & Actuators Function---------------------------------------*/
void CheckButtonsAndStopMotors(void)
{
    if (Button_IsPressed(&button[B1]) || Button_IsPressed(&button[B2])) {
        ms_motor_control(&motor_shield_v1, MS_V1, M1, 0);
    }
    if (Button_IsPressed(&button[B3]) || Button_IsPressed(&button[B4])) {
        ms_motor_control(&motor_shield_v1, MS_V1, M2, 0);
    }
    if (!Button_IsPressed(&button[B4])) {
        ms_motor_control(&motor_shield_v1, MS_V1, M3, 0);
    }
    if (!Button_IsPressed(&button[B3])) {
        ms_motor_control(&motor_shield_v1, MS_V1, M4, 0);
    }
}

/* Robotic Arm Configuration---------------------------------------*/

RoboticArmState_TypeDef roboticArmState = STATE_IDLE;
NonBlockingDelay_TypeDef myDelay;

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
        case STATE_MOVE_TO_GRAB:
            HandleMoveToGrabState();
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
    myDelay = CreateNonBlockingDelay();
}

void HandleInitState(void)
{
    servo_control(&servo[S1], -3, ANGLE, true);
    servo_control(&servo[S2], 0, ANGLE, true);
    servo_control(&servo[S3], 30, ANGLE, true);

    if (!Button_IsPressed(&button[B2])) {
        ms_motor_control(&motor_shield_v1, MS_V1, M1, -1000);
    } else {
        ms_motor_control(&motor_shield_v1, MS_V1, M1, 0);
        if (!myDelay.active) {
            myDelay.Start(&myDelay, 200);
        }
    }

    if (myDelay.IsExpired(&myDelay)) {
        roboticArmState = STATE_MOVE_TO_GRAB;
    }
}

void HandleMoveToGrabState(void)
{
    if (!myDelay.active) {
        if (Button_IsPressed(&button[B1])) {
            ms_motor_control(&motor_shield_v1, MS_V1, M1, 0);
            myDelay.Start(&myDelay, 200);
        } else {
            ms_motor_control(&motor_shield_v1, MS_V1, M1, 1000);
        }
    } else if (myDelay.IsExpired(&myDelay)) {
        roboticArmState = STATE_GRAB_SHUTTLECOCK;
    }
}

void HandleGrabShuttlecockState(void)
{
    if (!myDelay.active) {
        servo_control(&servo[S3], 0, ANGLE, true);
        myDelay.Start(&myDelay, 100);
    } else if (myDelay.IsExpired(&myDelay)) {
        roboticArmState = STATE_MOVE_TO_SCAN;
    }
}

void HandleMoveToScanState(void)
{
    if (!myDelay.active) {
        if (Button_IsPressed(&button[B2])) {
            ms_motor_control(&motor_shield_v1, MS_V1, M1, 0);
            servo_control(&servo[S1], -15, ANGLE, true);
            servo_control(&servo[S2], 10, ANGLE, true);
            servo_control(&servo[S3], 1, ANGLE, true);
            myDelay.Start(&myDelay, 100);
        } else {
            ms_motor_control(&motor_shield_v1, MS_V1, M1, -1000);
        }
    } else if (myDelay.IsExpired(&myDelay)) {
        roboticArmState = STATE_WAIT_FOR_SCAN;
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
    if (defect_result_received) {
        if (!myDelay.active) {
            // Control M2 to move the ball holder to the appropriate position
            if (defect_result) {
                // Handle good shuttlecock
                if (!Button_IsPressed(&button[B4])) {
                    ms_motor_control(&motor_shield_v1, MS_V1, M2, -1000); // Move the ball holder until B4 is pressed
                } else {
                    ms_motor_control(&motor_shield_v1, MS_V1, M2, 0); // Stop ball holder movement
                    servo_control(&servo[S1], 0, ANGLE, true); // Reset S1 to 0 degrees
                    servo_control(&servo[S2], 90, ANGLE, true); // Set S2 to 90 degrees
                    myDelay.Start(&myDelay, 500); // Start a non-blocking delay to ensure S2 reaches 90 degrees
                }
            } else {
                // Handle defective shuttlecock
                if (!Button_IsPressed(&button[B3])) {
                    ms_motor_control(&motor_shield_v1, MS_V1, M2, 1000); // Move the ball holder until B3 is pressed
                } else {
                    ms_motor_control(&motor_shield_v1, MS_V1, M2, 0); // Stop ball holder movement
                    servo_control(&servo[S1], 0, ANGLE, true); // Reset S1 to 0 degrees
                    servo_control(&servo[S2], 90, ANGLE, true); // Set S2 to 90 degrees
                    myDelay.Start(&myDelay, 500); // Start a non-blocking delay to ensure S2 reaches 90 degrees
                }
            }

            // Control M1 to move the carriage until B1 is pressed
            if (myDelay.IsExpired(&myDelay)) {
                ms_motor_control(&motor_shield_v1, MS_V1, M1, 1000); // Move the carriage forward
                if (Button_IsPressed(&button[B1])) {
                    ms_motor_control(&motor_shield_v1, MS_V1, M1, 0); // Stop carriage movement
                    servo_control(&servo[S2], 180, ANGLE, true); // Set S2 to 180 degrees
                    myDelay.Start(&myDelay, 1000); // Start a non-blocking delay to ensure S2 reaches 180 degrees
                }
            }

            // After reaching the required position, adjust S1 and release S3 to drop the shuttlecock
            if (myDelay.IsExpired(&myDelay)) {
                servo_control(&servo[S1], defect_result ? -65 : 65, ANGLE, true); // Set S1 to -65 or 65 degrees based on shuttlecock quality
                servo_control(&servo[S3], 10, ANGLE, true); // Release gripper to drop the shuttlecock
                myDelay.Start(&myDelay, 1000); // Start a non-blocking delay to ensure shuttlecock is dropped
            }
        } else if (myDelay.IsExpired(&myDelay)) {
            roboticArmState = STATE_STORE_SHUTTLECOCK; // Transition to storing shuttlecock state
        }
    }
}

void HandleStoreShuttlecockState(void)
{
    static int check_count = 0;
    bool skip_check = true;

    if (!myDelay.active) {
        if (skip_check || !CheckBucketFull(defect_result)) {
            ms_motor_control(&motor_shield_v1, MS_V1, M2, defect_result ? 1000 : -1000);
            myDelay.Start(&myDelay, 1000);
            check_count = 0;
        } else {
            RotateWheel(defect_result);
            myDelay.Start(&myDelay, 1000);
            check_count++;
        }
    } else if (myDelay.IsExpired(&myDelay)) {
        if (check_count >= 5) {
            roboticArmState = STATE_STOP;
        } else {
            roboticArmState = STATE_INIT;
        }
    }
}

void HandleStopState(void)
{
    all_pwm_stop();
    ms_motor_control(&motor_shield_v1, MS_V1, M1, 0);
    ms_motor_control(&motor_shield_v1, MS_V1, M2, 0);
    ms_motor_control(&motor_shield_v1, MS_V1, M3, 0);
    ms_motor_control(&motor_shield_v1, MS_V1, M4, 0);
}

bool CheckBucketFull(bool defect_result)
{
    float distance = defect_result ? hc_sr04[1].distance : hc_sr04[2].distance;
    return distance < 5.0;
}

void RotateWheel(bool defect_result)
{
    if (defect_result) {
        if (!Button_IsPressed(&button[B4]) || Button_IsPressed(&button[B3])) {
            ms_motor_control(&motor_shield_v1, MS_V1, M3, 0);
        } else {
            ms_motor_control(&motor_shield_v1, MS_V1, M3, 1000);
        }
        if (Button_IsPressed(&button[B5])) {
            ms_motor_control(&motor_shield_v1, MS_V1, M3, 0);
        }
    } else {
        if (!Button_IsPressed(&button[B3]) || Button_IsPressed(&button[B4])) {
            ms_motor_control(&motor_shield_v1, MS_V1, M4, 0);
        } else {
            ms_motor_control(&motor_shield_v1, MS_V1, M4, 1000);
        }
        if (Button_IsPressed(&button[B6])) {
            ms_motor_control(&motor_shield_v1, MS_V1, M4, 0);
        }
    }
}
