#include "robotic_arm.h"

/* Global NonBlockingDelay Variables */
NonBlockingDelay_TypeDef myDelay = INIT_NON_BLOCKING_DELAY();
NonBlockingDelay_TypeDef s1Delay = INIT_NON_BLOCKING_DELAY();
NonBlockingDelay_TypeDef s2Delay = INIT_NON_BLOCKING_DELAY();
NonBlockingDelay_TypeDef m1Delay = INIT_NON_BLOCKING_DELAY();


/* Sensor & Actuators Function---------------------------------------*/
void CheckButtonsAndStopMotors(void)
{
    int m1_speed = get_dc_motor(&motor_shield_v1, MS_V1, M1)->controller.current_speed;
    int m2_speed = get_dc_motor(&motor_shield_v1, MS_V1, M2)->controller.current_speed;

    // Check M1 motor control based on button press and motor direction
    if ((Button_IsPressed(&button[B1]) && m1_speed > 0) ||
            (Button_IsPressed(&button[B2]) && m1_speed < 0)) {
        ms_motor_control(&motor_shield_v1, MS_V1, M1, 0); // Stop Motor M1
    }

    // Check M2 motor control based on button press and motor direction
    if ((Button_IsPressed(&button[B3]) && m2_speed > 0) ||
            (Button_IsPressed(&button[B4]) && m2_speed < 0)) {
        ms_motor_control(&motor_shield_v1, MS_V1, M2, 0); // Stop Motor M2
    }

    // Original logic for M3 and M4 motors
    if (!Button_IsPressed(&button[B4])) {
        ms_motor_control(&motor_shield_v1, MS_V1, M3, 0); // Stop Motor M3
    }
    if (!Button_IsPressed(&button[B3])) {
        ms_motor_control(&motor_shield_v1, MS_V1, M4, 0); // Stop Motor M4
    }
}

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
        roboticArmState = STATE_IDLE;
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
    static bool step1_done = false;
    static bool step2_done = false;
    static bool step3_done = false;
    static bool step4_done = false;
    static bool step5_done = false;

    if (defect_result_received) {
        // Step 1: Move ball holder (M2) and reset S1
        if (!step1_done) {
            printf("step 1\n");
            if (defect_result) {
                if (!Button_IsPressed(&button[B4])) {
                    ms_motor_control(&motor_shield_v1, MS_V1, M2, -1000);
                } else {
                    ms_motor_control(&motor_shield_v1, MS_V1, M2, 0);
                    servo_control(&servo[S1], 0, ANGLE, true);
                    s1Delay.Start(&s1Delay, 100);
                    step1_done = true;
                }
            } else {
                if (!Button_IsPressed(&button[B3])) {
                    ms_motor_control(&motor_shield_v1, MS_V1, M2, 1000);
                } else {
                    ms_motor_control(&motor_shield_v1, MS_V1, M2, 0);
                    servo_control(&servo[S1], 0, ANGLE, true);
                    s1Delay.Start(&s1Delay, 100);
                    step1_done = true;
                }
            }
        }

        // Step 2: Rotate S2 to 90 degrees
        if (!step2_done && s1Delay.IsExpired(&s1Delay)) {
            printf("step 2\n");
            servo_control(&servo[S2], 90, ANGLE, true);
            s2Delay.Start(&s2Delay, 1000);
            step2_done = true;
        }

        // Step 3: Move M1 and Rotate S2 to 180 degrees
        if (step2_done && !step3_done) {
            printf("step 3\n");
            ms_motor_control(&motor_shield_v1, MS_V1, M1, 1000);

            // Continuously check if Button B1 is pressed
            if (Button_IsPressed(&button[B1])) {
                printf("Button B1 pressed, stopping M1 and rotating S2 to 180 degrees\n");
                ms_motor_control(&motor_shield_v1, MS_V1, M1, 0); // Stop carriage movement
                servo_control(&servo[S2], 180, ANGLE, true); // Set S2 to 180 degrees
                m1Delay.Start(&m1Delay, 1000); // Start delay to ensure S2 reaches 180 degrees

                // Check if S2 has reached 180 degrees
                if (is_pwm_at_angle(&servo[S2], 180)) {
                    printf("S2 has reached 180 degrees\n");
                    step3_done = true;
                } else {
                    printf("S2 has not reached 180 degrees\n");
                }
            }
        }

        // Step 4: Adjust S1
        if (step3_done && !step4_done && m1Delay.IsExpired(&m1Delay)) {
            printf("step 4\n");
            servo_control(&servo[S1], defect_result ? -65 : 65, ANGLE, true);
            s1Delay.Start(&s1Delay, 1000);
            step4_done = true;
        }

        // Step 5: Release S3 to drop the shuttlecock
        if (step4_done && !step5_done && s1Delay.IsExpired(&s1Delay) && s2Delay.IsExpired(&s2Delay)) {
            printf("step 5\n");
            servo_control(&servo[S3], 10, ANGLE, true);
            s2Delay.Start(&s2Delay, 1000);
            step5_done = true;
        }

        // Step 6: Reset S1 to 0 degrees after dropping the shuttlecock
        if (step5_done && s2Delay.IsExpired(&s2Delay)) {
            printf("step 6\n");
            servo_control(&servo[S1], 0, ANGLE, true);
            roboticArmState = STATE_STORE_SHUTTLECOCK;

            // Reset steps
            step1_done = step2_done = step3_done = step4_done = step5_done = false;
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
