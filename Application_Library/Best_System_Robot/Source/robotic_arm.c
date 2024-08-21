#include "robotic_arm.h"

/* Sensor & Actuators Function---------------------------------------*/
void CheckButtonsAndStopMotors(void)
{
    // Check if Button B1 or B2 is pressed to stop Motor M1
    if (Button_IsPressed(&button[B1]) || Button_IsPressed(&button[B2])) {
        ms_motor_control(&motor_shield_v1, MS_V1, M1, 0); // Stop Motor M1
    }

    // Check if Button B3 or B4 is pressed to stop Motor M2
    if (Button_IsPressed(&button[B3]) || Button_IsPressed(&button[B4])) {
        ms_motor_control(&motor_shield_v1, MS_V1, M2, 0); // Stop Motor M2
    }

    // Check if Button B4 is not pressed to stop Motor M3
    if (!Button_IsPressed(&button[B4])) {
        ms_motor_control(&motor_shield_v1, MS_V1, M3, 0); // Stop Motor M3
    }

    // Check if Button B3 is not pressed to stop Motor M4
    if (!Button_IsPressed(&button[B3])) {
        ms_motor_control(&motor_shield_v1, MS_V1, M4, 0); // Stop Motor M4
    }
}

/* Robotic Arm Configuration---------------------------------------*/

RoboticArmState_TypeDef roboticArmState = STATE_INIT;
NonBlockingDelay_TypeDef myDelay;

bool defect_result_received = false; // Flag to indicate if defect result is received
bool defect_result = false;          // Variable to store the defect result

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
    myDelay = CreateNonBlockingDelay(); // Initialize the non-blocking delay object
    // Stay in IDLE state until further instructions
}

void HandleInitState(void)
{
    servo_control(&servo[S1], -3, ANGLE, true); // Set Servo S1 to -3 degrees
    servo_control(&servo[S2], 0, ANGLE, true);  // Set Servo S2 to 0 degrees
    servo_control(&servo[S3], 30, ANGLE, true); // Open gripper to 30 degrees

    // Ensure the carriage hits button B2
    if (!Button_IsPressed(&button[B2])) {
        ms_motor_control(&motor_shield_v1, MS_V1, M1, -1000); // Move the carriage until button B2 is pressed
    } else {
        ms_motor_control(&motor_shield_v1, MS_V1, M1, 0); // Stop the carriage movement
        myDelay.Start(&myDelay, 200);                     // Start a non-blocking delay of 200 ms to allow the arm to stabilize
    }

    if (myDelay.IsExpired(&myDelay)) {
        // Check if the delay has expired
        roboticArmState = STATE_MOVE_TO_GRAB; // Transition to the next state
    }
}

void HandleMoveToGrabState(void)
{
    if (!myDelay.active) {
        if (Button_IsPressed(&button[B1])) {
            ms_motor_control(&motor_shield_v1, MS_V1, M1, 0); // Stop Motor M1
            myDelay.Start(&myDelay, 200);                     // Start a non-blocking delay of 200 ms to allow the arm to stabilize
        } else {
            ms_motor_control(&motor_shield_v1, MS_V1, M1, 1000); // Move the carriage forward
        }
    } else {
        if (myDelay.IsExpired(&myDelay)) {
            // Check if the delay has expired
            roboticArmState = STATE_GRAB_SHUTTLECOCK; // Transition to the next state
        }
    }
}

void HandleGrabShuttlecockState(void)
{
    if (!myDelay.active) {
        servo_control(&servo[S3], 0, ANGLE, true); // Close the gripper
        myDelay.Start(&myDelay, 100);              // Start a non-blocking delay of 100 ms to ensure the gripper is fully closed
    } else {
        if (myDelay.IsExpired(&myDelay)) {
            // Check if the delay has expired
            roboticArmState = STATE_MOVE_TO_SCAN; // Transition to the next state
        }
    }
}

void HandleMoveToScanState(void)
{
    if (!myDelay.active) {
        if (Button_IsPressed(&button[B2])) {
            ms_motor_control(&motor_shield_v1, MS_V1, M1, 0); // Stop Motor M1
            servo_control(&servo[S1], 15, ANGLE, true);       // Set Servo S1 to 15 degrees
            servo_control(&servo[S2], 10, ANGLE, true);       // Set Servo S2 to 10 degrees
            servo_control(&servo[S3], 1, ANGLE, true);        // Set Servo S3 to 1 degrees
            myDelay.Start(&myDelay, 100);                     // Start a non-blocking delay of 100 ms to ensure the servo reaches the scanning position
        } else {
            ms_motor_control(&motor_shield_v1, MS_V1, M1, -1000); // Move the carriage backward
        }
    } else {
        if (myDelay.IsExpired(&myDelay)) {
            // Check if the delay has expired
            roboticArmState = STATE_WAIT_FOR_SCAN; // Transition to the next state
        }
    }
}

void HandleWaitForScanState(void)
{
    if (!myDelay.active) {
        defect_result_received = false;          // Reset the defect result flag
        printf("{\"request\":\"scan_defect\"}"); // Send scan request to RPi
        myDelay.Start(&myDelay, 1000);           // Start a non-blocking delay to wait for the RPi response
    } else {
        if (myDelay.IsExpired(&myDelay)) {
            // Check if the delay has expired
            roboticArmState = STATE_SORT_AND_DROP; // Transition to the next state
        }
    }
}

void HandleSortAndDropState(void)
{
    if (defect_result_received) {
        // Check if the defect result is received
        if (!myDelay.active) {
            servo_control(&servo[S1], defect_result ? -65 : 65, ANGLE, true); // Sort Shuttlecock by setting Servo S1 to -65 or 65 degrees
            servo_control(&servo[S3], 10, ANGLE, true);                       // Open the gripper to drop the shuttlecock
            myDelay.Start(&myDelay, 1000);                                    // Start a non-blocking delay of 1000 ms to ensure the shuttlecock is dropped
        } else {
            if (myDelay.IsExpired(&myDelay)) {
                // Check if the delay has expired
                roboticArmState = STATE_STORE_SHUTTLECOCK; // Transition to the next state
            }
        }
    }
}

void HandleStoreShuttlecockState(void)
{
    static int check_count = 0; // Add a counter to track the number of checks
    bool skip_check = true; // Set this to true to skip the bucket full check

    if (!myDelay.active) {
        if (skip_check || !CheckBucketFull(defect_result)) {
            // Skip the bucket full check or if the bucket is not full
            ms_motor_control(&motor_shield_v1, MS_V1, M2, defect_result ? 1000 : -1000); // Push the shuttlecock into the cylinder
            myDelay.Start(&myDelay, 1000); // Start a non-blocking delay to ensure the shuttlecock is pushed
            check_count = 0; // Reset the counter if a non-full cylinder is found
        } else {
            // If the bucket is full
            RotateWheel(defect_result); // Rotate the wheel to the next cylinder
            myDelay.Start(&myDelay, 1000); // Start a non-blocking delay to ensure the wheel rotation
            check_count++;
        }
    } else {
        if (myDelay.IsExpired(&myDelay)) { // Check if the delay has expired
            if (check_count >= 5) { // If all cylinders are full after 5 checks
                roboticArmState = STATE_STOP; // Transition to the stop state
            } else {
                roboticArmState = STATE_INIT; // Transition to the initial state
            }
        }
    }
}

void HandleStopState(void)
{
    all_pwm_stop();                                   // Stop all PWM signals
    ms_motor_control(&motor_shield_v1, MS_V1, M1, 0); // Stop Motor M1
    ms_motor_control(&motor_shield_v1, MS_V1, M2, 0); // Stop Motor M2
    ms_motor_control(&motor_shield_v1, MS_V1, M3, 0); // Stop Motor M3
    ms_motor_control(&motor_shield_v1, MS_V1, M4, 0); // Stop Motor M4
    // Stay in STOP state until further instructions
}

bool CheckBucketFull(bool defect_result)
{
    // Use HCSR04 ultrasonic sensor to check if the bucket is full
    // Determine whether to check the good bucket or the bad bucket based on defect_result
    // Use hc_sr04[1].distance and hc_sr04[2].distance to get the distance

    float distance;
    if (defect_result) {
        // Check the good bucket
        distance = hc_sr04[1].distance;
    } else {
        // Check the bad bucket
        distance = hc_sr04[2].distance;
    }

    // Assume that a distance less than a certain threshold (e.g., 5 cm) indicates the bucket is full
    if (distance < 5.0) {
        return true; // Bucket is full
    } else {
        return false; // Bucket is not full
    }
}

void RotateWheel(bool defect_result)
{
    // Determine whether to rotate to the next good bucket or bad bucket based on defect_result
    // Use ms_motor_control() to control the motor

    if (defect_result) {
        if (!Button_IsPressed(&button[B4]) || Button_IsPressed(&button[B3])) {
            // Rotate to the next good bucket position
            ms_motor_control(&motor_shield_v1, MS_V1, M3, 0); // Stop Motor M3
        } else {
            ms_motor_control(&motor_shield_v1, MS_V1, M3, 1000); // Rotate Motor M3
        }
        if (Button_IsPressed(&button[B5])) {
            ms_motor_control(&motor_shield_v1, MS_V1, M3, 0); // Stop Motor M3
        }
    } else {
        if (!Button_IsPressed(&button[B3]) || Button_IsPressed(&button[B4])) {
            // Rotate to the next bad bucket position
            ms_motor_control(&motor_shield_v1, MS_V1, M4, 0); // Stop Motor M4
        } else {
            ms_motor_control(&motor_shield_v1, MS_V1, M4, 1000); // Rotate Motor M4
        }
        if (Button_IsPressed(&button[B6])) {
            ms_motor_control(&motor_shield_v1, MS_V1, M4, 0); // Stop Motor M4
        }
    }
}
