/* Robotic Arm Configuration---------------------------------------*/
RoboticArmState_TypeDef roboticArmState = STATE_INIT;

bool defect_result_received = false; // Flag to indicate if defect result is received
bool defect_result = false; // Variable to store the defect result

void UpdateRoboticArmState(void)
{
    switch (roboticArmState) {
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
    case STATE_WAIT_FOR_RPI:
        HandleWaitForRpiState();
        break;
    case STATE_SORT_AND_DROP:
        HandleSortAndDropState();
        break;
    case STATE_CHECK_STORAGE:
        HandleCheckStorageState();
        break;
    case STATE_ROTATE_AND_CHECK:
        HandleRotateAndCheckState();
        break;
    case STATE_STOP:
        HandleStopState();
        break;
    default:
        break;
    }
}

void HandleInitState(void) // X
{
    myDelay = CreateNonBlockingDelay(); // Initialize the non-blocking delay object
    servo_control(&servo[S1], -3, ANGLE, true); // Set Servo S1 to -3 degrees
    servo_control(&servo[S2], 0, ANGLE, true);  // Set Servo S2 to 0 degrees
    servo_control(&servo[S3], 40, ANGLE, true); // Open gripper

    // Ensure the carriage hits button B2
    if (!Button_IsPressed(&button[B2])) {
        ms_motor_control(&motor_shield_v1, MS_V1, M1, -1000); // Move the carriage until button B2 is pressed
    } else {
        ms_motor_control(&motor_shield_v1, MS_V1, M1, 0); // Stop the carriage movement
        myDelay.Start(&myDelay, 200); // Start a non-blocking delay of 200 ms to allow the arm to stabilize
    }

    if (myDelay.IsExpired(&myDelay)) { // Check if the delay has expired
        roboticArmState = STATE_MOVE_TO_GRAB; // Transition to the next state
    }
}

void HandleInitState(void)
{
    myDelay = CreateNonBlockingDelay(); // Initialize the non-blocking delay object
    servo_control(&servo[S1], -3, ANGLE, true); // Set Servo S1 to -3 degrees
    servo_control(&servo[S2], 0, ANGLE, true);  // Set Servo S2 to 0 degrees
    servo_control(&servo[S3], 40, ANGLE, true); // Open gripper

    // Ensure the carriage hits button B2
    if (!Button_IsPressed(&button[B2])) {
        ms_motor_control(&motor_shield_v1, MS_V1, M1, -1000); // Move the carriage until button B2 is pressed
    } else {
        ms_motor_control(&motor_shield_v1, MS_V1, M1, 0); // Stop the carriage movement
        myDelay.Start(&myDelay, 200); // Start a non-blocking delay of 200 ms to allow the arm to stabilize
    }

    if (myDelay.IsExpired(&myDelay)) { // Check if the delay has expired
        roboticArmState = STATE_MOVE_TO_GRAB; // Transition to the next state
    }
}


void HandleMoveToGrabState(void)
{
    if (!myDelay.active) {
        if (Button_IsPressed(&button[B1])) {
            ms_motor_control(&motor_shield_v1, MS_V1, M1, 0); // Stop Motor M1
            myDelay.Start(&myDelay, 200); // Start a non-blocking delay of 200 ms to allow the arm to stabilize
        } else {
            ms_motor_control(&motor_shield_v1, MS_V1, M1, 1000); // Move the carriage forward
        }
    } else {
        if (myDelay.IsExpired(&myDelay)) { // Check if the delay has expired
            roboticArmState = STATE_GRAB_SHUTTLECOCK; // Transition to the next state
        }
    }
}

void HandleGrabShuttlecockState(void)
{
    if (!myDelay.active) {
        servo_control(&servo[S3], 0, ANGLE, true); // Close the gripper
        myDelay.Start(&myDelay, 100); // Start a non-blocking delay of 100 ms to ensure the gripper is fully closed
    } else {
        if (myDelay.IsExpired(&myDelay)) { // Check if the delay has expired
            roboticArmState = STATE_MOVE_TO_SCAN; // Transition to the next state
        }
    }
}

void HandleMoveToScanState(void)
{
    if (!myDelay.active) {
        if (Button_IsPressed(&button[B2])) {
            ms_motor_control(&motor_shield_v1, MS_V1, M1, 0); // Stop Motor M1
            servo_control(&servo[S2], 20, ANGLE, true); // Set Servo S2 to 20 degrees
            myDelay.Start(&myDelay, 100); // Start a non-blocking delay of 100 ms to ensure the servo reaches the scanning position
        } else {
            ms_motor_control(&motor_shield_v1, MS_V1, M1, -1000); // Move the carriage backward
        }
    } else {
        if (myDelay.IsExpired(&myDelay)) { // Check if the delay has expired
            roboticArmState = STATE_WAIT_FOR_RPI; // Transition to the next state
        }
    }
}

void HandleWaitForRpiState(void)
{
    if (!myDelay.active) {
        defect_result_received = false; // Reset the defect result flag
        printf("{\"request\":\"scan_defect\"}"); // Send scan request to RPi
        myDelay.Start(&myDelay, 1000); // Start a non-blocking delay to wait for the RPi response
    } else {
        if (myDelay.IsExpired(&myDelay)) { // Check if the delay has expired
            roboticArmState = STATE_SORT_AND_DROP; // Transition to the next state
        }
    }
}

void HandleSortAndDropState(void)
{
    if (defect_result_received) { // Check if the defect result is received
        if (!myDelay.active) {
            servo_control(&servo[S1], defect_result ? -65 : 65, ANGLE, true); // Sort Shuttlecock by setting Servo S1 to -65 or 65 degrees
            servo_control(&servo[S3], 40, ANGLE, true); // Open the gripper to drop the shuttlecock
            myDelay.Start(&myDelay, 1000); // Start a non-blocking delay of 1000 ms to ensure the shuttlecock is dropped
        } else {
            if (myDelay.IsExpired(&myDelay)) { // Check if the delay has expired
                roboticArmState = STATE_CHECK_STORAGE; // Transition to the next state
            }
        }
    }
}

void HandleCheckStorageState(void)
{
    if (!myDelay.active) {
        if (CheckBucketFull(defect_result)) { // Check if the bucket is full
            roboticArmState = STATE_ROTATE_AND_CHECK; // Transition to the next state
        } else {
            ms_motor_control(&motor_shield_v2, MS_V2, M2, defect_result ? 1000 : -1000); // Push the shuttlecock into the cylinder
            myDelay.Start(&myDelay, 1000); // Start a non-blocking delay to ensure the shuttlecock is pushed
        }
    } else {
        if (myDelay.IsExpired(&myDelay)) { // Check if the delay has expired
            roboticArmState = STATE_INIT; // Transition to the initial state
        }
    }
}

void HandleRotateAndCheckState(void)
{
    if (!myDelay.active) {
        RotateWheel(defect_result); // Rotate the wheel to the next cylinder
        myDelay.Start(&myDelay, 1000); // Start a non-blocking delay to ensure the wheel rotation
    } else {
        if (myDelay.IsExpired(&myDelay)) { // Check if the delay has expired
            roboticArmState = STATE_CHECK_STORAGE; // Transition to the next state
        }
    }
}

void HandleStopState(void)
{
    all_pwm_stop(); // Stop all PWM signals
    ms_motor_control(&motor_shield_v1, MS_V1, M1, 0); // Stop Motor M1
    ms_motor_control(&motor_shield_v1, MS_V1, M2, 0); // Stop Motor M2
    ms_motor_control(&motor_shield_v1, MS_V1, M3, 0); // Stop Motor M3
    ms_motor_control(&motor_shield_v1, MS_V1, M4, 0); // Stop Motor M4
}
