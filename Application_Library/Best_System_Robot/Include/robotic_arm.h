/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ROBOTIC_ARM_H
#define ROBOTIC_ARM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Include */
#include "servo.h"
#include "sensor.h"
#include "timing_delays.h"

// Define an enum type to represent the state of the robotic arm
typedef enum {
    STATE_IDLE,
    STATE_INIT,
    STATE_GRAB_SHUTTLECOCK,
    STATE_MOVE_TO_SCAN,
    STATE_WAIT_FOR_SCAN,
    STATE_SORT_AND_DROP,
    STATE_STORE_SHUTTLECOCK,
    STATE_STOP
} RoboticArmState_TypeDef;


// Declare the global objects with extern keyword
extern RoboticArmState_TypeDef roboticArmState;

extern bool defect_result_received;
extern bool defect_result;


/* FUNCTION (Prototype) DEFINITIONS */
void CheckButtonsAndStopMotors(void);
void UpdateRoboticArmState(void);
void HandleIdleState(void);
void HandleInitState(void);
void HandleGrabShuttlecockState(void);
void HandleMoveToScanState(void);
void HandleWaitForScanState(void);
void HandleSortAndDropState(void);
void HandleStoreShuttlecockState(void);
void HandleStopState(void);

bool CheckBucketFull(bool defect_result);
void RotateWheel(bool defect_result);


#ifdef __cplusplus
}
#endif

#endif /* ROBOTIC_ARM_H */

/*****END OF FILE*****/
