#include "robotic_arm.h"

NonBlockingDelay_TypeDef myDelay;

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

bool defect_result_received = false;
bool defect_result = false;


void UpdateRoboticArmState(void)
{
    switch (roboticArmState) {
    case STATE_INIT:
        myDelay = CreateNonBlockingDelay();
        // 初始化機械手臂到起始位置
        servo_control(&servo[S1], -3, ANGLE, true);
        servo_control(&servo[S2], 0, ANGLE, true);
        servo_control(&servo[S3], 40, ANGLE, true); // 打開夾子
        if (Button_IsPressed(&button[B2])) {
            ms_motor_control(&motor_shield_v1, MS_V1, M1, 0); // 停止 M1
            roboticArmState = STATE_MOVE_TO_GRAB;
        } else {
            ms_motor_control(&motor_shield_v1, MS_V1, M1, -1000);
        }
        break;

    case STATE_MOVE_TO_GRAB:
        if (Button_IsPressed(&button[B1])) {
            ms_motor_control(&motor_shield_v1, MS_V1, M1, 0); // 停止 M1
            myDelay.Start(&myDelay, 200); // 等待0.2秒
            roboticArmState = STATE_GRAB_SHUTTLECOCK;
        } else {
            ms_motor_control(&motor_shield_v1, MS_V1, M1, 1000);
        }
        break;

    case STATE_GRAB_SHUTTLECOCK:
        if (myDelay.IsExpired(&myDelay)) {
            servo_control(&servo[S3], 0, ANGLE, true); // 合上夾子
            myDelay.Start(&myDelay, 100);  // 等待0.1秒
            roboticArmState = STATE_MOVE_TO_SCAN;
        }
        break;

    case STATE_MOVE_TO_SCAN:
        if (myDelay.IsExpired(&myDelay)) {
            //pwm_stop(&servo[S3]);
            if (Button_IsPressed(&button[B2])) {
                ms_motor_control(&motor_shield_v1, MS_V1, M1, 0); // 停止 M1
                roboticArmState = STATE_POSITION_FOR_SCANNING;
            } else {
                ms_motor_control(&motor_shield_v1, MS_V1, M1, -1000);
            }
        }
        break;

    case STATE_POSITION_FOR_SCANNING:
        servo_control(&servo[S2], 20, ANGLE, true); // 設置 S2 到掃描角度
        myDelay.Start(&myDelay, 100); // 等待0.1秒
        roboticArmState = STATE_REQUEST_FOR_SCANNING;
        break;

    case STATE_REQUEST_FOR_SCANNING:
        if (myDelay.IsExpired(&myDelay)) {
            defect_result_received = false;
            printf("{\"request\":\"scan_defect\"}"); // Send scan request
            roboticArmState = STATE_WAIT_FOR_DEFECT_RESULT;
        }
        break;

    case STATE_WAIT_FOR_DEFECT_RESULT:
        if (defect_result_received) {
            roboticArmState = STATE_PLACE_SHUTTLECOCK;
        }
        break;

    case STATE_PLACE_SHUTTLECOCK:
        // 檢查是否已經觸碰到B1或B2限位開關，如果是，則不啟動M1馬達
        if (!Button_IsPressed(&button[B1])) {
            servo_control(&servo[S2], 90, ANGLE, true); // 設置 S2 到 90 度
            ms_motor_control(&motor_shield_v1, MS_V1, M1, 1000); // 正轉動 M1
        } else if (Button_IsPressed(&button[B1])) {
            ms_motor_control(&motor_shield_v1, MS_V1, M1, 0); // 停止 M1 馬達
            servo_control(&servo[S2], 180, ANGLE, true); // 設置 S2 到 180 度
            myDelay.Start(&myDelay, 1000); // 等待1秒
        }

        // 根據 defect_result 設置 M2 的初始轉動
        if ((defect_result && !Button_IsPressed(&button[B4]))|| (!defect_result && !Button_IsPressed(&button[B3]))) {
            ms_motor_control(&motor_shield_v1, MS_V1, M2, (defect_result * 2 - 1) * -1000);
        } else if((defect_result && Button_IsPressed(&button[B4]))|| (!defect_result && Button_IsPressed(&button[B3]))) {
            ms_motor_control(&motor_shield_v1, MS_V1, M2, 0);
            if (Button_IsPressed(&button[B1])) {
                roboticArmState = STATE_KEEP_PLACEMENT;
            }
        }
        break;

    case STATE_KEEP_PLACEMENT:
        if (myDelay.IsExpired(&myDelay)) {
            if (defect_result && Button_IsPressed(&button[B4])) { // 如果是好羽毛球且B4被按下
                servo_control(&servo[S1], -85, ANGLE, true); // S1往左擺動
                // 啟動新的延遲以確保S1擺動完成
                myDelay.Start(&myDelay, 1000);
                roboticArmState = STATE_FINISH_PLACEMENT;
            } else if (!defect_result && Button_IsPressed(&button[B3])) { // 如果是壞羽毛球且B3被按下
                servo_control(&servo[S1], 65, ANGLE, true); // S1往右擺動
                // 啟動新的延遲以確保S1擺動完成
                myDelay.Start(&myDelay, 1000);
                roboticArmState = STATE_FINISH_PLACEMENT;
            }
        }
        break;


    case STATE_FINISH_PLACEMENT:
        if (myDelay.IsExpired(&myDelay)) {
            servo_control(&servo[S3], 10, ANGLE, true); // 鬆開夾子
            myDelay.Start(&myDelay, 1000);
            roboticArmState = STATE_SORT_SHUTTLECOCK;
        }
        break;

    case STATE_SORT_SHUTTLECOCK:
        if (myDelay.IsExpired(&myDelay)) {
            servo_control(&servo[S1], -3, ANGLE, true); // S1回到起始位置
            servo_control(&servo[S2], 0, ANGLE, true); // S2回到起始位置
            servo_control(&servo[S3], 40, ANGLE, true); // 打開夾子
            myDelay.Start(&myDelay, 1000);
            roboticArmState = STATE_IDLE;
        }

        /*
            if ((defect_result && Button_IsPressed(&button[B4])) || (!defect_result && Button_IsPressed(&button[B3]))) {
                if (CheckBucketFull(defect_result ? LEFT : RIGHT)) {
                    if (defect_result) {
                        RotateLeftWheel();
                    } else {
                        RotateRightWheel();
                    }
                } else {
                    ms_motor_control(&motor_shield_v2, MS_V2, M2, defect_result ? 1000 : -1000); // 正轉或反轉 M2 推球進羽球筒
                    roboticArmState = STATE_INIT;
                }
            }
                */
        break;

    case STATE_IDLE:
        if (myDelay.IsExpired(&myDelay)) {
            all_pwm_stop();
            ms_motor_control(&motor_shield_v1, MS_V1, M1, 0);
            ms_motor_control(&motor_shield_v1, MS_V1, M2, 0);
            ms_motor_control(&motor_shield_v1, MS_V1, M3, 0);
            ms_motor_control(&motor_shield_v1, MS_V1, M4, 0);
        }
        
        break;

    default:
        break;
    }
}