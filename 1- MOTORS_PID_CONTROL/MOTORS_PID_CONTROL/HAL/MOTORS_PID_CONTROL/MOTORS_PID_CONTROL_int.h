/*
 * MOTORS_PID_CONTROL_int.h
 *
 * Created: 03/10/2021 04:09:13 PM
 *  Author: Toothless XII
 */ 


#ifndef MOTORS_PID_CONTROL_INT_H_
#define MOTORS_PID_CONTROL_INT_H_

enum MOTOR_Movement_t { FORWARD_DIRECTION , TURN_90_RIGHT , TURN_90_LEFT , TURN_180 , TURN_45_RIGHT , TURN_45_LEFT };
enum MOTOR_Direction_t { MOTOR_Forward , MOTOR_Right , MOTOR_Left };

void HMotorPidControl_void_PIDinitialize(void);
void HMotorPidControl_void_SetMotorDirection (enum MOTOR_Direction_t Direction);
void HMotorPidControl_void_RotateRobot (enum MOTOR_Movement_t Movement);
void HMotorPidControl_void_MoveRobotForward ( u8 Copy_u8_CellsNumber);
void HMotorPidControl_void_MoveRobot (enum MOTOR_Movement_t Movement , u8 Copy_u8_CellsNumber);
void HMotorPidControl_void_StopRobot (void);

#endif /* MOTORS_PID_CONTROL_INT_H_ */