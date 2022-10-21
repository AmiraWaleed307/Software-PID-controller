/*
 * MOTORS_PID_CONTROL_prog.c
 *
 * Created: 03/10/2021 04:09:44 PM
 *  Author: Toothless XII
 */ 

#include "../../../../../1- LIB/STD_types.h"
#include "../../../../../1- LIB/BIT_math.h"
#include "../../MCAL/Clock_Driver/CLOCK_interface.h"
#include "../../MCAL/GPIO_Driver/GPIO_int.h"
#include "../../MCAL/INTERRUPTS_Driver/INTERRUPT.h"
#include "../../MCAL/Timer Driver/TIM_int.h"
#include "../../MCAL/ADC_Driver/ADC_int.h"
#include "MOTORS_PID_CONTROL_int.h"
#include "MOTORS_PID_CONTROL_config.h"
#include "MOTORS_PID_CONTROL_priv.h"

void HMotorPidControl_void_CountTimeUs (void)
{
	PCurrT = PCurrT + 1;
}

void HMotorPidControl_void_PIDinitialize(void)
{
	MTIM8_void_SetPeriodicFunction ( TIM2 , 1 , HMotorPidControl_void_CountTimeUs );
	PReference_Reading_1 = MADC_u16_ReadChannelSynch(ADC_1);
	PReference_Reading_2 = MADC_u16_ReadChannelSynch(ADC_2);
	MGPIO_void_SetPinDirection (GPIO_PORTB , 4 , GPIO_OUTPUT);
	MGPIO_void_SetPinDirection (GPIO_PORTB , 5 , GPIO_OUTPUT);
	MGPIO_void_SetPinDirection (GPIO_PORTB , 2 , GPIO_OUTPUT);
	MGPIO_void_SetPinDirection (GPIO_PORTB , 3 , GPIO_OUTPUT);	
}

void HMotorPidControl_void_SetMotorDirection (enum MOTOR_Direction_t Direction)
{
	switch (Direction)
	{
		case MOTOR_Forward : 
			MGPIO_void_SetPinValue (GPIO_PORTB , 4 , GPIO_HIGH);
			MGPIO_void_SetPinValue (GPIO_PORTB , 5 , GPIO_LOW);
			MGPIO_void_SetPinValue (GPIO_PORTB , 2 , GPIO_HIGH);
			MGPIO_void_SetPinValue (GPIO_PORTB , 3 , GPIO_LOW);
			break;
		case MOTOR_Right :
			MGPIO_void_SetPinValue (GPIO_PORTB , 4 , GPIO_LOW);
			MGPIO_void_SetPinValue (GPIO_PORTB , 5 , GPIO_HIGH);
			MGPIO_void_SetPinValue (GPIO_PORTB , 2 , GPIO_HIGH);
			MGPIO_void_SetPinValue (GPIO_PORTB , 3 , GPIO_LOW);
			break;
		case MOTOR_Left :
			MGPIO_void_SetPinValue (GPIO_PORTB , 4 , GPIO_HIGH);
			MGPIO_void_SetPinValue (GPIO_PORTB , 5 , GPIO_LOW);
			MGPIO_void_SetPinValue (GPIO_PORTB , 2 , GPIO_LOW);
			MGPIO_void_SetPinValue (GPIO_PORTB , 3 , GPIO_HIGH);
			break;
	}
}

void HMotorPidControl_void_RotateRobot (enum MOTOR_Movement_t Movement)
{
		u32	Local_CurrentTime = PCurrT;
		u16 Local_Reading_1 = 0;
		u16 Local_Reading_2 = 0;
		f32 Local_CurrentPos_1 = 0;
		f32 Local_CurrentPos_2 = 0;
		f32 Local_Target = 0;
		f32 Local_u_1 = 0;
		f32 Local_u_2 = 0;
		
		switch (Movement)
		{
			case TURN_90_RIGHT :
				Local_Target = Wheel_Circum / 4;
				break;
			case TURN_90_LEFT :
				Local_Target = Wheel_Circum / 4;
				break;
			case TURN_180 :
				Local_Target = Wheel_Circum / 2;
				break;
			case TURN_45_RIGHT :
				Local_Target = Wheel_Circum / 8;
				break;
			case TURN_45_LEFT :
				Local_Target = Wheel_Circum / 8;
				break;	
			default:
				return;
		}
		f32 error_1 = Local_Target;
		f32 error_2 = Local_Target;
		while (error_1 !=0 || error_2 !=0 )
		{
			
			Local_Reading_1 = MADC_u16_ReadChannelSynch(ADC_1);
			Local_Reading_2 = MADC_u16_ReadChannelSynch(ADC_2);
			switch (Movement)
			{
				case TURN_90_RIGHT :
					Local_CurrentPos_1 = Wheel_Circum - (Local_Reading_1-PReference_Reading_1) * Wheel_Circum / H_ADC_Res;
					Local_CurrentPos_2 = (Local_Reading_2-PReference_Reading_2) * Wheel_Circum / H_ADC_Res;
					break;
				case TURN_90_LEFT :
					Local_CurrentPos_1 = (Local_Reading_1-PReference_Reading_1) * Wheel_Circum / H_ADC_Res;
					Local_CurrentPos_2 = Wheel_Circum - (Local_Reading_2-PReference_Reading_2) * Wheel_Circum / H_ADC_Res;
					break;
				case TURN_180 :
					Local_CurrentPos_1 = Wheel_Circum - (Local_Reading_1-PReference_Reading_1) * Wheel_Circum / H_ADC_Res;
					Local_CurrentPos_2 = (Local_Reading_2-PReference_Reading_2) * Wheel_Circum / H_ADC_Res;
					break;
				case TURN_45_RIGHT :
					Local_CurrentPos_1 = Wheel_Circum - (Local_Reading_1-PReference_Reading_1) * Wheel_Circum / H_ADC_Res;
					Local_CurrentPos_2 = (Local_Reading_2-PReference_Reading_2) * Wheel_Circum / H_ADC_Res;
					break;
				case TURN_45_LEFT :
					Local_CurrentPos_1 = (Local_Reading_1-PReference_Reading_1) * Wheel_Circum / H_ADC_Res;
					Local_CurrentPos_2 = Wheel_Circum - (Local_Reading_2-PReference_Reading_2) * Wheel_Circum / H_ADC_Res;
					break;
				default:
					return;
			}

			error_1 = Local_Target - Local_CurrentPos_1;
			error_2 = Local_Target - Local_CurrentPos_2;
			f32 DeltaTime = Local_CurrentTime - PprevT;
			f32 dedt_1 = (error_1 - Peprev_1)/DeltaTime;
			f32 dedt_2 = (error_2 - Peprev_2)/DeltaTime;
			Peinteg_1 = Peinteg_1 + (error_1 * dedt_1);
			Peinteg_2 = Peinteg_2 + (error_2 * dedt_2);
			Local_u_1 = Kp_1*error_1 + Kd_1*dedt_1 + Ki_1*Peinteg_1;
			Local_u_2 = Kp_2*error_2 + Kd_2*dedt_2 + Ki_2*Peinteg_2;
			if (Local_u_1 < 0)
			Local_u_1 = Local_u_1 * (-1);
			if (Local_u_2 < 0)
			Local_u_2 = Local_u_2 * (-1);
			if (Local_u_1 > 255)
			Local_u_1 = 255;
			if (Local_u_2 > 255)
			Local_u_2 = 255;
			MTIM8_void_SetDutyCycle(TIM0, TIM_CHANNEL_A, Local_u_1);
			MTIM8_void_SetDutyCycle(TIM0, TIM_CHANNEL_B, Local_u_2);
			PprevT = Local_CurrentTime;
			Peprev_1 = error_1;
			Peprev_2 = error_2;
		}
		
	
}

void HMotorPidControl_void_MoveRobotForward ( u8 Copy_u8_CellsNumber)
{
		u32	Local_CurrentTime = PCurrT;
		u16 Local_Reading_1 = 0;
		u16 Local_Reading_2 = 0;
		f32 Local_CurrentPos_1 = 0;
		f32 Local_CurrentPos_2 = 0;
		f32 Local_Target = Cell_Length * Copy_u8_CellsNumber;
		f32 Local_u_1 = 0;
		f32 Local_u_2 = 0;
		f32 error_1 = Local_Target;
		f32 error_2 = Local_Target;
		while (error_1 !=0 || error_2 !=0 )
		{
			Local_Reading_1 = MADC_u16_ReadChannelSynch(ADC_1);
			Local_Reading_2 = MADC_u16_ReadChannelSynch(ADC_2);
			Local_CurrentPos_1 = (Local_Reading_1-PReference_Reading_1) * Wheel_Circum / H_ADC_Res;
			Local_CurrentPos_2 = (Local_Reading_2-PReference_Reading_2) * Wheel_Circum / H_ADC_Res;
			error_1 = Local_Target - Local_CurrentPos_1;
			error_2 = Local_Target - Local_CurrentPos_2;
			f32 DeltaTime = Local_CurrentTime - PprevT;
			f32 dedt_1 = (error_1 - Peprev_1)/DeltaTime;
			f32 dedt_2 = (error_2 - Peprev_2)/DeltaTime;
			Peinteg_1 = Peinteg_1 + (error_1 * dedt_1);
			Peinteg_2 = Peinteg_2 + (error_2 * dedt_2);
			Local_u_1 = Kp_1*error_1 + Kd_1*dedt_1 + Ki_1*Peinteg_1;
			Local_u_2 = Kp_2*error_2 + Kd_2*dedt_2 + Ki_2*Peinteg_2;
			if (Local_u_1 < 0)
				Local_u_1 = Local_u_1 * (-1);
			if (Local_u_2 < 0)
				Local_u_2 = Local_u_2 * (-1);
			if (Local_u_1 > 255)
				Local_u_1 = 255;
			if (Local_u_2 > 255)
				Local_u_2 = 255;
			MTIM8_void_SetDutyCycle(TIM0, TIM_CHANNEL_A, Local_u_1);
			MTIM8_void_SetDutyCycle(TIM0, TIM_CHANNEL_B, Local_u_2);
			PprevT = Local_CurrentTime;
			Peprev_1 = error_1;
			Peprev_2 = error_2;
		}
		
		
}

void HMotorPidControl_void_MoveRobot (enum MOTOR_Movement_t Movement , u8 Copy_u8_CellsNumber)
{	
	switch (Movement)
	{
		case FORWARD_DIRECTION :
			HMotorPidControl_void_SetMotorDirection (MOTOR_Forward);
			HMotorPidControl_void_MoveRobotForward (Copy_u8_CellsNumber);
			break;
			
		case TURN_90_RIGHT :
			HMotorPidControl_void_SetMotorDirection (MOTOR_Right);
			HMotorPidControl_void_RotateRobot (TURN_90_RIGHT);
			HMotorPidControl_void_MoveRobotForward (Copy_u8_CellsNumber);
			break;
			
		case TURN_90_LEFT :
			HMotorPidControl_void_SetMotorDirection (MOTOR_Left);
			HMotorPidControl_void_RotateRobot (TURN_90_LEFT);
			HMotorPidControl_void_MoveRobotForward (Copy_u8_CellsNumber);			
			break;
		
		case TURN_180 :
			HMotorPidControl_void_SetMotorDirection (MOTOR_Right);
			HMotorPidControl_void_RotateRobot (TURN_180);
			HMotorPidControl_void_MoveRobotForward (Copy_u8_CellsNumber);			
			break;
		
		case TURN_45_RIGHT :
			HMotorPidControl_void_SetMotorDirection (MOTOR_Right);
			HMotorPidControl_void_RotateRobot (TURN_45_RIGHT);
			HMotorPidControl_void_MoveRobotForward (Copy_u8_CellsNumber);
			break;
		
		case TURN_45_LEFT :
			HMotorPidControl_void_SetMotorDirection (MOTOR_Left);
			HMotorPidControl_void_RotateRobot (TURN_45_LEFT);
			HMotorPidControl_void_MoveRobotForward (Copy_u8_CellsNumber);			
			break;	
	}
	
}

void HMotorPidControl_void_StopRobot (void)
{
	MGPIO_void_SetPinValue (GPIO_PORTB , 4 , GPIO_LOW);
	MGPIO_void_SetPinValue (GPIO_PORTB , 5 , GPIO_LOW);
	MGPIO_void_SetPinValue (GPIO_PORTB , 2 , GPIO_LOW);
	MGPIO_void_SetPinValue (GPIO_PORTB , 3 , GPIO_LOW);
}
