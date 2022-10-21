/*
 * GPIO_prog.c
 *
 *  Created on: ??þ/??þ/????
 *      Author: Mohamed Alaa
 */
#include "../../../../../1- LIB/STD_types.h"
#include "../../../../../1- LIB/BIT_math.h"
#include "GPIO_int.h"
#include "GPIO_priv.h"



volatile REGISTER_t*     PGPIO_PRegister_t_GetPtrToReg(enum GPIOPORT_t copy_PORT_ID)
{
	switch (copy_PORT_ID)
	{
	case GPIO_PORTB :
		return PORTB_REG;
		break;
	case GPIO_PORTC :
		return PORTC_REG;
		break;
	case GPIO_PORTD :
		return PORTD_REG;
		break;
	default:
		return 0 ;
	}

}

void MGPIO_void_SetPinDirection (enum GPIOPORT_t PortID, u8 PinNumber, enum GPIODIRECTION_t Direction)
{
	volatile REGISTER_t* local_PReg =0;
	local_PReg = PGPIO_PRegister_t_GetPtrToReg(PortID);
	if (local_PReg == 0)
		return;
	switch (Direction)
	{
		case  GPIO_INPUT:
			CLR_BIT(local_PReg->DDR,PinNumber);
			CLR_BIT(local_PReg->PORT,PinNumber);
			break;
		case  GPIO_OUTPUT:
			SET_BIT(local_PReg->DDR,PinNumber);
			CLR_BIT(local_PReg->PORT,PinNumber);
			break;
		case  GPIO_INPUT_PULLUP:
			CLR_BIT(local_PReg->DDR,PinNumber);
			SET_BIT(local_PReg->PORT,PinNumber);
			break;

	}
	}
void MGPIO_void_SetPortDirection (enum GPIOPORT_t PortID, enum GPIODIRECTION_t Direction)
{
	volatile REGISTER_t* local_PReg =0;
	local_PReg = PGPIO_PRegister_t_GetPtrToReg(PortID);
	if (local_PReg == 0)
		return;
	switch (Direction)
	{
		case  GPIO_INPUT:
			CLR_REG(local_PReg->DDR);
			CLR_REG(local_PReg->PORT);
			break;
		case  GPIO_OUTPUT:
			SET_REG(local_PReg->DDR);
			CLR_REG(local_PReg->PORT);
			break;
		case  GPIO_INPUT_PULLUP:
			CLR_REG(local_PReg->DDR);
			SET_REG(local_PReg->PORT);
			break;

	}
	}

void MGPIO_void_SetPinValue (enum GPIOPORT_t PortID, u8 PinNumber, enum GPIOSTATE_t Value)
{
	volatile REGISTER_t* local_PReg =0;
	local_PReg = PGPIO_PRegister_t_GetPtrToReg(PortID);
	if (local_PReg == 0)
		return;
	switch (Value)
	{
		case  GPIO_LOW:
			CLR_BIT(local_PReg->PORT,PinNumber);
			break;
		case  GPIO_HIGH:
			SET_BIT(local_PReg->PORT,PinNumber);
			break;

	}

	}
void MGPIO_void_SetPortValue (enum GPIOPORT_t PortID, u8 Value)
{
	volatile REGISTER_t* local_PReg =0;
		local_PReg = PGPIO_PRegister_t_GetPtrToReg(PortID);
		if (local_PReg == 0)
			return;
		local_PReg->PORT=Value;
	}

void MGPIO_void_TogglePin (enum GPIOPORT_t PortID, u8 PinNumber)
{
	volatile REGISTER_t* local_PReg =0;
	local_PReg = PGPIO_PRegister_t_GetPtrToReg(PortID);
	if (local_PReg == 0)
		return;
	TOG_BIT(local_PReg->PORT,PinNumber);

	}
void MGPIO_void_TogglePort (enum GPIOPORT_t PortID)
{
	volatile REGISTER_t* local_PReg =0;
		local_PReg = PGPIO_PRegister_t_GetPtrToReg(PortID);
		if (local_PReg == 0)
			return;
		TOG_REG(local_PReg->PORT);
	}

u8 MGPIO_void_GetPinValue (enum GPIOPORT_t PortID, u8 PinNumber)
{
	volatile REGISTER_t* local_PReg =0;
	local_PReg = PGPIO_PRegister_t_GetPtrToReg(PortID);
	if (local_PReg == 0)
		return 0;
	return (GET_BIT(local_PReg->PIN,PinNumber));

	}
u8 MGPIO_void_GetPortValue (enum GPIOPORT_t PortID)
{
	volatile REGISTER_t* local_PReg =0;
	local_PReg = PGPIO_PRegister_t_GetPtrToReg(PortID);
	if (local_PReg == 0)
		return 0;
	return (local_PReg->PIN);

	}
