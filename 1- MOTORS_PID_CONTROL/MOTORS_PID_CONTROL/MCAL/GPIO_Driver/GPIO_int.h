/*
 * GPIO_int.h
 *
 *  Created on: ??þ/??þ/????
 *      Author: Mohamed Alaa
 */

#ifndef GPIO_INT_H_
#define GPIO_INT_H_
#define F_CPU           8000000UL
enum GPIOPORT_t {GPIO_PORTB,GPIO_PORTC,GPIO_PORTD};
enum GPIODIRECTION_t {GPIO_INPUT,GPIO_OUTPUT,GPIO_INPUT_PULLUP};
enum GPIOSTATE_t {GPIO_LOW,GPIO_HIGH};

void MGPIO_void_SetPinDirection (enum GPIOPORT_t PortID, u8 PinNumber, enum GPIODIRECTION_t Direction);
void MGPIO_void_SetPortDirection (enum GPIOPORT_t PortID, enum GPIODIRECTION_t Direction);

void MGPIO_void_SetPinValue (enum GPIOPORT_t PortID, u8 PinNumber, enum GPIOSTATE_t Value);
void MGPIO_void_SetPortValue (enum GPIOPORT_t PortID, u8 Value);

void MGPIO_void_TogglePin (enum GPIOPORT_t PortID, u8 PinNumber);
void MGPIO_void_TogglePort (enum GPIOPORT_t PortID);

u8 MGPIO_void_GetPinValue (enum GPIOPORT_t PortID, u8 PinNumber);
u8 MGPIO_void_GetPortValue (enum GPIOPORT_t PortID);

#endif /* GPIO_INT_H_ */
