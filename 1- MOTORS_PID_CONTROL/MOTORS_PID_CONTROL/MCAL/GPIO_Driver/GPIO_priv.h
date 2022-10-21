/*
 * GPIO_priv.h
 *
 *  Created on: ??þ/??þ/????
 *      Author: Mohamed Alaa
 */

#ifndef GPIO_PRIV_H_
#define GPIO_PRIV_H_

typedef struct			
{
	u8 PIN  ;
	u8 DDR  ;
	u8 PORT ;

}REGISTER_t;


#define PORTB_REG        ((volatile REGISTER_t*)0x23)
#define PORTC_REG        ((volatile REGISTER_t*)0x26)
#define PORTD_REG        ((volatile REGISTER_t*)0x29)

volatile REGISTER_t*     PGPIO_PRegister_t_GetPtrToReg(enum GPIOPORT_t copy_PORT_ID);

#endif /* GPIO_PRIV_H_ */
