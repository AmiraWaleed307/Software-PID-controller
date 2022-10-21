/*
 * INTERRUPT.h
 *
 * Created: 19/09/2021 03:58:37 PM
 *  Author: Toothless XII
 */ 


#ifndef INTERRUPT_H_
#define INTERRUPT_H_

#define	SREG						*((volatile u8*) 0x5F)
#define SREG_IBIT					7

#define INT_ENABLE_GLOBAL_INT		(SET_BIT(SREG,SREG_IBIT))
#define INT_DISABLE_GLOBAL_INT		(CLR_BIT(SREG,SREG_IBIT))



#endif /* INTERRUPT_H_ */