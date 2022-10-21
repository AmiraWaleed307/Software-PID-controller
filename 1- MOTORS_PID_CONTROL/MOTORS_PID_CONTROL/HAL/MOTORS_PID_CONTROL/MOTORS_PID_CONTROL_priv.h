/*
 * MOTORS_PID_CONTROL_priv.h
 *
 * Created: 03/10/2021 05:16:51 PM
 *  Author: Toothless XII
 */ 


#ifndef MOTORS_PID_CONTROL_PRIV_H_
#define MOTORS_PID_CONTROL_PRIV_H_


static  u16	PReference_Reading_1 = 0;
static  u16	PReference_Reading_2 = 0;
static	f32 Peprev_1 = 0;
static	f32 Peprev_2 = 0;
static	u32	PCurrT = 0;
static	u32	PprevT = 0;
static	f32 Peinteg_1 = 0;
static	f32 Peinteg_2 = 0;

#endif /* MOTORS_PID_CONTROL_PRIV_H_ */