
#include "../../../../../1- LIB/STD_types.h"
#include "../../../../../1- LIB/BIT_math.h"
#include "TIM_int.h"
#include "TIM_priv.h"
#include "../INTERRUPTS_Driver/INTERRUPT.h"
#include "../Clock_Driver/CLOCK_interface.h"


volatile TIM_8_REG_t* PTIM8_GetTimPointer(enum TIM8_ID_t TIMER_ID)
{
	switch (TIMER_ID)
	{

	case TIM0 :
		return TIMER0_REG ;
		break ;
	case TIM2 :
		return TIMER2_REG ;
		break ;

	default  :
		return  0 ;

	}
}


void MTIM8_void_initialize(enum TIM8_ID_t TIMER_ID,enum TIM_MODE_OPTIONS TIM_MODE, u8 ChannelA_MODE ,u8 ChannelB_MODE
							,enum TIM_CLOCK_SOURCE TIM_SOURCE )
{

	if (TIMER_ID > NUM_8BIT_TIMERS-1)
			return ;
	volatile TIM_8_REG_t* PTR_TIM_REG = PTIM8_GetTimPointer(TIMER_ID) ;

	CLR_REG(PTR_TIM_REG->TCCRA);
	CLR_REG(PTR_TIM_REG->TCCRB);
	PTR_TIM_REG->TCCRA |= ChannelA_MODE<<6 | ChannelB_MODE<<4 |(TIM_MODE&3) ;
	PTR_TIM_REG->TCCRB |= (GET_BIT(TIM_MODE,2))<<3 | TIM_SOURCE ;
	p_8PreScalers[TIMER_ID] = TIM_SOURCE ;
	p_8Mode[TIMER_ID] = TIM_MODE  ;


}


u8 MTIM8_u8_GetCounter(enum TIM8_ID_t TIMER_ID)
{
	if (TIMER_ID > NUM_8BIT_TIMERS-1)
				return 0 ;
	volatile TIM_8_REG_t* PTR_TIM_REG = PTIM8_GetTimPointer(TIMER_ID) ;

	return (PTR_TIM_REG->TCNT) ;
}


void MTIM8_u8_SetCounter(enum TIM8_ID_t TIMER_ID,u8 preload)
{
	if (TIMER_ID > NUM_8BIT_TIMERS-1)
					return  ;
	volatile TIM_8_REG_t* PTR_TIM_REG = PTIM8_GetTimPointer(TIMER_ID) ;

	PTR_TIM_REG->TCNT = preload ;
}

void MTIM8_void_BusyWait(enum TIM8_ID_t TIMER_ID,u8 Ticks)
{
	if (TIMER_ID > NUM_8BIT_TIMERS-1)
		return  ;
	volatile TIM_8_REG_t* PTR_TIM_REG = PTIM8_GetTimPointer(TIMER_ID) ;

	PTR_TIM_REG->TCNT = 0 ;

	while(PTR_TIM_REG->TCNT < Ticks);

}


// period -> us
void MTIM8_void_SetPeriodicFunction(enum TIM8_ID_t TIMER_ID, u32 Period_us ,void (*p_CallBack)(void))
{
	if ((TIMER_ID > NUM_8BIT_TIMERS-1)||(p_8PreScalers[TIMER_ID]==TIM_CS_STOP)||(p_8PreScalers[TIMER_ID]>TIM_CS_1024_PRESCALER))
		return  ;
	if ( p_8Mode[TIMER_ID]==TIM_PWM_PHASECORRECT)
		Period_us=Period_us/2;
		
	volatile TIM_8_REG_t* PTR_TIM_REG = PTIM8_GetTimPointer(TIMER_ID) ;


	u32 Local_OverFlowNum =0 ;
	u8  Local_TicksNum =0 ;

	p_8OverFlowCallBack[TIMER_ID]  = p_CallBack ;

	f32 OverFlowTime = 256 * p_PreScalerValues[p_8PreScalers[TIMER_ID]] /(MCLOCK_u32GetSystemClock()/1000000);

	Local_OverFlowNum = (f32)Period_us /  OverFlowTime ;
	Local_TicksNum = 256* (((f32)Period_us/OverFlowTime)-Local_OverFlowNum) ;
	if (Local_TicksNum>0)
	{
		Local_OverFlowNum ++ ;
		PTR_TIM_REG->TCNT = 255 -Local_TicksNum ;
		p_8PeriodicInterrupt_Preload[TIMER_ID] = 255-Local_TicksNum ;
	}

	p_8PeriodicInterrupt_OVF_NUM[TIMER_ID] = Local_OverFlowNum ;


	switch (TIMER_ID)
	{
		case TIM0 :
			SET_BIT(*TIMSK0_REG,0) ;
			INT_ENABLE_GLOBAL_INT;
			break ;
		case TIM2 :
			SET_BIT(*TIMSK2_REG,0) ;
			INT_ENABLE_GLOBAL_INT;
			break ;
	}

}

void MTIM8_void_SetDutyCycle(enum TIM8_ID_t TIMER_ID,enum CHANNEL_ID CHANNEL,u8 Duty )
{
	if (TIMER_ID > NUM_8BIT_TIMERS-1)
					return  ;
	volatile TIM_8_REG_t* PTR_TIM_REG = PTIM8_GetTimPointer(TIMER_ID) ;
	switch (CHANNEL)
	{
	case TIM_CHANNEL_A:
		PTR_TIM_REG->OCRA = Duty ;
		break ;
	case TIM_CHANNEL_B :
		PTR_TIM_REG->OCRB = Duty ;
		break ;

	default :
		return ;
	}
}


void MTIM8_void_CTCSetFrequency(enum TIM8_ID_t TIMER_ID, u32 frequency )
{
	if (TIMER_ID > NUM_8BIT_TIMERS-1)
			return  ;
	volatile TIM_8_REG_t* PTR_TIM_REG = PTIM8_GetTimPointer(TIMER_ID) ;
	PTR_TIM_REG ->OCRA = ((float)MCLOCK_u32GetSystemClock()/(2.0* p_PreScalerValues[p_8PreScalers[TIMER_ID]]*frequency))-1 ;

}



void MTIM8_void_EnableInterrupt(enum TIM8_ID_t TIMER_ID ,enum TIM_INTERRUPT_t INTERRUPT_TYPE )
{
	switch (TIMER_ID)
	{

	case TIM0 :
		SET_BIT(*TIMSK0_REG,INTERRUPT_TYPE)  ;
		SET_BIT(*TIFR0_REG,INTERRUPT_TYPE)  ;
		break ;
	case TIM2 :
		SET_BIT(*TIMSK2_REG,INTERRUPT_TYPE)  ;
		SET_BIT(*TIFR2_REG,INTERRUPT_TYPE)  ;
		break ;

	default :
		return  ;

	}
	INT_ENABLE_GLOBAL_INT ;
}

void MTIM8_void_DisableInterrupt(enum TIM8_ID_t TIMER_ID ,enum TIM_INTERRUPT_t INTERRUPT_TYPE )
{
	switch (TIMER_ID)
	{
	case TIM0 :
		CLR_BIT(*TIMSK0_REG,INTERRUPT_TYPE)  ;
		break ;
	case TIM2 :
		CLR_BIT(*TIMSK2_REG,INTERRUPT_TYPE)  ;
		break ;
	default :
		return  ;
	}
}

void MTIM8_void_StartTimer(enum TIM8_ID_t TIMER_ID , u8 PreLoad)
{
	if (TIMER_ID > NUM_8BIT_TIMERS-1)
			return  ;
	volatile TIM_8_REG_t* PTR_TIM_REG = PTIM8_GetTimPointer(TIMER_ID) ;
	PTR_TIM_REG->TCNT = PreLoad ;
	PTR_TIM_REG->TCCRB |= p_8PreScalers[TIMER_ID] ;
	switch (TIMER_ID)
	{
		case TIM0 :
			*TIFR0_REG = 7;
			break;
		case TIM2:
			*TIFR2_REG = 7;
			break;
	}
}


void MTIM8_void_StopTimer(enum TIM8_ID_t TIMER_ID)
{
	if (TIMER_ID > NUM_8BIT_TIMERS-1)
			return  ;
	volatile TIM_8_REG_t* PTR_TIM_REG = PTIM8_GetTimPointer(TIMER_ID) ;
	PTR_TIM_REG->TCCRB &= ~7 ; // 0xF8 ->  1111 1000
}

void MTIM8_void_SetCallBack(enum TIM8_ID_t TIMER_ID ,enum TIM_INTERRUPT_t INTERRUPT_TYPE,void (*p_CallBack)(void) )
{
	switch (INTERRUPT_TYPE)
	{
	case TIM_OVF_INTERRUPT :
		p_8OverFlowCallBack[TIMER_ID]  = p_CallBack ;
		break ;
	case TIM_COMA_INTERRUPT :
		p_8OutCompareMatchACallBack[TIMER_ID] = p_CallBack ;
		break ;
	case TIM_COMB_INTERRUPT :
		p_8OutCompareMatchBCallBack[TIMER_ID]=p_CallBack ;
		break ;
	default :
		return ;
	}

}


u8	MTIM8_u8GetFlag(enum TIM8_ID_t TIMER_ID ,enum TIM_INTERRUPT_t INTERRUPT_TYPE)
{
	switch (TIMER_ID)
	{
	case TIM0 :
		return(GET_BIT(*TIFR0_REG,INTERRUPT_TYPE));
		break ;
	case TIM2 :
		return(GET_BIT(*TIFR2_REG,INTERRUPT_TYPE));
		break ;
	default :
		return 0 ;
	}
}

void TIM0_VECTOR_ISR_TIM0_COMPA  (void)
{
	if (p_8OutCompareMatchACallBack[TIM0]!=0)
	{
		p_8OutCompareMatchACallBack[TIM0]() ;
	}
}


void TIM0_VECTOR_ISR_TIM0_COMPB  (void)
{
	if (p_8OutCompareMatchBCallBack[TIM0]!=0)
	{
		p_8OutCompareMatchBCallBack[TIM0]() ;
	}
}


void TIM0_VECTOR_ISR_TIM0_OVF  (void)
{
	if (p_8OverFlowCallBack[TIM0]!=0)
	{
		p_8OVFCounter[TIM0] ++ ;
		if ((p_8PeriodicInterrupt_OVF_NUM[TIM0] == p_8OVFCounter[TIM0] ) || p_8PeriodicInterrupt_OVF_NUM[TIM0]==0 )
		{
		p_8OverFlowCallBack[TIM0]() ;
		p_8OVFCounter[TIM0] = 0 ;
		if(p_8PeriodicInterrupt_OVF_NUM[TIM0]!=0 && p_8Mode[TIM0]==TIM_NORMAL_MODE)
			TIMER0_REG->TCNT = p_8PeriodicInterrupt_Preload[TIM0] ;
		}
	}
}


void TIM2_VECTOR_ISR_TIM2_COMPA  (void)
{
	if (p_8OutCompareMatchACallBack[TIM2]!=0)
	{
		p_8OutCompareMatchACallBack[TIM2]() ;
	}

}


void TIM2_VECTOR_ISR_TIM2_COMPB  (void)
{
	if (p_8OutCompareMatchBCallBack[TIM2]!=0)
	{
		p_8OutCompareMatchBCallBack[TIM2]() ;
	}
}


void TIM2_VECTOR_ISR_TIM2_OVF  (void)
{
	if (p_8OverFlowCallBack[TIM2]!=0)
	{
		p_8OVFCounter[TIM2] ++ ;
		if ((p_8PeriodicInterrupt_OVF_NUM[TIM2] == p_8OVFCounter[TIM2] ) || p_8PeriodicInterrupt_OVF_NUM[TIM2]==0 )
		{
		p_8OverFlowCallBack[TIM2]() ;
		p_8OVFCounter[TIM2] = 0 ;
		if(p_8PeriodicInterrupt_OVF_NUM[TIM2]!=0 && p_8Mode[TIM2]==TIM_NORMAL_MODE)
			TIMER0_REG->TCNT = p_8PeriodicInterrupt_Preload[TIM2] ;
		}
	}
}

