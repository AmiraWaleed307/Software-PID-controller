

#ifndef TIM_PRIV_H_
#define TIM_PRIV_H_


#define NUM_8BIT_TIMERS		2
#define NUM_16BIT_TIMERS	1


typedef struct
{
	u8 TCCRA ;
	u8 TCCRB ;
	u8 TCNT ;
	u8 OCRA ;
	u8 OCRB ;
}TIM_8_REG_t;

#define TIMER0_REG 		((volatile TIM_8_REG_t*)0x44)
#define TIMER2_REG 		((volatile TIM_8_REG_t*)0xB0)

#define TIMSK0_REG		((volatile u8*)0x6E)
#define TIMSK2_REG		((volatile u8*)0x70)

#define TIFR0_REG		((volatile u8*)0x35)
#define TIFR2_REG		((volatile u8*)0x37)


u8 p_8PreScalers[NUM_8BIT_TIMERS] = {0} ;
u8 p_8PeriodicInterrupt_Preload[NUM_8BIT_TIMERS] = {0} ;
u32 p_8PeriodicInterrupt_OVF_NUM[NUM_8BIT_TIMERS] = {0} ;
u32 p_8OVFCounter[NUM_8BIT_TIMERS] = {0} ;
u8 p_8Mode[NUM_8BIT_TIMERS] = {0} ;

u32 p_PreScalerValues[6]={0,1,8,64,256,1024};


#define TIM0_VECTOR_ISR_TIM0_COMPA		__vector_14
#define TIM0_VECTOR_ISR_TIM0_COMPB		__vector_15
#define TIM0_VECTOR_ISR_TIM0_OVF		__vector_16

#define TIM2_VECTOR_ISR_TIM2_COMPA		__vector_7
#define TIM2_VECTOR_ISR_TIM2_COMPB		__vector_8
#define TIM2_VECTOR_ISR_TIM2_OVF		__vector_9

void TIM0_VECTOR_ISR_TIM0_COMPA  (void)   __attribute__((signal,used));
void TIM0_VECTOR_ISR_TIM0_COMPB  (void)   __attribute__((signal,used));
void TIM0_VECTOR_ISR_TIM0_OVF  (void)   __attribute__((signal,used));

void TIM2_VECTOR_ISR_TIM2_COMPA  (void)   __attribute__((signal,used));
void TIM2_VECTOR_ISR_TIM2_COMPB  (void)   __attribute__((signal,used));
void TIM2_VECTOR_ISR_TIM2_OVF  (void)   __attribute__((signal,used));


void (*p_8OverFlowCallBack[NUM_8BIT_TIMERS]) (void) = {0} ;
void (*p_8OutCompareMatchACallBack[NUM_8BIT_TIMERS]) (void) = {0} ;
void (*p_8OutCompareMatchBCallBack[NUM_8BIT_TIMERS]) (void) = {0} ;


volatile TIM_8_REG_t* PTIM8_GetTimPointer(enum TIM8_ID_t TIMER_ID);

#endif /* TIM_PRIV_H_ */
