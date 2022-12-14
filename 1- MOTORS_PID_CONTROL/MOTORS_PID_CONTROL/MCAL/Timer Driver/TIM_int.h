

#ifndef TIM_INT_H_
#define TIM_INT_H_

enum TIM8_ID_t{TIM0,TIM2} ;
enum CHANNEL_ID{TIM_CHANNEL_A,TIM_CHANNEL_B} ;
enum TIM_MODE_OPTIONS {TIM_NORMAL_MODE,TIM_PWM_PHASECORRECT,TIM_COMPARE_MATCH,TIM_FAST_PWM
						,TIM_PWM_PHASECORRECT_TOP_OCRA=5 ,TIM_FAST_PWM_TOP_OCRA=7} ;
enum TIM_CTC_t {TIM_CTC_DISCONNECTED,TIM_CTC_TOGGLE,TIM_CTC_CLEAR,TIM_CTC_SET} ;
enum TIM_FAST_PWM_t {TIM_FAST_PWM_DISCONNECTED,TIM_FAST_PWM_TOGGLE,TIM_FAST_PWM_NON_INVERTING
					,TIM_FAST_PWM_INVERTING} ;
enum TIM_PHASE_CORRECT_t {TIM_PHASE_CORRECT_DISCONNECTED,TIM_PHASE_CORRECT_TOGGLE,TIM_PHASE_CORRECT_NON_INVERTING
						,TIM_PHASE_CORRECT_INVERTING} ;
enum TIM_CLOCK_SOURCE {TIM_CS_STOP ,TIM_CS_NO_PRESCALER,TIM_CS_8_PRESCALER,TIM_CS_64_PRESCALER,TIM_CS_256_PRESCALER
						,TIM_CS_1024_PRESCALER,TIM_CS_EXT_FALLING,TIM_CS_EXT_RISING} ;

enum TIM_INTERRUPT_t {TIM_OVF_INTERRUPT,TIM_COMA_INTERRUPT,TIM_COMB_INTERRUPT} ;



void MTIM8_void_initialize(enum TIM8_ID_t TIMER_ID,enum TIM_MODE_OPTIONS TIM_MODE, u8 ChannelA_MODE ,u8 ChannelB_MODE
							,enum TIM_CLOCK_SOURCE TIM_SOURCE );


u8 MTIM8_u8_GetCounter(enum TIM8_ID_t TIMER_ID) ;
void MTIM8_u8_SetCounter(enum TIM8_ID_t TIMER_ID,u8 preload) ;
void MTIM8_void_BusyWait(enum TIM8_ID_t TIMER_ID,u8 Ticks) ;


// period -> us
void MTIM8_void_SetPeriodicFunction(enum TIM8_ID_t TIMER_ID, u32 Period_us,void (*p_CallBack)(void));

void MTIM8_void_SetDutyCycle(enum TIM8_ID_t TIMER_ID,enum CHANNEL_ID CHANNEL,u8 Duty ) ;


void MTIM8_void_CTCSetFrequency(enum TIM8_ID_t TIMER_ID, u32 frequency );


void MTIM8_void_EnableInterrupt(enum TIM8_ID_t TIMER_ID ,enum TIM_INTERRUPT_t INTERRUPT_TYPE ) ;
void MTIM8_void_DisableInterrupt(enum TIM8_ID_t TIMER_ID ,enum TIM_INTERRUPT_t INTERRUPT_TYPE ) ;

void MTIM8_void_StartTimer(enum TIM8_ID_t TIMER_ID , u8 PreLoad) ;
void MTIM8_void_StopTimer(enum TIM8_ID_t TIMER_ID) ;

void MTIM8_void_SetCallBack(enum TIM8_ID_t TIMER_ID ,enum TIM_INTERRUPT_t INTERRUPT_TYPE,void (*p_CallBack)(void) );


u8	MTIM8_u8GetFlag(enum TIM8_ID_t TIMER_ID ,enum TIM_INTERRUPT_t INTERRUPT_TYPE);


#endif /* TIM_INT_H_ */
