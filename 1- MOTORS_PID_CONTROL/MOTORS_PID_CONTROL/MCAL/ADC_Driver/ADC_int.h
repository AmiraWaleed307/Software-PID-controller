/*
 * ADC_int.h
 *
 * Created: 20/09/2021 09:58:27 PM
 *  Author: Toothless XII
 */ 


#ifndef ADC_INT_H_
#define ADC_INT_H_ 

enum ADC_VOLT_REF {ADC_AREF , ADC_AVCC , ADC_INTERNAL_1V1 = 3 };
enum ADC_CHANNEL { ADC_0 , ADC_1 , ADC_2 , ADC_3 , ADC_4 , ADC_5 , ADC_6 , ADC_7 , ADC_TEMP_SEN , ADC_VBG_1V1 = 14 , ADC_GND };
enum ADC_PRE {ACD_2_PRE = 1 , ACD_4_PRE , ACD_8_PRE , ACD_16_PRE , ACD_32_PRE , ACD_64_PRE , ACD_128_PRE};
enum ADC_MODE_t { ADC_MODE_SINGLE_CONVERSION , ADC_MODE_CONTINOUS_CONVERSION };
enum ADC_TRIG_SRC_t {ADC_FREE_RUNNING , ADC_ANALOG_COMP , ADC_EXTI0 , TIM0A_COMP , TIM0_OVF , TIM1B_COMP , TIM1_OVF , TIM1_CIU};
enum ADC_RESOLUTION { ADC_10BIT , ADC_8BIT };

void MADC_void_Initialize ( enum ADC_MODE_t Mode , enum ADC_VOLT_REF Voltage , enum ADC_PRE Prescaler , enum ADC_RESOLUTION Resolution );

u16 MADC_u16_ReadChannelSynch ( enum ADC_CHANNEL Channel );

void MADC_void_ReadChannelAsynch(enum ADC_CHANNEL Channel, void (*CallBack)(u16));

void MADC_void_StartContinousMode (void);
void MADC_void_StopContinousMode  (void);

void MADC_void_SetAutoTriggerSource (enum ADC_TRIG_SRC_t TriggerSource);

void MADC_void_EnableDigitalInputBuffer  ( enum ADC_CHANNEL Channel );
void MADC_void_DisableDigitalInputBuffer ( enum ADC_CHANNEL Channel );

#endif /* ADC_INT_H_ */