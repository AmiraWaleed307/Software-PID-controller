/*
 * ADC_priv.h
 *
 * Created: 20/09/2021 09:58:45 PM
 *  Author: Toothless XII
 */ 


#ifndef ADC_PRIV_H_
#define ADC_PRIV_H_

typedef	struct {
	u8 ADC_ADCL;
	u8 ADC_ADCH;
	u8 ADC_ADCSRA;
	u8 ADC_ADCSRB;
	u8 ADC_ADMUX;
	u8 RESERVED;
	u8 ADC_DIDR0;
	
} ADC_STRUCT;

#define ADC_REG					((volatile ADC_STRUCT*)0x78)
  
#define ADC_VECTOR_ISR_ADC		__vector_21

void ADC_VECTOR_ISR_ADC (void) __attribute__((signal,used));

void (*PADC_void_CallBack) (u16)= 0;

static u8 PADC_Conversion_Resolution = 0 ; 
static u8 PADC_FREE_RUNNING_MODE = 0;
#endif /* ADC_PRIV_H_ */