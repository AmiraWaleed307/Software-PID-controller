/*
 * ADC_prog.c
 *
 * Created: 20/09/2021 09:59:03 PM
 *  Author: Toothless XII
 */ 

#include "../../../../../1- LIB/STD_types.h"
#include "../../../../../1- LIB/BIT_math.h"
#include "ADC_int.h"
#include "ADC_priv.h"
#include "../INTERRUPTS_Driver/INTERRUPT.h"

void MADC_void_Initialize ( enum ADC_MODE_t Mode , enum ADC_VOLT_REF Voltage , enum ADC_PRE Prescaler , enum ADC_RESOLUTION Resolution )
{
	PADC_Conversion_Resolution = Resolution;
	CLR_REG(ADC_REG->ADC_ADMUX);
	ADC_REG->ADC_ADMUX |= ((Voltage<<6) | (Resolution<<5) );
	
	CLR_REG(ADC_REG->ADC_ADCSRA);
	ADC_REG->ADC_ADCSRA |= Prescaler | (Mode<<5) | (Mode<<3) ;	
	if (Mode == ADC_MODE_CONTINOUS_CONVERSION)
		INT_ENABLE_GLOBAL_INT;
}

u16 MADC_u16_ReadChannelSynch ( enum ADC_CHANNEL Channel )
{
	SET_BIT(ADC_REG->ADC_ADCSRA,7);
	u16 Local_Result = 0;
	ADC_REG->ADC_ADMUX &= 0xF0;
	ADC_REG->ADC_ADMUX |= Channel;
	SET_BIT (ADC_REG->ADC_ADCSRA,6);
	while (GET_BIT(ADC_REG->ADC_ADCSRA,4)==0);
	SET_BIT(ADC_REG->ADC_ADCSRA,4);
	CLR_BIT(ADC_REG->ADC_ADCSRA,7);
	if (PADC_Conversion_Resolution == ADC_8BIT)
	{
		return (ADC_REG->ADC_ADCH);
	}
	else 
	{
		Local_Result = ADC_REG->ADC_ADCL;
		Local_Result |= ((u16)ADC_REG->ADC_ADCH)<<8;
		return Local_Result;
	}
}

void MADC_void_ReadChannelAsynch(enum ADC_CHANNEL Channel, void (*CallBack)(u16))
{
	PADC_void_CallBack = CallBack;
	ADC_REG->ADC_ADMUX &= 0xF0;
	ADC_REG->ADC_ADMUX |= Channel;
	
}

void MADC_void_StartContinousMode (void)
{
	SET_BIT(ADC_REG->ADC_ADCSRA,7);
	if ( PADC_FREE_RUNNING_MODE )
		SET_BIT(ADC_REG->ADC_ADCSRA,6);
}

void MADC_void_StopContinousMode  (void)
{
	CLR_BIT(ADC_REG->ADC_ADCSRA,7);
}

void MADC_void_SetAutoTriggerSource (enum ADC_TRIG_SRC_t TriggerSource)
{
	if ( TriggerSource == ADC_FREE_RUNNING )
		PADC_FREE_RUNNING_MODE = 1;
	else 
		PADC_FREE_RUNNING_MODE = 0;
	
	CLR_REG(ADC_REG->ADC_ADCSRB);
	ADC_REG->ADC_ADCSRB |= TriggerSource;
}

void MADC_void_EnableDigitalInputBuffer  ( enum ADC_CHANNEL Channel )
{
	if (Channel>5)
		return;
		
	CLR_BIT(ADC_REG->ADC_DIDR0,Channel);
	
}

void MADC_void_DisableDigitalInputBuffer ( enum ADC_CHANNEL Channel )
{
	if (Channel>5)
	return;
	
	SET_BIT(ADC_REG->ADC_DIDR0,Channel);
	
}

void ADC_VECTOR_ISR_ADC (void)
{
	
	if (PADC_void_CallBack != 0)
	{
		u16 Local_Result = 0;
		if (PADC_Conversion_Resolution == ADC_8BIT)
		{
			Local_Result = ADC_REG->ADC_ADCH;
		}
		else
		{
			Local_Result = ADC_REG->ADC_ADCL;
			Local_Result |= ((u16)ADC_REG->ADC_ADCH)<<8;
		}
		
		PADC_void_CallBack(Local_Result);
	}
	if (PADC_FREE_RUNNING_MODE)
		SET_BIT(ADC_REG->ADC_ADCSRA,6);
}