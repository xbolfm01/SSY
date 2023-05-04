/*
 * ADC_lib.h
 *
 * Created: 9.3.2017 16:24:50
 *  Author: Krajsa
 */ 


#ifndef ADC_LIB_H_
#define ADC_LIB_H_

void ADC_Init(int prescale,int uref);
uint16_t ADC_get(int chan);
uint16_t ADC_readTemp();


#endif /* ADC_LIB_H_ */