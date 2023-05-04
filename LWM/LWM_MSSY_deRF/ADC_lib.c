/*
* ADC_lib.c
*
* Created: 9.3.2017 16:24:34
*  Author: Krajsa
*/

#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>

/* --------------- MAKRA ------------------*/
#define sbi(var, mask)  ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)  ((var) &= (uint8_t)~(1 << mask))
#define tbi(var,mask)	(var & (1 << mask) )
#define xbi(var,mask)	((var)^=(uint8_t)(1 << mask))
/*-----------------------------------------*/
#include "ADC_lib.h"
void ADC_Init(int prescale,int uref) {
	ADMUX=0;
	ADCSRA=0;
	ADCSRA |= (prescale<<ADPS0);
	ADMUX |= (uref<<REFS0);
	//Kontrola jesli jsou napeti OK
	sbi(ADCSRA,ADEN);
	while(!(ADCSRB & 0x80));
	while(!(ADCSRB & 0x20));
}
void ADC_stop(void){
	cbi(ADCSRA,ADEN);
}
uint16_t ADC_get(int chan) {
	uint16_t tmp=0;
	//smazat MUX
	ADMUX &= ~(15 << MUX0);
	ADCSRB&= ~(1 << MUX5);
	//nastavit spravny
	ADMUX |= (chan<<MUX0);
	ADCSRA |= 0x40; // spustit konverzi
	while((tbi(ADCSRA,ADSC))){} //pockat nez skonci
	//pomocna = ADCL | (ADCH << 8);
	tmp=ADC;
	ADCSRA |= (1<<ADIF);
	return tmp;
}
uint16_t ADC_readTemp(){
	uint16_t tmp=0;
	ADMUX &= ~(15 << MUX0);
	ADCSRB&= ~(1 << MUX5);
	ADCSRB|= (1<<MUX5);
	ADMUX |= (0b01001<<MUX0);
	ADCSRA |= 0x40; // spustit konverzi
	while((tbi(ADCSRA,ADSC))){} //pockat nez skonci
	//pomocna = ADCL | (ADCH << 8);
	tmp=ADC;
	ADCSRA |= (1<<ADIF);
	return tmp;
	
}