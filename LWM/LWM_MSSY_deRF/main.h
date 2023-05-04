/*
 * main.h
 *
 * Created: 6.4.2017 16:58:47
 *  Author: Krajsa
 */ 

#include "derf/sensors_interface.h"
#ifndef MAIN_H_
#define MAIN_H_

/*************PERIFERIE*******************************/
#ifdef HAL_ATMEGA128RFA1
#define LED0 PG5
#define LED1 PE3
#define LED2 PE4
#define BUT1 PB7
#endif
#ifdef HAL_ATMEGA256RFR2
#define LED0 PB4
#define LED1 PB5
#define LED2 PB6
#define LED3 PE3
#define BUT1 PE5
#define BUT2 PF0
#define BUT3 PF1
#endif
/*****************************************************/
/*************MAKRA***********************************/
#define sbi(var, mask)  ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)  ((var) &= (uint8_t)~(1 << mask))
#define tbi(var,mask)	(var & (1 << mask) )
#define xbi(var,mask)	((var)^=(uint8_t)(1 << mask))
/*****************************************************/
#ifdef HAL_ATMEGA128RFA1
#define LED0ON (sbi(PORTG,LED0))
#define LED0OFF (cbi(PORTG,LED0))
#define LED0SW (xbi(PORTG,LED0))
#define LED1ON (sbi(PORTE,LED1)) 
#define LED1OFF (cbi(PORTE,LED1))
#define LED1SW (xbi(PORTE,LED1)) 
#define LED2ON (sbi(PORTE,LED2))
#define LED2OFF (cbi(PORTE,LED2))
#define LED2SW (xbi(PORTE,LED2))
#endif
#ifdef HAL_ATMEGA256RFR2
#define LED0ON (sbi(PORTB,LED0))
#define LED0OFF (cbi(PORTB,LED0))
#define LED0SW (xbi(PORTB,LED0))
#define LED1ON (sbi(PORTB,LED1))
#define LED1OFF (cbi(PORTB,LED1))
#define LED1SW (xbi(PORTB,LED1))
#define LED2ON (sbi(PORTB,LED2))
#define LED2OFF (cbi(PORTB,LED2))
#define LED2SW (xbi(PORTB,LED2))
#define LED3ON (sbi(PORTE,LED3))
#define LED3OFF (cbi(PORTE,LED3))
#define LED3SW (xbi(PORTE,LED3))
#endif

typedef struct{
	int Addr;
	int8_t RSSI;
} Soused_t;

typedef struct{
	Soused_t soused[100];
	int pocet;
}SousedeBuff_t;

extern bool appDataReqBusy;
extern NWK_DataReq_t appDataReq;
extern SousedeBuff_t Sousede;
extern temperature_t temp;
extern uint8_t tempAddr;
extern uint16_t AddrSeed;


/* luminosity sensor value */
extern luminosity_t lumi;

/* acceleration sensor value */
extern acceleration_t accel;

//void SendDataToBuff(int Dest_Addr, int App_end, void *data,int delka);
//void appSendDataU(int Dest_Addr, int App_end, void *data, int delka);
//void appDataConf(NWK_DataReq_t *req);

#endif /* MAIN_H_ */