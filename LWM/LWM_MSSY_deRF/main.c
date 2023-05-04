/*
 * LWM_MSSY_deRF.c
 *
 * Created: 14.4.2017 12:56:52
 * Author : Ondra
 */ 
// NA GIT
#include <avr/io.h>
/************************************************************************/
/* INCLUDES                                                             */
/************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "config.h"
#include "hal.h"
#include "phy.h"
#include "sys.h"
#include "nwk.h"
#include "sysTimer.h"
#include "halBoard.h"
#include "halUart.h"
#include "util/delay.h"
#include "ADC_lib.h"
#include "main.h"
#include "derf/sensors_interface.h"
#include "commands.h"
#include "halUart.h"
#include "halSleep.h"
#include "halBoard.h"
#include "halLed.h"
#include "derf/usb.h"
#include "derf/eeprom.h"
#include "derf/io_access.h"

/************************************************************************/
/* DFEFINES                                                             */
/************************************************************************/
#if defined(APP_COORDINATOR)
  #define APP_NODE_TYPE     0
#elif defined(APP_ROUTER)
  #define APP_NODE_TYPE     1
#else
  #define APP_NODE_TYPE     2
#endif

#define APP_CAPTION_SIZE    (sizeof(APP_CAPTION) - 1)
#define APP_COMMAND_PENDING 0x01

#define APP_ENDPOINT        1
#define APP_LED_NETWORK     0
#define APP_LED_DATA        1


//#define ADDR_ENDPOINT		3
#define ADDR_ENDPOINT		2
#define ADDR_REQUEST_MSG 0x10 
#define ADDR_RESPONSE_MSG 0x20
#define ADDR_CONFIRM_MSG 0x30
#define ADDR_ERROR_MSG 0x40 //No available address to offer
#define ADDR_WAIT_TIME 5000

#define RSSI_ENDPOINT		4
#define RSSI_SEND_TIME	 10000
#define RSSI_ENABLE

#define PRIMAR			0x1		
#define SEKUNDAR		0x2		

#define EEPROM_START 0
#define APP_MAX_NODES 20
#define NODE_LIST_T_SIZE 166

#define DISTANCE_1	2 // Vzdialenost d0 - fyzicky zmerane
#define DISTANCE_2	4	// Dalsia vzdialenost d1 - fyzicky zmerane
/************************************************************************/
/* TYPES                                                                */
/************************************************************************/
typedef struct PACK
{
  uint8_t      commandId;
  uint8_t      nodeType;
  uint64_t     extAddr;
  uint16_t     shortAddr;
  uint32_t     softVersion;
  uint32_t     channelMask;
  uint8_t      workingChannel;
  uint16_t      panId;
  uint16_t    parentShortAddr;
  uint8_t      lqi;
  int8_t       rssi;

  struct PACK
  {
    uint8_t    type;
    uint8_t    size;
    int32_t    battery;
    int32_t    temperature;
    int32_t    light;
  } sensors;

  struct PACK
  {
    uint8_t    type;
    uint8_t    size;
    char       text[APP_CAPTION_SIZE];
  } caption;
} AppMessage_t; 


typedef struct PACK
{
uint8_t msg_ID;
uint16_t node_address;
uint16_t node_ID;
}AppAddress_t;

typedef enum AppState_t
{
  APP_STATE_INITIAL,
  APP_STATE_ADDR_REQUEST,
  APP_STATE_ADDR_WAIT,
  APP_STATE_SEND,
  APP_STATE_WAIT_CONF,
  APP_STATE_SENDING_DONE,
  APP_STATE_WAIT_SEND_TIMER,
  APP_STATE_WAIT_COMMAND_TIMER,
  APP_STATE_PREPARE_TO_SLEEP,
  APP_STATE_SLEEP,
  APP_STATE_WAKEUP,
} AppState_t;

typedef struct PACK
{
	uint8_t count;
	uint16_t node_address[APP_MAX_NODES];
	uint16_t node_ID[APP_MAX_NODES];
}NodeList_t;

typedef struct PACK
{
	uint8_t state;
	uint16_t my_address;
	uint16_t my_ID;
	#if defined(APP_COORDINATOR)
	//uint16_t pocetUzlu;
	NodeList_t seznamUzlu;
	#endif
}NodeState_t;

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

int printCHAR(char character, FILE *stream);
#if defined(APP_COORDINATOR)
static void appAddrResponse(uint16_t src_addr,uint16_t node_id);
static void appADDR_RESP_Conf(NWK_DataReq_t *req);
#endif
#if defined(APP_ROUTER) || defined(APP_ENDDEVICE)
static void appAddrRequest(uint16_t node_id);
static void appAddrConf(uint16_t my_addr,uint16_t node_id);
static void appADDR_REQ_Conf(NWK_DataReq_t *req);
static void appADDR_CONF_Conf(NWK_DataReq_t *req);
#endif

static void appSendRssi();
static void appRSSIConfirmation(NWK_DataReq_t *req);
static bool appRSSIInd(NWK_DataInd_t *ind);
static void appRSSISendingTimerHandler(SYS_Timer_t *timer);

static uint8_t button0_Check();
static uint8_t button1_Check();
static void board_init();
static void eeprom_read(uint32_t ee_address, uint16_t len, uint8_t *buf);
static void eeprom_write(uint32_t ee_address, uint16_t len, uint8_t *buf);
static void error(void);
static int16_t nodePosition(int16_t address);
static bool isNodeInTable(int16_t address);
static bool existujuca_adresa(int16_t address);
// Definícia metódy pre výpocet zoslabovacieho clena
static double vypocet_zoslabovaciho_clen(double rssi_1, double rssi_2);
/************************************************************************/
/* VARIABLES                                                            */
/************************************************************************/
volatile uint16_t usb_v;
static AppState_t appState = APP_STATE_INITIAL;
//UART DEBUG
FILE uart_str = FDEV_SETUP_STREAM(printCHAR, NULL, _FDEV_SETUP_RW);//soubor pro stdout
//USB DEBUG
FILE usb_stream = FDEV_SETUP_STREAM(usb_putc_std, usb_getc_std,_FDEV_SETUP_RW);


#if defined(APP_ROUTER) || defined(APP_ENDDEVICE)
static NWK_DataReq_t appNwkDataReq;
static SYS_Timer_t appNetworkStatusTimer;
static SYS_Timer_t appCommandWaitTimer;
static SYS_Timer_t appADDRWaitTimer;
static bool appNetworkStatus;
#endif
static NWK_DataReq_t appNwkADDRReq;
static NWK_DataReq_t appNwkRSSIReq; //optimalizovat

static AppMessage_t appMsg;
static SYS_Timer_t appDataSendingTimer;
static SYS_Timer_t appRSSISendingTimer;

//uint16_t myNodeID = 0;
uint32_t messno=0;

NodeState_t stavUzlu;
uint8_t rssi_counter=0;
uint8_t sec_ID = 0; // ID pre jednotlive pakety, bude sa inkrementovat o 1 vzdy


// Pole prijatych RSSI. Prijme sa 5 hodnot a nasledne sa z nich vypocita najlepsia hodnota. 
int16_t pole_prijatych_RSSI[4];
// Kontrola ci zariadenie uz nemalo niekedy v minulosti pridelenu adresu. 
int16_t pole_ulozenych_adres[APP_MAX_NODES];
//int16_t table[APP_MAX_NODES][10] = {0};;
double zoslabovaci_clen = 0;
uint8_t nodeCounter = 0;
uint8_t rssiCounter = 0;
//uint8_t msgCounter = 0;

static uint8_t button0_Check(){
	uint8_t btn_count=0;
	
	for (uint8_t i=0;i<10;i++)
	{
		if(!button_pressed(BUTTON_0)){
			btn_count++;
		}
		_delay_ms(100);
	}

	if (btn_count>8)
	{
		return 1;
	} 
	else
	{
		return 0;
	}
	
}

static uint8_t button1_Check(){
		uint8_t btn_count=0;
		
		for (uint8_t i=0;i<10;i++)
		{
			if(!button_pressed(BUTTON_1)){
				btn_count++;
			}
			_delay_ms(100);
		}

		if (btn_count>8)
		{
			return 1;
		}
		else
		{
			return 0;
		}
}

static void board_init(){
	HAL_BoardInit();
	HAL_LedInit();
	io_init();
	eeprom_init();
	ADC_Init(4,2);
	_delay_ms(50);
	usb_v=ADC_get(1);

	if (usb_v>800) //USB connected, use USB debug, else UART
	{
		usb_init();
		_delay_ms(100);
		// ZMENA ZO STANDART VYSTUPU NA USB VYSTUP,
		// BUDE VYPISOVANY NA USB SERIOVU LINKU
		stdout = &usb_stream;
		stdin  = &usb_stream;
		printf("USB ready \n\r");
	} 
	else
	{
		HAL_UartInit(38400);
		stdout = &uart_str;
		printf("UART ready \n\r");
	}
	 
	printf("Start... \n\r");

	
	//Hold right button during boot to erase EEPROM
	if (button0_Check())//
	{
		// VYTVORI SA POLE arr A VYPLNI SA NULAMI. 
		uint8_t arr[sizeof(NodeState_t)] = {0};
		eeprom_write(0,sizeof(NodeState_t),arr);
		printf("Smazano... \n\r");
		led_set(LED_0, LED_ON);
		led_set(LED_1, LED_ON);
		led_set(LED_2, LED_ON);
		_delay_ms(200);
		led_set(LED_0, LED_OFF);
		led_set(LED_1, LED_OFF);
		led_set(LED_2, LED_OFF);
	}

	
}
	
static void error(void)
{
	/* set flashing time -> CPU frequency / (toggle/flashing time (in ms)) = tick time */
	/* --> this results into 2,5Hz */
	uint32_t FLASH_TICKS = F_CPU / 200;
	uint32_t counter = 0;
	printf("\r\n[EEPROM] Error - Application stopped\r\n");
	while(1)
	{
		if(counter++ == FLASH_TICKS)
		{
			counter = 0;
			led_set(LED_0, LED_TOGGLE);
			led_set(LED_1, LED_TOGGLE);
		}
	}
}

// CITANIE DAT Z EEPROM. *buf JE UKAZATEL NA BUFFER, ODKIAL SA MAJU CITAT DATA
static void eeprom_read(uint32_t ee_address, uint16_t len, uint8_t *buf)
{
   EEPROM_ENABLE();
   int32_t ret = eeprom_read_bytes(ee_address, len, buf);
   EEPROM_DISABLE();
   if(ret <= 0) { error(); }
   printf("[EEPROM] Read succeeded \r\n");
}

static void eeprom_write(uint32_t ee_address, uint16_t len, uint8_t *buf)
{
   EEPROM_ENABLE();
   int16_t ret = eeprom_write_bytes(ee_address, len, buf);
   /* wait until write cycle finished */
   _delay_ms(5);
   EEPROM_DISABLE();
   if(ret <= 0) { error(); }
   printf("\r\n[EEPROM] Successfully wrote %d bytes \r\n", ret);
}

int printCHAR(char character, FILE *stream)
{
	HAL_UartWriteByte(character);

	return 0;
}

void HAL_UartBytesReceived(uint16_t bytes)
{
  for (uint16_t i = 0; i < bytes; i++)
  {
    uint8_t byte = HAL_UartReadByte();
    APP_CommandsByteReceived(byte);
  }
}

/*************************************************************************//**
*****************************************************************************/
static void appUartSendMessage(uint8_t *data, uint8_t size)
{
// 	usb_putc_std(c,NULL);
//   
//   uint8_t cmd_buff[127];
//   uint8_t cmd_buff_pos=0;
   uint8_t cs = 0;
//   

  //HAL_UartWriteByte(0x10);
  usb_putc_std(0x10,NULL);
  //HAL_UartWriteByte(0x02);
  usb_putc_std(0x02,NULL);

  for (uint8_t i = 0; i < size; i++)
  {
    if (data[i] == 0x10)
    {
     // HAL_UartWriteByte(0x10);
	  usb_putc_std(0x10,NULL);
      cs += 0x10;
    }
  //  HAL_UartWriteByte(data[i]);
	usb_putc_std(data[i],NULL);
    cs += data[i];
  }

 // HAL_UartWriteByte(0x10);
  usb_putc_std(0x10,NULL);
 // HAL_UartWriteByte(0x03);
  usb_putc_std(0x03,NULL);
  cs += 0x10 + 0x02 + 0x10 + 0x03;

  //HAL_UartWriteByte(cs);
  usb_putc_std(cs,NULL);

//  cmd_buff[cmd_buff_pos++]=0x10;
//  cmd_buff[cmd_buff_pos++]=0x02;
// 
//   for (uint8_t i = 0; i < size; i++)
//   {
// 	  if (data[i] == 0x10)
// 	  {
// 		  cmd_buff[cmd_buff_pos++]=0x10;
// 		  cs += 0x10;
// 	  }
// 	  cmd_buff[cmd_buff_pos++]=data[i];
// 	  cs += data[i];
//   }
// 
//   cmd_buff[cmd_buff_pos++]=0x10;
//   cmd_buff[cmd_buff_pos++]=0x03;
//   cs += 0x10 + 0x02 + 0x10 + 0x03;
//   cmd_buff[cmd_buff_pos++]=cs;
//  
// printf("%.*s", cmd_buff_pos, cmd_buff);
}

	// Funkcia, pre porovnanie, ktora sa vyuzije neskor pre zoradenie prvkov v poli.
	int compare(const void *a, const void *b)
	{
		int *x = (int*) a;
		int *y = (int*) b;
		return (*x - *y);
	}

static void appUartSendMessageHR(uint8_t *data, uint8_t size)
{
	AppMessage_t *BufferHR = (AppMessage_t *)data;
	//static uint8_t BufferHR_size = size;
	
	
	if (BufferHR->shortAddr!=0)
	{
	//printf("---------------------------------------------\n\r");
	//printf("Command ID: %d \n\r", BufferHR->commandId);
	//printf("Node Type: %d \n\r", BufferHR->nodeType);
	//printf("Ext addr: %llu \n\r", BufferHR->extAddr);
	//printf("Short addr: %hu \n\r", BufferHR->shortAddr);
	printf("Short addr: 0x%X \n\r", BufferHR->shortAddr);
	//printf("Soft ver.: %lu \n\r", BufferHR->softVersion);
	//printf("Channel mask: %lu \n\r", BufferHR->channelMask);
	//printf("Working channel: %d \n\r", BufferHR->workingChannel);
	//printf("Pan ID: %hu \n\r", BufferHR->panId);
	printf("Parent addr: 0x%X \n\r", BufferHR->parentShortAddr);
	//printf("Parent addr: %hu \n\r", BufferHR->parentShortAddr);
	printf("LQI: %d \n\r", BufferHR->lqi);
	printf("RSSI: %d \n\r", BufferHR->rssi);
	//printf("Node Type: %d \n\r", BufferHR->nodeType);
	//printf("Sensors Type: %d \n\r", BufferHR->sensors.type);
	//printf("Sensors Size: %d \n\r", BufferHR->sensors.size);
	printf("Sensors Battery: %ld \n\r", BufferHR->sensors.battery);
	printf("Sensors Temp: %ld \n\r", BufferHR->sensors.temperature);
	printf("Sensors Light: %ld \n\r", BufferHR->sensors.light);
	//printf("Caption Type: %d \n\r", BufferHR->caption.type);
	//printf("Caption Size: %d \n\r", BufferHR->caption.size);
	printf("Caption Text:");
	int i;
	for (i=0;i<BufferHR->caption.size;i++)
	{
		printf("%c",(char)BufferHR->caption.text[i]);
		//HAL_UartWriteByte(BufferHR->caption.text[i]);
	}
	//printf("%.*s", BufferHR->caption.size, BufferHR->caption.text);
	printf("\n\r");
	}
	else{
		
	}
	
#if defined(APP_COORDINATOR)
	// Kontrola ci sa uzol s danou adresou uz nachadza v tabulke alebo nie.
	sec_ID++;
	int16_t address = (int16_t)BufferHR->shortAddr;
	if (!existujuca_adresa(address)) 
	{
		// Ak sa nenachadza a jeho adresa je rozna od nuly, tak sa zapise.
		if (address != 0)
		{
			pole_ulozenych_adres[nodeCounter] = address;
			pole_ulozenych_adres[nodeCounter] = true;
			nodeCounter++;
		}
	}
	
	else {
		if (address != 0){
			int16_t nodePos = nodePosition((int16_t)BufferHR->shortAddr);
			if (nodePos != (APP_MAX_NODES + 1)) 
			{
				// Vybere sa median z prijatych hodnot RSSI.  
				//int rssi_MEDIAN = 0;
				int n = sizeof(pole_prijatych_RSSI) / sizeof(pole_prijatych_RSSI[0]);
				qsort(pole_prijatych_RSSI, n, sizeof(int), compare);
			    int element = pole_prijatych_RSSI[2];
			    printf("Median z prijatych RSSI je : %d\n", element);		
			}else
			{
				printf("V poli nic neni. \n");	
			}
				
		}
	}
		#endif

		// Volanie fcie pre vypocet zoslabovacieho clena
		eta = zoslabovaci_clen(pole_prijatych_RSSI[nodePosition((int16_t)rssi_1)], pole_prijatych_RSSI[nodePosition((int16_t)rssi_2)]);
		msgCounter = 0;

		// Volanie fcie pre vypocet vzdialenosti od koordinatora
		int16_t vzdialenost = (int16_t)DISTANCE_1 * pow(10, (pole_ulozenych_adres[nodePosition((int16_t)PRIMAR)] - pole_ulozenych_adres[1])/(10*eta));
		printf("%d \n\r", vzdialenost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

// Funkcia, ktora overi ci koordinator uz v minulosti pridelil zariadeniu adresu alebo nie.
static bool existujuca_adresa(int16_t address) {
	for (int16_t i = 0; i < nodeCounter; i++){
		if (pole_ulozenych_adres[i] == address) {
			return true;
		}
	}
	return false;
}


static double vypocet_zoslabovaciho_clen(double rssi_1, double rssi_2) {
	double zoslabovaci_clen = (double)((double)rssi_2 - (double)rssi_1)/(10*log10((double)DISTANCE_1/(double)DISTANCE_2));
	printf("%X", zoslabovaci_clen);
	return zoslabovaci_clen;
}

double calculateDistanceFromRSSI(BufferHR->rssi_, int txPower, double eta) {
	double distance = pow(10.0, ((double) txPower - rssi) / (10 * zoslabovaci_clen));
	return distance;
}

static int16_t nodePosition(int16_t address) {
	for (int16_t i = 0; i < nodeCounter; i++){
		if (pole_ulozenych_adres[i] == address) {
			return i;
		}
	}
	return APP_MAX_NODES + 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

/*************************************************************************//**
*****************************************************************************/
static bool appDataInd(NWK_DataInd_t *ind)
{
  AppMessage_t *msg = (AppMessage_t *)ind->data; 
  //not whole message, but payload only

  HAL_LedToggle(APP_LED_DATA);

  msg->lqi = ind->lqi;
  msg->rssi = ind->rssi;
  
  #if defined(APP_COORDINATOR)          
  appUartSendMessageHR(ind->data, ind->size);
  #else
  appUartSendMessage(ind->data, ind->size);
  #endif

  if (APP_CommandsPending(ind->srcAddr))
    NWK_SetAckControl(APP_COMMAND_PENDING);

  return true;
}

/////////////////////////////////////////////////////////////////////////////////
static bool appAddrInd(NWK_DataInd_t *ind)
{
	printf("\033[1;31;40m");
	printf("Address message \n\r");
	// PRETYPOVANIE POINTERA Z NWK_DataInd_t NA POINTER TYPU AppAddress_t, PRETO
	// ABY STRUKTURA AppAdress_t, KTORA JE DEFINOVANA NA ZACIATKU PROGRAMU, MOHLA
	// BYT POUZITA PRE DALSIE SPRACOVANIE ADRESY.
	AppAddress_t *addr_msg = (AppAddress_t *)ind->data;
	
	switch (addr_msg->msg_ID)
	{
	#if defined(APP_COORDINATOR)
	case ADDR_REQUEST_MSG:
	
	appAddrResponse(ind->srcAddr,addr_msg->node_ID);
	printf("Addr.REQ , ID %x \n\r",addr_msg->node_ID);
	
	break;
		
	case ADDR_CONFIRM_MSG:
	printf("Add.CONF %x by ID %x \n\r",addr_msg->node_address,addr_msg->node_ID);
	break;
	
	#endif
	
	#if defined(APP_ROUTER) || defined(APP_ENDDEVICE)
	
	case ADDR_RESPONSE_MSG:
	printf("Add. RESP, %x \n\r",addr_msg->node_address);
	
	if (addr_msg->node_ID==stavUzlu.my_ID) //pouze pro moje ID
	{
		SYS_TimerStop(&appADDRWaitTimer); //stop wait timer
		
		appMsg.extAddr = addr_msg->node_address;
		appMsg.shortAddr = addr_msg->node_address;
		stavUzlu.my_address=addr_msg->node_address;
		stavUzlu.state=1;
		NWK_SetAddr(stavUzlu.my_address);
		eeprom_write(EEPROM_START,sizeof(NodeState_t),(uint8_t *) &stavUzlu);
		appAddrConf(stavUzlu.my_address,stavUzlu.my_ID);
		
	}
	//appState = APP_STATE_ADDR_WAIT;
	
	break;
		
	#endif
	
	case ADDR_ERROR_MSG:
	#if defined(APP_ROUTER) || defined(APP_ENDDEVICE)
	_delay_ms(60000);	//delay nebo sleep po nekolik minut;PREDELAT
	appState=APP_STATE_ADDR_REQUEST;

	#endif
	
	break;	
	default :
	printf("Unknown Address Message, SRC:%X, MSG_ID:%X \n\r",ind->srcAddr,addr_msg->msg_ID);		
	}
	printf("\033[0;37;40m");
	return true;
}

static bool appRSSIInd(NWK_DataInd_t *ind)
{	
	uint8_t id = ind->data[0];
	
}
/*************************************************************************//**
*****************************************************************************/
static void appDataSendingTimerHandler(SYS_Timer_t *timer)
{
  if (APP_STATE_WAIT_SEND_TIMER == appState)
    appState = APP_STATE_SEND;
  else
    SYS_TimerStart(&appDataSendingTimer);

  (void)timer;
}

#if defined(APP_ROUTER) || defined(APP_ENDDEVICE)
/*************************************************************************//**
*****************************************************************************/
static void appNetworkStatusTimerHandler(SYS_Timer_t *timer)
{
  HAL_LedToggle(APP_LED_NETWORK);
  (void)timer;
}

/*************************************************************************//**
*****************************************************************************/
static void appCommandWaitTimerHandler(SYS_Timer_t *timer)
{
  appState = APP_STATE_SENDING_DONE;
  (void)timer;
}

static void appADDRWaitTimerHandler(SYS_Timer_t *timer)
{
	
	appState = APP_STATE_PREPARE_TO_SLEEP;  //Not receveid ADDR response, go to sleep....
	(void)timer;
}
#endif

static void appRSSISendingTimerHandler(SYS_Timer_t *timer)
{
	appSendRssi();
	//HAL_LedToggle(APP_LED_NETWORK);
	(void)timer;
}
/*************************************************************************//**
*****************************************************************************/

static void appRSSIConfirmation(NWK_DataReq_t *req)
{
	printf("Prijate udaje od zariadenia vratane RSSI: \n\r");
	HAL_LedOff(APP_LED_DATA);
	SYS_TimerStart(&appRSSISendingTimer);
	//osetrit lepe....
}

#if defined(APP_ROUTER) || defined(APP_ENDDEVICE)
static void appDataConf(NWK_DataReq_t *req)
{
  HAL_LedOff(APP_LED_DATA);

  if (NWK_SUCCESS_STATUS == req->status)
  {
    if (!appNetworkStatus)
    {
      HAL_LedOn(APP_LED_NETWORK);
      SYS_TimerStop(&appNetworkStatusTimer);
      appNetworkStatus = true;
    }
  }
  else
  {
    if (appNetworkStatus)
    {
      HAL_LedOff(APP_LED_NETWORK);
      SYS_TimerStart(&appNetworkStatusTimer);
      appNetworkStatus = false;
    }
  }

  if (APP_COMMAND_PENDING == req->control)
  {
    SYS_TimerStart(&appCommandWaitTimer);
    appState = APP_STATE_WAIT_COMMAND_TIMER;
  }
  else
  {
	  if (appState != APP_STATE_ADDR_WAIT)
	  {
		  appState = APP_STATE_SENDING_DONE;
	  }
    
  }
}
// FUNKCIA, KTORA SA VOLA PRO ODOSLANI POZIADAVKU PRE ZISKANIE ADRESY. 
static void appADDR_REQ_Conf(NWK_DataReq_t *req)
{
	appState = APP_STATE_ADDR_WAIT;
}
static void appADDR_CONF_Conf(NWK_DataReq_t *req)
{
	appState = APP_STATE_SEND;
}
#endif

#if defined(APP_COORDINATOR)

// FUNKCIA, KTORA SA VOLA PO ODOSLANIE ODPOVEDI NA POZIADAVKU NA ZISKANIE ADRESY. 
static void appADDR_RESP_Conf(NWK_DataReq_t *req)
{
	

	if (NWK_SUCCESS_STATUS == req->status)
	{
		//Osetrit, mozna:
		//appState=APP_STATE_SEND;
		
	}
	else
	{
		//osetrit
	}
	
	
}
#endif
/*************************************************************************//**
*****************************************************************************/
static void appSendData(void)
{
	TWI_MasterInit();
    BMA150_Init();
	ISL29020_Init();
	TMP102_Init();
	
// 	// temperature sensor value */
 	static temperature_t temp;
 	// luminosity sensor value */
 	static luminosity_t lumi;
// 	// acceleration sensor value */
 	static acceleration_t accel;
// 	// measure temperature */
 	TMP102_StartOneshotMeasurement();
 	TMP102_GetTemperature(&temp, true);
// 	// measure luminosity */
 	ISL29020_StartOneshotMeasurement();
 	ISL29020_GetLuminosity(&lumi);
// 	// measure acceleration */
 	BMA150_GetAcceleration(&accel);
	
	
// AK JE NWK_ENABLE_ROUTING V KODE DEFINOVANY, TAK SA DO parentShortAddr ULOZI
// ADRESA NEXT HOP PRE DORUCENIE SPRAVY POD ROUTING TABLE. AK NENI DEFINOVANY,
// TAK SA TAM ULOZI 0.	
#ifdef NWK_ENABLE_ROUTING
  appMsg.parentShortAddr = NWK_RouteNextHop(0, 0);
#else
  appMsg.parentShortAddr = 0;
#endif

// DATA ZO SENZOROV
  ADC_Init(4,2);
  appMsg.sensors.battery     = ADC_get(0);
  appMsg.sensors.temperature = temp.integralDigit; //ADC_readTemp();
  appMsg.sensors.light       = lumi;

#if defined(APP_COORDINATOR)
  
  SYS_TimerStart(&appDataSendingTimer);
  appState = APP_STATE_WAIT_SEND_TIMER;
#else

  appNwkDataReq.dstAddr = 0;
  appNwkDataReq.dstEndpoint = APP_ENDPOINT;
  appNwkDataReq.srcEndpoint = APP_ENDPOINT;
  appNwkDataReq.options = NWK_OPT_ACK_REQUEST | NWK_OPT_ENABLE_SECURITY;
  appNwkDataReq.data = (uint8_t *)&appMsg;
  appNwkDataReq.size = sizeof(appMsg);
  appNwkDataReq.confirm = appDataConf;

  HAL_LedOn(APP_LED_DATA);
  NWK_DataReq(&appNwkDataReq);

  appState = APP_STATE_WAIT_CONF;
#endif
}


///////////////////////////////////////////////////////////////////////////
// PRIDELOVANIE ADRIES //

#if defined(APP_COORDINATOR)
static void appAddrResponse(uint16_t src_addr,uint16_t node_id)
{
	static AppAddress_t msg;

	
	// KONTROLA CI NENI PREKROCENY MAX POCET UZLOV.	
	if (stavUzlu.seznamUzlu.count<APP_MAX_NODES)
	{
			msg.node_ID=node_id;
			msg.msg_ID=ADDR_RESPONSE_MSG;
		
		// AK JE ADRESA UZLU MENSIA AKO 0x8000, TAK SA UZLU PRIRADI ADRESA Z
		// NASLEDUJUCEHO CISLA V ZOZNAME.		
		if(src_addr<0x8000){
			stavUzlu.seznamUzlu.node_address[stavUzlu.seznamUzlu.count]=stavUzlu.seznamUzlu.count+1;
			stavUzlu.seznamUzlu.node_ID[stavUzlu.seznamUzlu.count]=node_id;
			msg.node_address=stavUzlu.seznamUzlu.node_address[stavUzlu.seznamUzlu.count];
		
		// AK JE ADRESA UZLU VACSIA AKO 0x8000, TAK SA UZLU PRIRADI ADRESA AKO
		// NASLEDUJUCE CISLO V ZOZNAME + 0x8000.
		}
		else{
			stavUzlu.seznamUzlu.node_address[stavUzlu.seznamUzlu.count]=stavUzlu.seznamUzlu.count+1+0x8000;
			stavUzlu.seznamUzlu.node_ID[stavUzlu.seznamUzlu.count]=node_id;
			msg.node_address=stavUzlu.seznamUzlu.node_address[stavUzlu.seznamUzlu.count];
		}
	
	stavUzlu.seznamUzlu.count=stavUzlu.seznamUzlu.count+1;
	}
	else {
		
		msg.node_ID=node_id;
		msg.msg_ID=ADDR_ERROR_MSG;
		
	}
		
	appNwkADDRReq.dstAddr = src_addr;
	appNwkADDRReq.dstEndpoint = ADDR_ENDPOINT;
	appNwkADDRReq.srcEndpoint = ADDR_ENDPOINT;
	appNwkADDRReq.options = NWK_OPT_ACK_REQUEST | NWK_OPT_ENABLE_SECURITY;
	// UKAZATEL NA PREMENNU msg A UKAZATEL JE ULOZENY V STRUKTURE appNwkADDRReq,
	// CIZE PREMENNA msg, BUDE ZAHRNUTA DO PAKETU, KTORY SA ODOSLE CEZ appNwkADDRReq.
	appNwkADDRReq.data = (uint8_t *)&msg;
	appNwkADDRReq.size = sizeof(msg);
	appNwkADDRReq.confirm =  appADDR_RESP_Conf;

	HAL_LedOn(APP_LED_DATA);
	
	eeprom_write(EEPROM_START,sizeof(NodeState_t),(uint8_t *) &stavUzlu);
	
	NWK_DataReq(&appNwkADDRReq);

	//appState = APP_STATE_WAIT_CONF; //Osetrit
	
}
#endif

// METODA PRE ZISKANIE ADRESY PRE ROUTER ALEBO ED. POSIELA SA NA KOORDINATORA 0x0000.
#if defined(APP_ROUTER) || defined(APP_ENDDEVICE)
static void appAddrRequest(uint16_t node_id)
{
	static AppAddress_t msg;
	msg.node_ID=node_id;
	msg.msg_ID=ADDR_REQUEST_MSG;
	msg.node_address=APP_ADDR;
	
	
	appNwkADDRReq.dstAddr = 0x0000;
	appNwkADDRReq.dstEndpoint = ADDR_ENDPOINT;
	appNwkADDRReq.srcEndpoint = ADDR_ENDPOINT;
	appNwkADDRReq.options = NWK_OPT_ACK_REQUEST | NWK_OPT_ENABLE_SECURITY;	
	appNwkADDRReq.data = (uint8_t *)&msg;
	appNwkADDRReq.size = sizeof(msg);
	appNwkADDRReq.confirm = appADDR_REQ_Conf;

	HAL_LedOn(APP_LED_DATA);
	NWK_DataReq(&appNwkADDRReq);

	appState = APP_STATE_ADDR_WAIT;
	
	SYS_TimerStart(&appADDRWaitTimer);
	
}

// KONFIRMACNA METODA OD ED SMEROM NA COORDINATORA, ZE MA PRIDELENU ADRESU
static void appAddrConf(uint16_t my_addr,uint16_t node_id)
{
	static AppAddress_t msg;
	msg.node_ID=node_id;
	msg.msg_ID=ADDR_CONFIRM_MSG;
	msg.node_address=my_addr;
	
	
	appNwkADDRReq.dstAddr = 0x0000;
	appNwkADDRReq.dstEndpoint = ADDR_ENDPOINT;
	appNwkADDRReq.srcEndpoint = ADDR_ENDPOINT;
	appNwkADDRReq.options = NWK_OPT_ACK_REQUEST | NWK_OPT_ENABLE_SECURITY;
	appNwkADDRReq.data = (uint8_t *)&msg;
	appNwkADDRReq.size = sizeof(msg);
	appNwkADDRReq.confirm = appADDR_CONF_Conf;

	HAL_LedOn(APP_LED_DATA);
	NWK_DataReq(&appNwkADDRReq);

	//appState = APP_STATE_WAIT_CONF;
	
}
#endif
static void appSendRssi(){
	
// METODA PRE ODOSIELANIE HODNOTY RSSI, NA ENDPOINT 4 (RSSI ENDPOINT) 
	appNwkRSSIReq.dstAddr = 0xFFFF;
	appNwkRSSIReq.dstEndpoint = RSSI_ENDPOINT;
	appNwkRSSIReq.srcEndpoint = RSSI_ENDPOINT;
	appNwkRSSIReq.options = NWK_OPT_ENABLE_SECURITY;
	appNwkRSSIReq.data = &rssi_counter;
	appNwkRSSIReq.size = sizeof(rssi_counter);
	// VYPIS DO KONZOLE, ZE SA ODOSLALO RSSI 
	appNwkRSSIReq.confirm = appRSSIConfirmation;

	HAL_LedOn(APP_LED_DATA);
	NWK_DataReq(&appNwkRSSIReq);
	//rssi_counter++;
	sec_ID++; // ID pre jednotlive spravy
	
}

/*************************************************************************//**
*****************************************************************************/
static void appInit(void)
{
  board_init();
  eeprom_read(EEPROM_START,sizeof(NodeState_t),(uint8_t *) &stavUzlu);
   
   	
  appMsg.commandId            = APP_COMMAND_ID_NETWORK_INFO;
  appMsg.nodeType             = APP_NODE_TYPE;
  appMsg.extAddr              = APP_ADDR;
  appMsg.shortAddr            = APP_ADDR;
  appMsg.softVersion          = 0x01010100;
  appMsg.channelMask          = (1L << APP_CHANNEL);
  appMsg.panId                = APP_PANID;
  appMsg.workingChannel       = APP_CHANNEL;
  appMsg.parentShortAddr      = 0;
  appMsg.lqi                  = 0;
  appMsg.rssi                 = 0;

  appMsg.sensors.type        = 1;
  appMsg.sensors.size        = sizeof(int32_t) * 3;
  appMsg.sensors.battery     = 0;
  appMsg.sensors.temperature = 0;
  appMsg.sensors.light       = 0;

  appMsg.caption.type         = 32;
  appMsg.caption.size         = APP_CAPTION_SIZE;
  memcpy(appMsg.caption.text, APP_CAPTION, APP_CAPTION_SIZE);
 
 	/*  rssiMsg.shortAddr         = APP_ADDR;
 	  rssiMsg.rssi              = 0;
 	  rssiMsg.secID				= sec_ID; // Identifikator spravy 
	*/


  if (stavUzlu.state==0)
  {
	   NWK_SetAddr(APP_ADDR);
	   printf("Config empty\n\r");
	   
  } 
  else
  {
	  NWK_SetAddr(stavUzlu.my_address);
	  appMsg.extAddr              = stavUzlu.my_address;
	  appMsg.shortAddr            = stavUzlu.my_address;
	  printf("Using saved config, address: %x\n\r",stavUzlu.my_address);
  }
 
  
  NWK_SetPanId(APP_PANID);
  PHY_SetChannel(APP_CHANNEL);
#ifdef PHY_AT86RF212
  PHY_SetBand(APP_BAND);
  PHY_SetModulation(APP_MODULATION);
#endif
  PHY_SetRxState(true);

#ifdef NWK_ENABLE_SECURITY
  NWK_SetSecurityKey((uint8_t *)APP_SECURITY_KEY);
#endif

  NWK_OpenEndpoint(APP_ENDPOINT, appDataInd);
  NWK_OpenEndpoint(ADDR_ENDPOINT, appAddrInd);
  // RSSI
  NWK_OpenEndpoint(RSSI_ENDPOINT, appRSSIInd);

  appDataSendingTimer.interval = APP_SENDING_INTERVAL;
  appDataSendingTimer.mode = SYS_TIMER_INTERVAL_MODE;
  appDataSendingTimer.handler = appDataSendingTimerHandler;

#if defined(APP_ROUTER) || defined(APP_ENDDEVICE)
  appNetworkStatus = false;
  appNetworkStatusTimer.interval = 500;
  appNetworkStatusTimer.mode = SYS_TIMER_PERIODIC_MODE;
  appNetworkStatusTimer.handler = appNetworkStatusTimerHandler;
  SYS_TimerStart(&appNetworkStatusTimer);

  appCommandWaitTimer.interval = NWK_ACK_WAIT_TIME;
  appCommandWaitTimer.mode = SYS_TIMER_INTERVAL_MODE;
  appCommandWaitTimer.handler = appCommandWaitTimerHandler;
 
  appADDRWaitTimer.interval = ADDR_WAIT_TIME;
  appADDRWaitTimer.mode = SYS_TIMER_INTERVAL_MODE;
  appADDRWaitTimer.handler = appADDRWaitTimerHandler;
#else
  HAL_LedOn(APP_LED_NETWORK);
#endif

// Opatovne zavolanie RSSI casovacov
  appRSSISendingTimer.interval = RSSI_SEND_TIME;
  appRSSISendingTimer.mode = SYS_TIMER_INTERVAL_MODE;
  appRSSISendingTimer.handler = appRSSISendingTimerHandler;

#ifdef PHY_ENABLE_RANDOM_NUMBER_GENERATOR
  srand(PHY_RandomReq());
#endif

  APP_CommandsInit();

#if defined(APP_ROUTER) || defined(APP_ENDDEVICE)
	if (stavUzlu.state==0)
	{
		appState = APP_STATE_ADDR_REQUEST;
	} 
	else
	{
		appState = APP_STATE_SEND;
	}
	
#else
	appState = APP_STATE_SEND;
	stavUzlu.state=1;
#endif
  
  ADC_Init(4,2);
 
 
 //#if defined(APP_ROUTER) || defined(APP_ENDDEVICE)
 #ifdef RSSI_ENABLE
 SYS_TimerStart(&appRSSISendingTimer);
 #endif 
 //#endif
}

/*************************************************************************//**
*****************************************************************************/
static void APP_TaskHandler(void)
{
  switch (appState)
  {
    case APP_STATE_INITIAL:
    {
      appInit();
    } break;

    case APP_STATE_SEND:
    {
      appSendData();
    } break;
	
	#if defined(APP_ROUTER) || defined(APP_ENDDEVICE)
	case APP_STATE_ADDR_REQUEST:
	{
	   stavUzlu.my_ID = rand() & 0xffff;
	   appAddrRequest(stavUzlu.my_ID);
	   //printf("ADDR_req, nodeID=%x \n\r",stavUzlu.my_ID);
	   
	}break;
	case APP_STATE_ADDR_WAIT:
	{
		
	}break;
	#endif
	
	case APP_STATE_WAIT_CONF:
	{
		
	}break;

    case APP_STATE_SENDING_DONE:
    {
#if defined(APP_ENDDEVICE)
      appState = APP_STATE_PREPARE_TO_SLEEP;
#else
      SYS_TimerStart(&appDataSendingTimer);
      appState = APP_STATE_WAIT_SEND_TIMER;
#endif
    } break;

    case APP_STATE_PREPARE_TO_SLEEP:
    {
      if (!NWK_Busy())
      {
        NWK_SleepReq();
        appState = APP_STATE_SLEEP;
      }
    } break;

    case APP_STATE_SLEEP:
    {
      HAL_LedClose();
	  printf("Sleep... \r\n");
      HAL_Sleep(APP_SENDING_INTERVAL);
      appState = APP_STATE_WAKEUP;
    } break;

    case APP_STATE_WAKEUP:
    {
      NWK_WakeupReq();
      printf("WakeUp... \r\n");
      HAL_LedInit();
      HAL_LedOn(APP_LED_NETWORK);

      appState = APP_STATE_SEND;
    } break;

    default:
      break;
  }
}

/*************************************************************************//**
*****************************************************************************/
int main(void)
{
  SYS_Init();

  while (1)
  {
    SYS_TaskHandler();
    HAL_UartTaskHandler();
    APP_TaskHandler();
  }
}
