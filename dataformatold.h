/***********************************************************************************
    Filename:     per_test.h

    Description:  PER test header file

***********************************************************************************/

#ifndef DATAFORMAT_H
#define DATAFORMAT_H

/***********************************************************************************
* INCLUDES
*/


#include "contiki.h"
#include "soc.h"
#include "stack.h"
#include "sys/clock.h"
#include "sys/autostart.h"
#include "dev/serial-line.h"
#include "dev/slip.h"
#include "dev/leds.h"
#include "dev/io-arch.h"
#include "dev/dma.h"
#include "dev/cc2530-rf.h"
#include "dev/watchdog.h"
#include "dev/clock-isr.h"
#include "dev/port2.h"
#include "dev/lpm.h"
#include "dev/button-sensor.h"
#include "dev/adc-sensor.h"
#include "dev/leds-arch.h"
#include "net/rime.h"
#include "net/netstack.h"
#include "net/mac/frame802154.h"
#include "debug.h"
#include "cc253x.h"
#include "sfr-bits.h"
#include "contiki-lib.h"
#include "contiki-net.h"

//----------for list handling and memory alloc---------//
#include "lib/list.h"
#include "lib/memb.h"
//-----------------------------------------------------//

#include <stdio.h> /* For printf() */
#include <string.h>


//void utilReverseBuf(uint8_t* pBuf, uint8_t length);

//#define st(x)      do { x } while (__LINE__ == -1)
//#define UINT16_HTON(x)  st( utilReverseBuf((uint8*)&x, sizeof(uint16_t)); )



//--------list handling-----//
typedef struct packetListStruct
{
    struct packetListStruct *next;
    void *bufCache;
}packetListStruct_t;
LIST(packetBuf_list);
MEMB(packetBuf_memb, struct packetListStruct, 6);


//-------dongbo huang-------//

//-----------sleep flag processing--------------//
static int sleepingCMD=0;
int process_getSleepCMD();
void process_setSleepCMD(int state);
//----------------dongbo huang-----------------//



#define HELLOMSG 0x00
#define SLEEPMSG 0xff

//routing table
typedef struct{
    rimeaddr_t route;
    int8_t hopCount;
    int8_t rssi;
}tableContent;


/***********************************************************************************
* TYPEDEFS
*/

typedef struct {
	uint8_t srcAddr[2];
	uint8_t hopCount;
	uint8_t instructionType;
}cumt_instruction;



typedef struct {
	uint8_t startAddr[2];
	//uint8_t finalAddr[2];
	uint8_t temperature[2];
}cumt_information;


/**************************************From Monitoring Software************************/


void get_self_addr(void);
int buildAndSendFrame(uint8_t* payload, int payloadLen,  rimeaddr_t nxthop);
int instructionSend(uint8_t instructin);

//for sensor

float getTemperature();
//int temperatureDataSend(rimeaddr_t nxthop, uint8_t *final_dst);
int temperatureDataSend(rimeaddr_t nxthop);
int retrans(uint8_t *overAir,int payloadLen, rimeaddr_t nxthop);

//for input method
int is_broadcast_addr(uint8_t *addr);
int is_myAddr(uint8_t *addr);


int sensor_incomingPktProcessing(void);
int coord_incomingPktProcessing(void);

//int incomingPktProcessing(void);

//int dataformatInitialization(void);

//void addAddress(rimeaddr_t* addr);


void addSleepTimer(uint16_t sec);

void intoPM2Mode();



////////////////////////////////////////////////////////////////////////////////////

//clock_time_t default_timebase(void);

//////////////////////for test//////////////////////////////

//int testDataSend(void);
//
//int receivedData(void);

void addIntoPacketList(packetListStruct_t *s);
void popFromPacketList(list_t *p);

#endif

