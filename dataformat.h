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



//--------packetbuf list handling-----//
typedef struct packetbufListStruct
{
    struct packetbufListStruct *next;
    void *framebuf;
    int dataLen;
}packetbufListStruct_t;


LIST(packetbuf_list);
MEMB(packetbuf_memb, struct packetbufListStruct, 10);
//--------------------------------------

//--------routing table ------------
//typedef struct routingTableStruct
//{
    //struct routingTableStruct *next;
    //rimeaddr_t *addr;
//}

rimeaddr_t routringTable[3];

//-----------------------------//



//----------instruction and data indications----------------
#define HELLOMSG 1
#define SLEEPMSG 0

#define INFORMATION 1
#define INSTRUCTION 0
//----------------------------------------------------


/***********************************************************************************
 * TYPEDEFS
 */
//---------------instruction and dataType-------------------------
typedef struct {
    uint8_t srcAddr[2];
    uint8_t hopCount;
    uint8_t instructionType;
}cumt_instruction;


typedef struct {
    uint8_t startAddr[2];
    uint8_t temperature[2];
}cumt_temperature;
//---------------------------------------------------------------



//-------address identification-------------
int is_broadcast_addr(uint8_t *addr);
int is_myAddr(uint8_t *addr);
//------------------------------------------

//----------------for input method--------------------
int sensor_incomingPacketProcessing(void);
int coord_incomingPacketProcessing(void);
//-------------------------------------------------

//------list processing-------------
void toPacketbufList(void *f, int datatogoLen);    //data should be ready to be sent, should not be called directly
//----------------------------------

//--------------for out going packet --------------
int buildBufflist(uint8_t* payload, int payloadLen,  rimeaddr_t nxthop);
int buildForwardInstructionList(void *payload, int payloadLen, rimeaddr_t nxthop);
void popList(void);
int sendPacket(void *datatogo, int datalen);
//csma time base
clock_time_t default_timebase(void);
//--------------------------------------------------

//-----------------for sensor--------------------
float getTemperature();
int temperatureInpack(rimeaddr_t nxthop);
//-----------------------------------------------

//----------------------sleep mode handler----------------------
void addSleepTimer(uint16_t sec);
void intoPM2Mode();

//-----------sleep flag processing--------------//
static int sleepingCMD=0;
int process_getSleepCMD();
void process_setSleepCMD(int state);
//----------------dongbo huang-----------------//

//--------------------------------------------------------------
