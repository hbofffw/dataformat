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

//----------instruction and data indications----------------
#define HELLOMSG 0x00 
#define SLEEPMSG 0xff      //here can be modified to adjust the sleep timer

//#define INFORMATION 1
//#define INSTRUCTION 0
//----------------------------------------------------



//---------------instruction and dataType----------------------
typedef struct {
    uint8_t srcAddr[2];
    uint8_t hopCount;
    uint8_t instructionType;
}cumt_instruction;


typedef struct {
    uint8_t startAddr[2];
    uint8_t temperature[2];
    uint8_t vdd[2];
}cumt_information;
//-------------------------------------------------------------


//-------------------------------------list handleing-----------------------------
//------------------------routing table ---------------------------
typedef struct routingTableStruct {
    struct routingTableStruct *next;
        int8_t rssiValue;
        uint8_t hopcount;
    rimeaddr_t *addr;
}routingTableStruct_t;

//---------------------------------------------------------------



//-----------------------buf struct------------------------------
typedef struct packetbufListStruct
{
    struct packetbufListStruct *next;
    uint8_t packetbuf[PACKETBUF_SIZE];
    int dataLen;
}packetbufListStruct_t;

//----------------------uploading struct-------------------------
typedef struct informationListStruct
{
    struct informationListStruct *next;
    cumt_information tmpData;
}informationListStruct_t;



//buf list
LIST(packetbuf_list);
MEMB(packetbuf_memb, struct packetbufListStruct, 5);

//for sink node communicating with computer
LIST(information_list);
MEMB(information_memb, struct informationListStruct,10);

//routing table list
LIST(routingTable_list);
MEMB(routingTable_memb, struct routingTableStruct,3);
//---------------------------end of list handling------------------------------




//-------address identification-------------
int is_broadcast_addr(uint8_t *addr);
int is_myAddr(uint8_t *addr);
//------------------------------------------

//----------------for input method--------------------
int sensor_incomingPacketProcessing(void);
int sink_incomingPacketProcessing(void);
//-------------------------------------------------

//------list processing---------------------------
//sensor
//void toPacketbufList(void *f, int datatogoLen);    //data should be ready to be sent, should not be called directly
int buildBufflist(uint8_t* payload, int payloadLen,  rimeaddr_t nxthop);
int sensor_popAndSendItemOfList(void);
int getpacketbufListLength(void);
//sink
void toInformationList(cumt_information tmp);
void sink_printItemOfList(void);
int getInformationListLength(void);

//---------------------------------------------------

//--------------for out going packet --------------

//for sensor to handle the sleep instruction 
int sendPacket(void *datatogo, int datalen);
int broadcastForward(uint8_t* overAir, int payloadLen);
//csma time base
clock_time_t default_timebase(void);
//for sink
int buildAndSendFrame(uint8_t* payload, int payloadLen,  rimeaddr_t nxthop);
int instructionSend(uint8_t instructin);
//--------------------------------------------------

//-----------------for sensing--------------------
float getTemperature();
float getVoltage();
void sensor_dataInpack(rimeaddr_t nxthop);
//-----------------------------------------------

//----------------------sleep mode handler----------------------
void addSleepTimer(uint16_t sec);
void intoPM2Mode();

//-----------sleep flag processing--------------//
static int sleepingCMD=0;
int process_getSleepCMD();
void process_setSleepCMD(int state);

//--------------------------------------------------------------
