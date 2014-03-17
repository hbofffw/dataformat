/******created by dongbo huang*****/

#include "dataformat.h"


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
//#include "net/mac/csma.h"
#include "net/rime/route.h" //for routing table



#include "sys/ctimer.h"

#include "lib/random.h"

#include "debug.h"
#include "cc253x.h"
#include "sfr-bits.h"
#include "contiki-lib.h"
#include "contiki-net.h"

#include <stdio.h> /* For printf() */
#include <string.h>

#define DEBUG 0
#if DEBUG
#include "debug.h"
#define PUTSTRING(...) putstring(__VA_ARGS__)
#define PUTHEX(...) puthex(__VA_ARGS__)
#else
#define PUTSTRING(...)
#define PUTHEX(...)
#endif


//--------------------------------parameters setting--------------------------------//
//for sequence number
static uint8_t mac_dsn;

/***************reserved for complicated network situation********************/
//for broadcasting retransmission flag
static int helloRetrans_flag=0;
static int sleepRetrans_flag=0;
//hop count flag
//static int hopCount_flag=0;
//route table, maybe use pointer list?
//static tableContent route[3]; 
/****************************************************************************/
//in our case use only one address to up data
static rimeaddr_t upGoingNode;
//-------------------------

//-------parameters for packetbuf list handle---------
packetListStruct_t *packetBufCache;
//----------------------------------------------------

////////////////////////////////////end of parameters setting////////////////////////////////////////////




//--------------------------------function setting---------------------------------------//

//-------------------list processing-------------------------

void toOutgoingList(rimeaddr_t *datatogoAddr, void *s, int datatogoLen, int datatogoType)
{
    packetbufListStruct_t *cache;
    cache = memb_alloc(&packetbuf_memb);
    rimeaddr_cmp(cache->addr, dataotogoAddr);
    cache->bufCache = s;
    cache->dataLen = datatogoLen;
    cache->dataType = datatogoType;
    list_add(packetbuf_list, cache);
}
void extractFromList(list_t t)
{

}
//----------------------------------

//-------for address identification---------
int is_broadcast_addr(uint8_t *addr)
{
    int i = 2;
    while(i-- > 0) {
        if(addr[i] != 0xff) {
            return 0;
        }
    }
    return 1;
}
int is_myAddr(uint8_t *addr)
{
    int flag = 0;
    int i = 2;
    while(i-->0){
        if(addr[i]!=rimeaddr_node_addr.u8[i])
            flag += 0;
        else
            flag += 1;
    }
    return flag;
}
//------------------------------------------

//----------------for input method---------------
int sensor_incomingPacketProcessing()
{
    frame802154_t frame;
    int flag=0;                   //function return flag
    int len;
    len = cc2530_rf_driver.pending_packet();
    if(len)
    {
        packetbuf_clear();
        len = cc2530_rf_driver.read(packetbuf_dataptr(),PACKETBUF_SIZE);
        packetbuf_set_datalen(len);
        if(frame802154_parse(packetbuf_dataptr(), len, &frame) &&
                packetbuf_hdrreduce(len - frame.payload_len))
        {
            if(is_broadcast_addr(frame.dest_addr) && !is_myAddr(frame.src_addr))
            {
                if(helloRetrans_flag == 0)
                {
                    //remember the upgoing node address
                    rimeaddr_copy(&upGoingNode,(rimeaddr_t*) &frame.src_addr);
                    inst = (cumt_instruction*)frame.payload;
                    if(inst->instructionType == HELLOMSG)
                    {
                        helloRetrans_flag = 1;   //has already forwarded hello instruction, next time when hello instruction comes, it will drop it 
                        inst->hopCount++;
                        if(temperatureDataSend(upGoingNode))
                        {
                            PUTSTRING("temperature sent\r\n");
                            flag = 1;
                        }
                        //check if this node has retransmitted some packets, if no, retransmit, if once, only update the route table
                        if(retrans((uint8_t*)inst, sizeof(inst),rimeaddr_null))
                        {
                            inst = NULL;
                            flag = 1;
                        }
                    }
                }
                if(sleepRetrans_flag == 0)
                {
                    inst = (cumt_instruction *)frame.payload;
                    if(inst->instructionType == SLEEPMSG)
                    {
                        if(retrans((uint8_t *)inst,sizeof(inst),rimeaddr_null))
                        {
                            inst=NULL;
                            sleepRetrans_flag = 1;  //should be 1
                            flag = 1;
                            /***reserved for sleep processing***/
                            PUTSTRING("into sleep mode\r\n");
                            process_setSleepCMD(1);
                            sleepRetrans_flag = 0;
                            helloRetrans_flag = 0;
                        }
                    }
                }
            }
            if(rimeaddr_cmp((rimeaddr_t *)&frame.dest_addr, &rimeaddr_node_addr))
            {
                PUTSTRING("some one send data on me");
                cumt_temperature *forwardTmp;
                forwardTmp = (cumt_temperature *)frame.payload;
                retrans(frame.payload,frame.payload_len,upGoingNode);
                flag = 1;
            }
        }
    }            //data for me 

    return flag;
}

int coord_incomingPacketProcessing(void)
{
    
}
//-----------------------------------------------

//--------------for outgoing packet --------------
int prepareList(void *dataPnt, int dataLen)
{
}
int sendList(list_t bufList)
{
}
//--------------------------------------------------

//-----------------for sensor--------------------
float getTemperature()
{
    int rv;
    struct sensors_sensor * sensor;
    float sane = 0;
    sensor = (struct sensors_sensor *) sensors_find(ADC_SENSOR);
    if(sensor)
    {
        rv = sensor->value(ADC_SENSOR_TYPE_TEMP);
        if(rv != -1)
        {
            sane = 11 + ((rv - 1480) / 4.5);
            //sane = ((rv-1367.5)/4.5)-4;
            return sane;

        }
        else
        {
            PUTSTRING("get ADC value failed\r\n");
            return 0;
        }
    }
    else
    {
        PUTSTRING("sensor API load failed\r\n");
        return 0;
    }
}
int temperatureInpack(rimeaddr_t nxthop)
{
}
//-----------------------------------------------

//----------------------sleep mode handler----------------------
void addSleepTimer(uint16_t sec)
{
}
void intoPM2Mode()
{
}

//-----------sleep flag processing--------------//
int process_getSleepCMD()
{

}
void process_setSleepCMD(int state)
{

}

//----------------dongbo huang-----------------//

//--------------------------------------------------------------
