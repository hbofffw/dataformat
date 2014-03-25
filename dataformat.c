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

#define DEBUG 0  //default :0 by dongbo huang
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
packetbufListStruct_t *packetBufCache;
//----------------------------------------------------

////////////////////////////////////end of parameters setting////////////////////////////////////////////




//--------------------------------function setting---------------------------------------//



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
                    cumt_instruction *inst;
                    //remember the upgoing node address
                    rimeaddr_copy(&upGoingNode,(rimeaddr_t*) &frame.src_addr);
                    inst = (cumt_instruction*)frame.payload;
                    if(inst->instructionType == HELLOMSG)
                    {
                        PUTSTRING("hello received!\r\n");
                        helloRetrans_flag = 1;   //has already forwarded hello instruction, next time when hello instruction comes, it will drop it 
                        inst->hopCount++;
                        //editing start here
                        temperatureInpack(upGoingNode);
                        if(buildBufflist((uint8_t *)inst, sizeof(*inst), rimeaddr_null))
                        {
                            helloRetrans_flag = 1;
                        }
                        inst = NULL;
                    }
                }
                if(sleepRetrans_flag == 0)
                {
                    cumt_instruction *inst;
                    inst = (cumt_instruction *)frame.payload;
                    PUTSTRING("sleep cmd received! \r\n");
                    if(inst->instructionType == SLEEPMSG)
                    {
                        if(broadcastForward(frame.payload, frame.payload_len));
                        {
                            inst=NULL;
                            sleepRetrans_flag = 1;  //should be 1
                            flag = 1;
                            PUTSTRING("into sleep mode\r\n");
                            process_setSleepCMD(1);
                        }
                    }
                }
                if(rimeaddr_cmp((rimeaddr_t *)&frame.dest_addr, &rimeaddr_node_addr))
                {
                    PUTSTRING("some one send data on me");
                    if(buildBufflist(frame.payload, frame.payload_len, upGoingNode))
                    {
                        flag = 1;
                    }
                }
            }
        }            //data for me 
        packetbuf_clear();
    }
    return flag;
}

int coord_incomingPacketProcessing(void)
{
    int len;
    int flag=0;
    len = cc2530_rf_driver.pending_packet();
    if(len)
    {
        frame802154_t frame;
        cumt_temperature* sensorData;
        //PUTSTRING("packet income\r\n");
        packetbuf_clear();
        len = cc2530_rf_driver.read(packetbuf_dataptr(),PACKETBUF_SIZE);
        packetbuf_set_datalen(len);
        if(frame802154_parse(packetbuf_dataptr(), packetbuf_datalen(), &frame) && packetbuf_hdrreduce(packetbuf_datalen() - frame.payload_len))
        {
            if(rimeaddr_cmp((rimeaddr_t *)&frame.dest_addr, &rimeaddr_node_addr)&& !rimeaddr_cmp((rimeaddr_t *)&frame.src_addr, &rimeaddr_node_addr))
            {
                sensorData = (cumt_temperature *)frame.payload;
                toInformationList(*sensorData);
                packetbuf_clear();
                flag = 1;
            }
        }
    }
    return flag;
}

//-----------------------------------------------

//-------------------list processing-------------------------

/*void toPacketbufList(void *f, int datatogoLen)*/
/*{*/
/*packetbufListStruct_t *cache;*/
/*cache = memb_alloc(&packetbuf_memb);*/
/*cache->packetbuf = f;*/
/*cache->dataLen = datatogoLen;*/
/*list_add(packetbuf_list, cache);*/
/*cache=NULL;*/
/*}*/
/*//----------------------------------*/

//--------------for outgoing packet --------------
//sensor 
int buildBufflist(uint8_t *payload, int payload_len, rimeaddr_t nxthop)
{
    int flag=0;
    int hdrlen;
    packetbuf_clear();
    packetbuf_copyfrom(payload,payload_len);
    frame802154_t frm;
    frm.fcf.src_addr_mode = FRAME802154_SHORTADDRMODE;
    frm.fcf.dest_addr_mode = FRAME802154_SHORTADDRMODE;
    frm.fcf.frame_type = FRAME802154_DATAFRAME;
    frm.fcf.security_enabled = 0;
    frm.fcf.frame_pending = 1;
    frm.fcf.panid_compression = 0;
    frm.fcf.frame_version = FRAME802154_IEEE802154_2003; // edited by dongbo huang: FRAME802154_IEEE802154_2006, another choice: FRAME802154_IEEE802154_2003
    if(mac_dsn == 0)
    {
        mac_dsn = random_rand() % 256;
    }
    else
    {
        mac_dsn++;
    }

    frm.seq=mac_dsn;
    frm.dest_pid = IEEE802154_PANID;
    frm.src_pid = IEEE802154_PANID;
    if (rimeaddr_cmp(&nxthop, &rimeaddr_null))
    {
        frm.fcf.ack_required = 0;
        frm.dest_addr[0] = 0xFF;
        frm.dest_addr[1] = 0xFF;
    }
    else
    {
        frm.fcf.ack_required = 1;
        rimeaddr_copy((rimeaddr_t *)&frm.dest_addr, &nxthop);  
    }

    rimeaddr_copy((rimeaddr_t *)&frm.src_addr, &rimeaddr_node_addr);
    frm.payload = packetbuf_dataptr();
    frm.payload_len = packetbuf_datalen();
    hdrlen = frame802154_hdrlen(&frm);
    if(packetbuf_hdralloc(hdrlen))
    {
        //-----------------create the output buff, add it to the list-------------------//
        //for list item operation and mem allocation
        packetbufListStruct_t *cache;
        cache = memb_alloc(&packetbuf_memb);
        //
        frame802154_create(&frm, packetbuf_hdrptr(), hdrlen);
        int i;
        uint8_t *bufTmp;
        //memset(bufTmp, 0, packetbuf_totlen());
        bufTmp = (uint8_t *)packetbuf_hdrptr();
        for(i = 0; i<packetbuf_totlen(); i++)
        {
            cache->packetbuf[i] = bufTmp[i]; 
        }
        cache->dataLen = packetbuf_totlen();
        list_add(packetbuf_list,cache);
        //---------------------------------------------------------------------//
        packetbuf_clear();
        flag=1;
    }
    return flag;
}

//sensor send one item in the list
int sensor_popAndSendItemOfList()
{
    int flag = 0;
    packetbufListStruct_t *pop;
    if(list_length(packetbuf_list) != 0)
        pop = (packetbufListStruct_t *)list_pop(packetbuf_list);
    if(sendPacket(pop->packetbuf, pop->dataLen))
    {
        flag = 1;
    }
    /*pop = NULL;*/
    memb_free(&packetbuf_memb, pop);
    return flag;
}

//for coordinator to handel temperature packets, CAN NOT be directly called
void toInformationList(cumt_temperature tmp) 
{
    informationListStruct_t *cache;
    cache = memb_alloc(&information_memb);
    cache->tmpData = tmp;
    list_add(information_list, cache);
}

//print item from list
void coord_printItemOfList(void)
{
    cumt_temperature sensorData;
    informationListStruct_t *pop;
    if(list_length(information_list) != 0)
        pop = (informationListStruct_t *)list_pop(information_list);
    sensorData = pop->tmpData;
    putstring("Frome node ");
    puthex(sensorData.startAddr[0]);
    puthex(sensorData.startAddr[1]);
    putstring(": ");
    printf("temperature=%d.%d\r\n", sensorData.temperature[0], sensorData.temperature[1]);
    memb_free(&information_memb, pop);
}


//for sensor or coord know the length of corresponding list
int getpacketbufListLength(void)
{
    return list_length(packetbuf_list);
}
int getInformationListLength(void)
{
    return list_length(information_list);
}
//------------------------------------------------------------------------------------

//----------------------------for outgoing packet--------------------------------------
//CAN NOT be directly called
int sendPacket(void *datatogo, int datalen)
{
    int flag=0;
    clock_time_t waitTime;
    int backoffs=0;
    int sendStatus;
    waitTime = CLOCK_SECOND/NETSTACK_RDC_CHANNEL_CHECK_RATE;
    do
    {
        while(cc2530_rf_driver.channel_clear() == 0)
        {
            waitTime = waitTime + (random_rand()%(backoffs * waitTime));
            backoffs ++;
            clock_wait(waitTime);
        }
        sendStatus = cc2530_rf_driver.send(datatogo, datalen);
        if(backoffs >5)
            backoffs=5;
        if(sendStatus == RADIO_TX_OK)
        {
            backoffs = 0;
            flag = 1;
            //indicate the status is OK
            leds_on(LEDS_RED);
        }
        else
        {
            PUTSTRING("FAILED TO SEND TRY AGAIN\r\n");
            waitTime = waitTime + (random_rand()%(backoffs * waitTime));
            backoffs++;
            clock_wait(waitTime);
        }
    }while(sendStatus != RADIO_TX_OK);
    return flag;
}
//CAN NOT be directly called
int broadcastForward(uint8_t* overAir, int payloadLen)
{
    if(buildAndSendFrame(overAir,payloadLen,rimeaddr_null))
        return 1;
    else
        return 0;
}

clock_time_t default_timebase(void)
{
    clock_time_t time=0;
    /* The retransmission time must be proportional to the channel
       check interval of the underlying radio duty cycling layer.*/
    //   time = NETSTACK_RDC.channel_check_interval();

    /* If the radio duty cycle has no channel check interval (i.e., it
       does not turn the radio off), we make the retransmission time
       proportional to the configured MAC channel check rate.*/
    if(time == 0) {
        time = CLOCK_SECOND / NETSTACK_RDC_CHANNEL_CHECK_RATE;
    }
    return time;
}

//for corrd
int buildAndSendFrame(uint8_t* payload, int payloadLen,  rimeaddr_t nxthop)
{
    int flag=0;
    int len;
    int sendStatus;

    clock_time_t waitTime; 
    int backoffs = 0;
    packetbuf_clear();
    packetbuf_copyfrom(payload,payloadLen);
    frame802154_t frm;
    frm.fcf.src_addr_mode = FRAME802154_SHORTADDRMODE;
    frm.fcf.dest_addr_mode = FRAME802154_SHORTADDRMODE;
    frm.fcf.frame_type = FRAME802154_DATAFRAME;
    frm.fcf.security_enabled = 0;
    frm.fcf.frame_pending = 1;
    frm.fcf.panid_compression = 0;
    frm.fcf.frame_version = FRAME802154_IEEE802154_2003; // edited by dongbo huang: FRAME802154_IEEE802154_2006, another choice: FRAME802154_IEEE802154_2003
    if(mac_dsn == 0)
    {
        mac_dsn = random_rand() % 256;
    }
    else
    {
        mac_dsn++;
    }
    frm.seq=mac_dsn;
    frm.dest_pid = IEEE802154_PANID;
    frm.src_pid = IEEE802154_PANID;
    if (rimeaddr_cmp(&nxthop, &rimeaddr_null))
    {
        frm.fcf.ack_required = 0;
        frm.dest_addr[0] = 0xFF;
        frm.dest_addr[1] = 0xFF;
    }
    else
    {
        frm.fcf.ack_required = 1;
        rimeaddr_copy((rimeaddr_t *)&frm.dest_addr, &nxthop);  
    }

    rimeaddr_copy((rimeaddr_t *)&frm.src_addr, &rimeaddr_node_addr);
    frm.payload = packetbuf_dataptr();
    frm.payload_len = packetbuf_datalen();
    //this is a test 
    len = frame802154_hdrlen(&frm);
    if(packetbuf_hdralloc(len))
    {
        frame802154_create(&frm, packetbuf_hdrptr(), len);
        if (rimeaddr_cmp(&nxthop, &rimeaddr_null))
        {
            waitTime = CLOCK_SECOND/NETSTACK_RDC_CHANNEL_CHECK_RATE;
            do
            {
                while(cc2530_rf_driver.channel_clear() == 0)
                {
                    waitTime = waitTime + (random_rand()%(backoffs * waitTime));
                    backoffs ++;
                    clock_wait(waitTime);
                }
                sendStatus = cc2530_rf_driver.send(packetbuf_hdrptr(), packetbuf_totlen());
                if(backoffs >5)
                    backoffs=5;
                if(sendStatus == RADIO_TX_OK)
                {
                    backoffs = 0;
                    flag = 1;
                }
                else
                {
                    PUTSTRING("FAILED TO SEND TRY AGAIN\r\n");
                    waitTime = waitTime + (random_rand()%(backoffs * waitTime));
                    backoffs++;
                    clock_wait(waitTime);
                }
            }while(sendStatus != RADIO_TX_OK);
        }
        else
        {
            waitTime = CLOCK_SECOND/NETSTACK_RDC_CHANNEL_CHECK_RATE;
            do
            {
                while(cc2530_rf_driver.channel_clear() == 0)
                {
                    waitTime = waitTime + (random_rand()%(backoffs * waitTime));
                    backoffs ++;
                    clock_wait(waitTime);
                }
                sendStatus = cc2530_rf_driver.send(packetbuf_hdrptr(), packetbuf_totlen());
                if(backoffs >5)
                    backoffs=5;
                if(sendStatus == RADIO_TX_OK)
                {
                    backoffs = 0;
                    flag = 1;
                }
                else
                {
                    PUTSTRING("FAILED TO SEND TRY AGAIN\r\n");
                    waitTime = waitTime + (random_rand()%(backoffs * waitTime));
                    backoffs++;
                    clock_wait(waitTime);
                }
            }while(sendStatus != RADIO_TX_OK);
        }
    }
    return flag;
}
int instructionSend(uint8_t instruction)
{
    static cumt_instruction instruc;
    //    memcpy(instruc.srcAddr,rimeaddr_node_addr.u8,sizeof(rimeaddr_node_addr.u8));
    instruc.srcAddr[0] = rimeaddr_node_addr.u8[0];
    instruc.srcAddr[1] = rimeaddr_node_addr.u8[1];
    instruc.hopCount = 0;
    instruc.instructionType = instruction;
    if(buildAndSendFrame((uint8_t *) &instruc,sizeof(instruc),  rimeaddr_null))
        return 1;
    else
        return 0;
}

//--------------------------------------------------

//-----------------sensing--------------------
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
void temperatureInpack(rimeaddr_t nxthop)
{
    cumt_temperature temperaturePkt;
    float tmp;
    uint8_t tmpInt;
    uint8_t tmpPnt;
    temperaturePkt.startAddr[0] = rimeaddr_node_addr.u8[0];
    temperaturePkt.startAddr[1] = rimeaddr_node_addr.u8[1];
    tmp= getTemperature();
    tmpInt = (uint8_t)tmp;
    tmpPnt = (uint8_t)(tmp*100)%100;
    temperaturePkt.temperature[0] = tmpInt;
    temperaturePkt.temperature[1] = tmpPnt;
    buildBufflist((uint8_t *)&temperaturePkt, sizeof(temperaturePkt), upGoingNode);
}
//-----------------------------------------------

//----------------------sleep mode handler----------------------
void addSleepTimer(uint16_t sec)
{
    uint32_t sleepTimer = 0;
    sleepTimer |= ST0;
    sleepTimer |= (uint32_t)ST1 <<  8;
    sleepTimer |= (uint32_t)ST2 << 16;
    sleepTimer += ((uint32_t)sec * (uint32_t)32768);
    ST2 = (uint8_t)(sleepTimer >> 16);
    ST1 = (uint8_t)(sleepTimer >> 8);
    ST0 = (uint8_t) sleepTimer;
}
void intoPM2Mode()
{
    helloRetrans_flag = 0;
    sleepRetrans_flag = 0;
    int j;
    int mode = 2;
    SLEEPCMD &= 0xFC;
    SLEEPCMD |= mode;
    for(j=0;j<4;j++);
    PCON = 0x01;    
}

//-----------sleep flag processing--------------//
int process_getSleepCMD()
{
    return sleepingCMD;
}
void process_setSleepCMD(int state)
{
    sleepingCMD = state;
}

//----------------dongbo huang-----------------//

//--------------------------------------------------------------
