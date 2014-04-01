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
//-----------------dongbo huang-----------------------


/*void Delay(int n)*/
/*{*/
    /*int jj;*/
    /*for(jj=0;jj<n;jj++);*/
    /*for(jj=0;jj<n;jj++);*/
    /*for(jj=0;jj<n;jj++);*/
    /*for(jj=0;jj<n;jj++);*/
    /*for(jj=0;jj<n;jj++);*/
/*}*/

//----------------------------



int buildAndSendFrame(uint8_t * payload, int payloadLen,  rimeaddr_t nxthop)
{
    int flag=0;
    int len;
    int status;

    clock_time_t waitTime; 
    int backoffs = 0;
    packetbuf_clear();
    packetbuf_copyfrom(payload,payloadLen);
    frame802154_t frm;
    //memset(&frm, 0, sizeof(frm));

    //packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, &nxthop);
    //packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &rimeaddr_node_addr);
    //packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK,1);
    //packetbuf_set_attr(PACKETBUF_ATTR_PENDING,1);

    /*if(framer_802154.create()!=FRAMER_FAILED)*/
    /*{*/
    /*PUTSTRING("frame created success!\r\n");*/
    /*}*/
    /*else{*/
    /*PUTSTRING("frame created failed!\r\n");*/
    /*}*/
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
            while(1)
            {
                waitTime = CLOCK_SECOND / NETSTACK_RDC_CHANNEL_CHECK_RATE;
                while(cc2530_rf_driver.channel_clear() ==0)
                {   
                    waitTime = waitTime + (random_rand()%(backoffs*waitTime));
                    backoffs++;	
                    clock_wait(waitTime);
                }
                status = cc2530_rf_driver.send(packetbuf_hdrptr(),packetbuf_totlen());
                if(status == RADIO_TX_OK)
                {	
                    backoffs = 0;
                    flag = 1;
                    break;
                }
                else if(status == RADIO_TX_NOACK)
                {
                    PUTSTRING("no ack\r\n");
                    waitTime = waitTime+(random_rand()%(backoffs * waitTime));
                    backoffs++;
                    clock_wait(waitTime);
                    continue;
                }
                else
                {
                    PUTSTRING("collision happened! \r\n"); 
                    waitTime = waitTime+(random_rand()%(backoffs * waitTime));
                    backoffs++;
                    clock_wait(waitTime);
                    continue;
                }
            }
        }
        else
        {
            while(1)

            {
                waitTime = CLOCK_SECOND / NETSTACK_RDC_CHANNEL_CHECK_RATE;
                while(cc2530_rf_driver.channel_clear() ==0)
                {   
                    waitTime = waitTime + (random_rand()%(backoffs*waitTime));
                    backoffs++;	
                    clock_wait(waitTime);
                }
                status = cc2530_rf_driver.send(packetbuf_hdrptr(),packetbuf_totlen());
                if(backoffs > 5)
                    backoffs = 3;

                if(status == RADIO_TX_OK)
                {	
                    backoffs = 0;
                    flag = 1;
                    break;
                }
                else if(status == RADIO_TX_NOACK)
                {
                    continue;
                }
                else
                {
                    PUTSTRING("no ack! \r\n"); 
                    backoffs++; 
                    waitTime = waitTime+(random_rand()%(backoffs * waitTime));
                    clock_wait(waitTime);
                    continue;
                }
            }
        }
    }
    return flag;
}

/***********************for coord instructions********************/

int instructionSend(uint8_t instruction)
{
    //get_self_addr();
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


/************************************************************/

/*********************for sensor temperature data************************/ 


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


int temperatureDataSend(rimeaddr_t nxthop)
{
    //get_self_addr();
    cumt_information temperaturePkt;
    float tmp;
    uint8_t tmpInt;
    uint8_t tmpPnt;
    //char temperatureValue[6];
    temperaturePkt.startAddr[0] = rimeaddr_node_addr.u8[0];
    temperaturePkt.startAddr[1] = rimeaddr_node_addr.u8[1];
    /*temperaturePkt.finalAddr[0] = finalAddr[0];*/
    /*temperaturePkt.finalAddr[1] = finalAddr[1];*/
    tmp= getTemperature();
    tmpInt = (uint8_t)tmp;
    tmpPnt = (uint8_t)(tmp*100)%100;
    temperaturePkt.temperature[0] = tmpInt;
    temperaturePkt.temperature[1] = tmpPnt;
    printf("temperature is : %d.%d\r\n",temperaturePkt.temperature[0], temperaturePkt.temperature[1]);
    /*temperatureValue[0] = (uint8_t)(tmp)/10+48;*/
    /*temperatureValue[1] = (uint8_t)(tmp)%10+48;*/
    /*temperatureValue[2] = '.';*/
    /*temperatureValue[3] = (uint8_t)(tmp*10)%10+48;*/
    /*temperatureValue[4] = (uint8_t)(tmp*100)%10+48;*/
    /*temperatureValue[5] = '\0';*/
    /*printf("temperature is %s\r\n",temperatureValue);*/
    if(buildAndSendFrame((uint8_t*) &temperaturePkt,sizeof(temperaturePkt), nxthop))
    {
        leds_on(LEDS_RED);
        return 1;
    }
    else
        return 0;
}

//for data packet retransmission

int retrans(uint8_t* overAir, int payloadLen, rimeaddr_t nxthop)
{
    if(buildAndSendFrame(overAir,payloadLen,nxthop))
        return 1;
    else
        return 0;
}




/**********************************************************/

/*********************input data functions*********************/

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

int sensor_incomingPktProcessing(void)
{	
    //get_self_addr();	
    //    rimeaddr_t nexthop; //use static address 
    frame802154_t frame;
    cumt_instruction* inst;
    int flag=0;                   //function return flag
    int len=0;
	len = cc2530_rf_driver.pending_packet();
    if(len)
    {
        packetbuf_clear();
        len = cc2530_rf_driver.read(packetbuf_dataptr(),PACKETBUF_SIZE);
        packetbuf_set_datalen(len);
        //memset(&frame,0,sizeof(frame));
        if(frame802154_parse(packetbuf_dataptr(), len, &frame) &&
                packetbuf_hdrreduce(len - frame.payload_len))
        {
            //decide the output packet pattern, build the own temeperature data packet or just restransmit the incoming packet
            /*framer_802154.parse();*/
            if(is_broadcast_addr(frame.dest_addr) && !is_myAddr(frame.src_addr))
            {
                if(helloRetrans_flag == 0)
                {
                    //remember the upgoing node address
                    rimeaddr_copy(&upGoingNode,(rimeaddr_t*) &frame.src_addr);
                    inst = (cumt_instruction*)frame.payload;
                    //memcpy(&inst,(cumt_instruction*)frame.payload,frame.payload_len);
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
                            //Delay(1000);
                            //cc2530_rf_driver.off();

                            //addSleepTimer(10);
                            //intoPM2Mode();
                            //cc2530_rf_driver.on();
                            //Delay(1000);

                            //initial the flags
                            //PUTSTRING("wake up\r\n");
                            //cc2530_rf_driver.init();
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
                cumt_information *forwardTmp;
                forwardTmp = (cumt_information *)frame.payload;
                retrans(frame.payload,frame.payload_len,upGoingNode);
                flag = 1;
            }
        }
    }            //data for me 

    return flag;
}

int coord_incomingPktProcessing(void)
{
    frame802154_t frame;
    cumt_information* sensorData;
    int len;
    //----handle temperature---
    /*unsigned short tmpInt;*/
    /*unsigned short tmpPnt;*/
    //----dongbo huang---------
    int flag=0;
    len = cc2530_rf_driver.pending_packet();
    if(len)
    {
        //PUTSTRING("packet income\r\n");
        packetbuf_clear();
        len = cc2530_rf_driver.read(packetbuf_dataptr(),PACKETBUF_SIZE);
        packetbuf_set_datalen(len);
        if(frame802154_parse(packetbuf_dataptr(), packetbuf_datalen(), &frame) && packetbuf_hdrreduce(packetbuf_datalen() - frame.payload_len))
        {
            if(rimeaddr_cmp((rimeaddr_t *)&frame.dest_addr, &rimeaddr_node_addr)&& !rimeaddr_cmp((rimeaddr_t *)&frame.src_addr, &rimeaddr_node_addr))
            {
                sensorData = (cumt_information *)frame.payload;
                putstring("Frome node ");
                puthex(sensorData->startAddr[0]);
                puthex(sensorData->startAddr[1]);
                putstring(": ");

                printf("temperature=%d.%d \r\n",  sensorData->temperature[0], sensorData->temperature[1]);
                packetbuf_clear();
                flag = 1;
            }
        }
    }
    return flag;
}

/*void addAddress(rimeaddr_t *addr)*/
/*{*/

/*}*/




clock_time_t default_timebase(void)
{
    clock_time_t time=0;
    /* The retransmission time must be proportional to the channel
       check interval of the underlying radio duty cycling layer. */
    //   time = NETSTACK_RDC.channel_check_interval();

    /* If the radio duty cycle has no channel check interval (i.e., it
       does not turn the radio off), we make the retransmission time
       proportional to the configured MAC channel check rate. */
    if(time == 0) {
        time = CLOCK_SECOND / NETSTACK_RDC_CHANNEL_CHECK_RATE;
    }
    return time;
}


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

//-----------sleep flag processing----------

int process_getSleepCMD()
{
    return sleepingCMD;
}
void process_setSleepCMD(int state)
{
    sleepingCMD = state;
}

void intoPM2Mode()
{

    int j;
    int mode = 2;
    SLEEPCMD &= 0xFC;
    SLEEPCMD |= mode;
    for(j=0;j<4;j++);
    PCON = 0x01;   
}


//------------dongbo huang---------------------
//------------list handler------------------
void addIntoPacketList(packetListStruct_t *s)
{
    packetListStruct_t *cache;
    cache = memb_alloc(&packetBuf_memb);
    cache = s;
    list_add(packetBuf_list, cache);
    cache = NULL;
}
//-------------dongbo huang-----------------
