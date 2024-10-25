/*
 * ModemControl.c
 *
 *  Created on: 06.04.2020.
 *      Author: fademike
 */



// #include <string.h>
// #include <stdio.h>

#include "ModemControl.h"
#include "cb.h"

void ModemControl_setAnswer(uint8_t t_NeedAnswer, int32_t t_delay_toWaitAnswer);
uint8_t CRC_PacketCalculate(uint8_t * buff);
void CRC_PacketSet(uint8_t * buff);
int8_t CRC_PacketCheck(uint8_t * buff);
void ModemControl_timeUpdate(void);



#ifdef SI4463
extern struct modem_struct rf_si4463;
struct modem_struct * rf_modem = &rf_si4463;
// #endif
#elif defined (nRF24)
// #ifdef nRF24
extern struct modem_struct rf_nrf24;
struct modem_struct * rf_modem = &rf_nrf24;
#endif

static inline void my_memcpy(uint8_t * a, uint8_t * b, uint16_t len){
    int i=0;
    for (i=0;i<len;i++) a[i] = b[i];
}

// volatile uint8_t pack_counter = 0;

static int ModemControl_status = -1;

circular_buffer toTx, toRx;

//  Variables of modem
volatile int32_t delay_synchro = 0;
volatile int32_t delay_toWaitAnswer = 0;
volatile int32_t delay_measureTx = 0;
volatile int32_t delay_afterSendSymbol = 0;
uint8_t NeedAnswer = 0;

uint8_t mode_blockPacket = SEND_PACKETBLOCK;
uint8_t mode_synchro = SYNCHRO_MODE;

uint8_t rfPack_getType(uint8_t * buff){
  return buff[0];
}
void rfPack_setType(uint8_t * buff, uint8_t type){
  buff[0] = type;
}
uint8_t rfPack_getLen(uint8_t * buff){
  uint8_t len = buff[1]&0x3F;
	if (len>PACKET_DATALEN) len = PACKET_DATALEN;
  return len;
}
int rfPack_setLen(uint8_t * buff, uint16_t len){
	if (len>PACKET_DATALEN) len = PACKET_DATALEN;
  buff[1] = len;
  return len;
}
uint8_t * rfPack_getDataPtr(uint8_t * buff, uint16_t n){
  return &buff[2+n];
}
void rfPack_setData(uint8_t * buff, uint16_t n, uint8_t data){
  buff[2+n] = data;
}

int32_t ModemControl_SendPacket_GetQueue(void)
{
  if (cb_isFull(&toTx)) return -1;
  else if (cb_isEmpty(&toTx)) return 0;
  return cb_getCnt(&toTx);
}
int32_t ModemControl_SendPacket_SizeQueue(void)
{
  return cb_getSize(&toTx);
}
// Copies the packet to the buffer for sending
void ModemControl_SendPacket(uint8_t * buff, uint16_t len)
{
  uint8_t * pack = cb_getBuf(&toTx, cb_WR);
  len = rfPack_setLen(pack, len);
	my_memcpy(rfPack_getDataPtr(pack, 0), buff, len); // copy packet for send
	cb_pass(&toTx, cb_WR);
}

volatile uint16_t ptr_SendSymbol=0; // pointer to fill in the block package
void ModemControl_SendSymbol_Complete(void)
{
  uint8_t * pack = cb_getBuf(&toTx, cb_WR);
  rfPack_setLen(pack, ptr_SendSymbol);
	cb_pass(&toTx, cb_WR);  // increase painter for send
	ptr_SendSymbol=0;
}
void ModemControl_SendSymbol(uint8_t data)
{
  rfPack_setData(cb_getBuf(&toTx, cb_WR), ptr_SendSymbol++, data);
	if (ptr_SendSymbol>=PACKET_DATALEN){ // if the pack is full
		ModemControl_SendSymbol_Complete();
	}
  ModemControl_timeUpdate();
	delay_afterSendSymbol=0;
}
// If there is a package, puts the package in the pointer (buf)
// Returned len pack if there is packet. 0 no packet
int32_t ModemControl_GetPacket(uint8_t * buf){
    if (cb_isEmpty(&toRx)) return 0;

    uint8_t * pack = cb_getBuf(&toRx, cb_RD);
    uint8_t len = rfPack_getLen(pack);
    my_memcpy(buf, rfPack_getDataPtr(pack, 0), len);
    cb_pass(&toRx, cb_RD);
    return len;
}
// Reads received packets and outputs one byte to pointer (buf)
// Returned 1 if there is data. 0 no data
int32_t ModemControl_GetByte(uint8_t * buf){     //FIXME !!!
    static int pos = 0;
    int read = 0; // flag of return data
    if (cb_isEmpty(&toRx)) {pos=0; return 0;}

    uint8_t * pack = cb_getBuf(&toRx, cb_RD);
    uint8_t len = rfPack_getLen(pack);//pack->len&0x3F;
    if (len > PACKET_DATALEN) len=PACKET_LEN;
    if (len>pos){
        *buf = *rfPack_getDataPtr(pack, pos);
        pos++;
        read = 1;
    }
    if (pos >= len){  // all bytes in this package geting 
        pos = 0;
        cb_pass(&toRx, cb_RD);
    }
    return read;
}

// return calculated crc 
uint8_t CRC_PacketCalculate(uint8_t * buff){
	uint8_t mCRC = 87;  // start byte for calculate my crc
	int x=0;
	for (x=0;x<((PACKET_LEN-1)-PACKET_LEN_CRC);x++)mCRC += buff[x]*3; // algorithm of calculation
	return mCRC;
}
// write calculated crc in pack
void CRC_PacketSet(uint8_t * buff){
	buff[(PACKET_LEN-1)-PACKET_LEN_CRC] = CRC_PacketCalculate(buff);
}
// check crc pack
int8_t CRC_PacketCheck(uint8_t * buff){
	return  (CRC_PacketCalculate(buff) == buff[(PACKET_LEN-1)-PACKET_LEN_CRC]);
}

void ModemControl_setAnswer(uint8_t t_NeedAnswer, int32_t t_delay_toWaitAnswer){
  NeedAnswer = t_NeedAnswer;
  delay_toWaitAnswer = t_delay_toWaitAnswer;
}

void ModemControl_timeUpdate(void){
  static uint32_t localTime_ms = 0;
  uint32_t realTime_ms = GetTime_ms();

  uint32_t dt_ms = realTime_ms - localTime_ms;
  if (localTime_ms > realTime_ms) dt_ms = 0;
  localTime_ms = realTime_ms;

  if (!mode_synchro) return;
  if (dt_ms<1) return;

  if (delay_synchro>0) delay_synchro -= dt_ms;
  if (delay_toWaitAnswer>0) delay_toWaitAnswer -= dt_ms;
  if (delay_measureTx<100) delay_measureTx += dt_ms; //limit calculate tx is 100 ms
  if (delay_afterSendSymbol<100) delay_afterSendSymbol += dt_ms;
}

int32_t ModemControl_init(void){

  #define BSIZE CMD_LIST_SIZE // buff size
  #define PSIZE PACKET_LEN    // pack size

  static uint8_t buf_tx[(BSIZE*sizeof(uint16_t)) + BSIZE*PSIZE];
  static uint8_t buf_rx[(BSIZE*sizeof(uint16_t)) + BSIZE*PSIZE];
  cb_init_static(&toTx, BSIZE, PSIZE, buf_tx);
  cb_init_static(&toRx, BSIZE, PSIZE, buf_rx);

  ModemControl_status = rf_modem->init();//RFinit();
  return ModemControl_status;
}

int32_t ModemControl_getStatus(void){
	return ModemControl_status;
}

int32_t ModemControl_Read(void){

  uint8_t * cPack = cb_getBuf(&toRx, cb_WR);

  int32_t ret = rf_modem->read(cPack, PACKET_LEN);
  if (ret < 0) ModemControl_status = ret;
  if (ret <= 0) return 0;
  
  uint8_t type = rfPack_getType(cPack);

  if (mode_synchro){
    if (type == PACKET_NONE )         ModemControl_setAnswer(PACKET_NONE,   0);
    else if (type == PACKET_ASK )     ModemControl_setAnswer(PACKET_ANSWER, 0);
    else if (type == PACKET_ANSWER )  ModemControl_setAnswer(PACKET_NONE,   0);
    else if (type == PACKET_DATA)     ModemControl_setAnswer(PACKET_ANSWER, 0);
    else if (type == PACKET_DATA_PART)ModemControl_setAnswer(PACKET_NONE,   SYNCHRO_TWAIT);
  }

  if ((type == PACKET_DATA) || (type == PACKET_DATA_PART)){         // if Packet is PACKET_DATA
    uint8_t len = rfPack_getLen(cPack);  // datalen of package
    if (len){ // if no zero len
      if 	(1){//(CRC_PacketCheck(rbuff)){ //TODO !!!
        cb_pass(&toRx, cb_WR);
      }
    }
  }
  return 1;
}

void ModemControl_Send(uint8_t * buf){

  ModemControl_timeUpdate();
  delay_measureTx = 0;  // reset timer to caltulate time tx pack

  ModemControl_status = rf_modem->write(buf, PACKET_LEN);   // TX Data

  if (!mode_synchro) return;

  ModemControl_timeUpdate();
  if (delay_measureTx <= 0)delay_measureTx = 1;  // time tx minimum 1ms
  ModemControl_setAnswer(PACKET_NONE, delay_measureTx+SYNCHRO_TWAIT);
}

int32_t ModemControl_Loop(void)
{
  if (ModemControl_status < 0) {ModemControl_init(); return -1;}
  ModemControl_timeUpdate();

  if (ModemControl_Read() != 0) return 1; // if you have read something.. go back and read again

  if (ptr_SendSymbol>0) { //if to have symbols to send
    if (delay_afterSendSymbol > 2) ModemControl_SendSymbol_Complete();  // prepare to tx, when no new data 2ms
  }

  if ((mode_synchro) && (delay_toWaitAnswer > 0)){return 0;}      //The timer between the packets being sent. Otherwise, heap data was not accepted.


  if(!cb_isEmpty(&toTx)){ // if buffer is not empty

    uint8_t blockPack_cnt = mode_blockPacket; // max send pack in block
    do{

      if ((cb_getCnt(&toTx)>1) && (blockPack_cnt)) rfPack_setType(cb_getBuf(&toTx, cb_RD), PACKET_DATA_PART);
      else rfPack_setType(cb_getBuf(&toTx, cb_RD), PACKET_DATA);
      
      ModemControl_Send(cb_getBuf(&toTx, cb_RD));
      cb_pass(&toTx, cb_RD);

      if (blockPack_cnt) blockPack_cnt--;
    } while((!cb_isEmpty(&toTx)) && (blockPack_cnt));

    delay_synchro = SYNCHRO_TIME;    //else delay_toWaitAnswer = 10;	//delay_toWaitAnswer = 10;  //50;
  }

  if (!mode_synchro) return 0;

  if (delay_toWaitAnswer > 0){return 0;}      //The timer between the packets being sent. Otherwise, heap data was not accepted.

  if (((NeedAnswer != PACKET_NONE) || (delay_synchro>0))){      //Maintain communication by sync pulses.

    //Delay_ms(delay_measure_tx_middle);                                  //So as not to foul the broadcast.
    uint8_t wbuff[PACKET_LEN];
    if (NeedAnswer != PACKET_NONE) rfPack_setType(wbuff, NeedAnswer);             // Pulse is Answer
    else if (delay_synchro>0) rfPack_setType(wbuff, PACKET_ASK);                    // Pulse is Ask
    else rfPack_setType(wbuff, PACKET_NONE);                                    // Pulse is None

    ModemControl_Send(wbuff);
  }
  return 0;
}
