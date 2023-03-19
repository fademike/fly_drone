/*
 * ModemControl.c
 *
 *  Created on: 06 ���. 2020 �.
 *      Author: NASA
 */

#include <string.h>	// for memcpy

#include "main.h"
#include "ModemControl.h"
#include "mavlink_handler.h"
#include "system.h"

#ifdef nRF24

#include "nRF24.h"

#define PACKET_LEN 32
#define PACKET_DATALEN 28
#define CMD_LIST_SIZE 10    // max buffer

#endif

#ifdef SI4463
#include "Si4463.h"b
#include "radio_config_Si4463.h"

#define PACKET_LEN 64
#define PACKET_DATALEN 60
#define CMD_LIST_SIZE 10    // any num RAM

#endif

#define DataEx(a) (a##_RD != a##_WR)
#define DataExAny(f, a) (f##_RD != a##_WR)
#define DataArrayPP(a) a = (++a >= CMD_LIST_SIZE) ? 0 : a;//if (++a>=CMD_LIST_SIZE) a=0
#define DataArrayMM(a) a = (--a < 0) ? CMD_LIST_SIZE : a;// if (--a<0) a=(CMD_LIST_SIZE-1)


unsigned char toSendCmdbuf[CMD_LIST_SIZE][PACKET_LEN];
int toSendCmd_RD;
int toSendCmd_WR;

unsigned char toRxCmdbuf[CMD_LIST_SIZE][PACKET_LEN];
int toRxCmd_RD;
int toRxCmd_WR;


//  Variables of modem

volatile int delay_to_wait_answer=0;
volatile int delay_synchro=0;
char NeedAnswer=0;


#define MODEM_PINCONTROL 0	// For debug pin output

static int ModemControl_status = -1;

int ModemControl_getStatus(void){
	return ModemControl_status;
}

int ModemControl_init(void){


#ifdef SI4463

	HAL_GPIO_WritePin(SI_SDN_GPIO_Port, SI_SDN_Pin, GPIO_PIN_SET);	//Shut down on (si4463)
	    HAL_Delay(200);
	    HAL_GPIO_WritePin(SI_SDN_GPIO_Port, SI_SDN_Pin, GPIO_PIN_RESET);	//Shut down off (si4463)
	    HAL_Delay(200);

	    unsigned char buffer[16];
	    do{


	    	SI446X_PART_INFO(buffer);

#ifdef SI_POOR_PRINTF
	    	Printf_str("Si data is : 0x");
	    	Printf_hex(buffer[2]);
	    	Printf_str(", 0x");
	    	Printf_hex(buffer[3]);
	    	Printf_str("\n\r");
	    	if ((buffer[2] == 0x44) && (buffer[3]== 0x63)) Printf_str("Dev is si4463\n\r");
#endif
//#ifdef SI_SIMPLE_PRINTF
	      	Printf("Si data is : 0x%x, 0x%x\n\r", buffer[2], buffer[3]);
	      	if ((buffer[2] == 0x44) && (buffer[3]== 0x63)) Printf("Dev is si4463\n\r");
//#endif

	      	HAL_Delay(200);

	    }while((buffer[3]!= 0x63));

	    RFinit();			//Set Settings Config to Si4463
	    //setFrequency(433.3*1000*1000);
	    setPower(0x20);	//(0x7F);

	    SI446X_FIFOINFO(0, 0, 1, 1);		// BUFFER CLEAR

	    changeState(STATE_RX);			// Set state to RX

	    ModemControl_status =  0;

	    Printf("si4463 init\n\r");

	#if 0	// carrier mode
			  setCarrier(1);
			  char wbuff[64];
			  RFwrite(wbuff, PACKET_LEN);

			  while(1){RFwrite(wbuff, PACKET_LEN);};
	#endif

#endif

#ifdef nRF24
	CSN(GPIO_PIN_RESET);
	CE(GPIO_PIN_SET);

	HAL_Delay(20);

	ModemControl_status = nRF24_init();
	if (ModemControl_status < 0) Printf("nRF_init false\n\r");
	else Printf("nRF_init OK\n\r");

	return ModemControl_status;
#endif
}


void ModemControl_SendPacket(char * buff)
{
	memcpy(&toSendCmdbuf[toSendCmd_WR][2], buff, PACKET_DATALEN);
	DataArrayPP(toSendCmd_WR);
}
void ModemControl_SendSymbol(char data)
{
	static int i=0;
	static char buff[PACKET_DATALEN];
	buff[i++] = data;
	if (i>=PACKET_DATALEN){memcpy(&toSendCmdbuf[toSendCmd_WR][2], buff, PACKET_DATALEN); i=0; DataArrayPP(toSendCmd_WR);}
}

void timer_tick_loop(void){
	static uint32_t l_time = 0;//system_getTime_ms();
	uint32_t c_time = system_getTime_ms();

	uint32_t delay = c_time - l_time;
	if (delay<=0) return;
	int t;
	for (t=0;t<delay;t++){
		if(delay_to_wait_answer>0) delay_to_wait_answer--;
		//else ThreadFly[thread_ModemControl].t_counter = 0;//{thread_ModemControl.t_counter = 0;}		//TODO need run tx answer
		if(delay_synchro>0) delay_synchro--;	// timer for synchronization modem
	}
	l_time = c_time;
}

uint32_t delay_measure_tx = 0; // for calculate tx time

void ModemControl_Send(unsigned char * buf){
	delay_measure_tx = system_getTime_ms();//0;	// start calculate tx time

#ifdef SI4463

           	   RFwrite(buf, PACKET_LEN);                                         // TX Data
#endif
#ifdef nRF24
           		TX_Mode(buf);
#endif


	delay_measure_tx = system_getTime_ms() - delay_measure_tx;	//calculate time for tx
	if (delay_measure_tx > 100) delay_measure_tx=0;
	delay_measure_tx++;
}

int ModemControl_Read(void){

	unsigned char rbuff[PACKET_LEN];

#ifdef nRF24
	unsigned char status=0x01;

	status=SPI_Read_Reg(STATUS);
	if(status & 0x40){//if ((xStatus&(1<<4))!=0){	                                                //PACKET_RX

		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x40);

		unsigned char fifo_status = 0;
		do{

			SPI_Read_Buf(RD_RX_PLOAD,rbuff,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer

#endif
#ifdef SI4463

		uint8_t buffer[6];
		SI446X_INT_STATUS(buffer);
		uint8_t xStatus = buffer[3];

		 if ((xStatus&(1<<4))!=0){	                                              //PACKET_RX
				 RFread(rbuff, PACKET_LEN);
				 SI446X_FIFOINFO(0, 0, 1, 1);		// BUFFER CLEAR

#endif

#if SYNCHRO_MODE
			if (rbuff[0] == PACKET_NONE ){
				NeedAnswer = 0; delay_to_wait_answer = 0;
			}
			else if (rbuff[0] == PACKET_ASK ){
				NeedAnswer = PACKET_ANSWER; delay_to_wait_answer = 0;
			}
			else if (rbuff[0] == PACKET_ANSWER ){
				NeedAnswer = 0; delay_to_wait_answer = 0;
			}
			else
#endif
			if ((rbuff[0] == PACKET_DATA) || (rbuff[0] == PACKET_DATA_PACK)){                                   // if Packet is PACKET_DATA

				  if (rbuff[0] == PACKET_DATA) {
					  NeedAnswer = PACKET_ANSWER;
					  delay_to_wait_answer = 0;
				  }
				  else {NeedAnswer = 0; delay_to_wait_answer = SYNCHRO_TWAIT;}

				char cnt = rbuff[1]&0x3F;
				if (cnt >=1){
#ifdef nRF24
					if 	(CRC_PacketCheck(rbuff)){					// crc check
						memcpy(toRxCmdbuf[toRxCmd_WR], rbuff, 32);
						DataArrayPP(toRxCmd_WR);
					}
#endif
					int i=2;
					do{
						mavlink_receive(rbuff[i++]);
					}while(--cnt>0);
				}
			}
#ifdef nRF24
			fifo_status=SPI_Read_Reg(FIFO_STATUS);
		}while((fifo_status&0x01) != 0x01);

		RX_Mode(); // Switch to RX mode//changeState(STATE_RX);
		return 1;
	}

	if(status&TX_DS) SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x20);
	if(status&MAX_RT) SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x10);

#endif
#ifdef SI4463
		changeState(STATE_RX);
		return 1;
		}

#endif
	return 0;
//#endif
}


void ModemControl_Work(void)
{
	if (ModemControl_status < 0) ModemControl_init();
	timer_tick_loop();

	if (ModemControl_Read() != 0) return;

#if 0                                                                     // UART Echo
     while(UartRxBuffer_rd!=UartRxBuffer_wr){
         UART1_SendData8((uint8_t)UartRxBuffer[UartRxBuffer_rd]);
         while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
         if (++UartRxBuffer_rd>=UART_BUF_SIZE) UartRxBuffer_rd = 0;
     }
#endif
     //SPI_Read_Reg(CD);//return;


	if (delay_to_wait_answer != 0)return;                           //The timer between the packets being sent. Otherwise, heap data was not accepted.

#define SEND_PACKETBLOCK 0

	if (DataEx(toSendCmd)){
#if SEND_PACKETBLOCK
		do{
			int future_RD = toSendCmd_RD;
			int middlePack = 0;
			DataArrayPP(future_RD);
			if (DataExAny(future, toSendCmd)) {middlePack = 1;}
			if (middlePack) {toSendCmdbuf[toSendCmd_RD][0] = PACKET_DATA_PACK;}
			else {
				toSendCmdbuf[toSendCmd_RD][0] = PACKET_DATA;
			}
#else
			toSendCmdbuf[toSendCmd_RD][0] = PACKET_DATA;
#endif
			toSendCmdbuf[toSendCmd_RD][1] = (PACKET_DATALEN)&0x3F;
			ModemControl_Send(toSendCmdbuf[toSendCmd_RD]);

			//if (middlePack) {Printf(".");} else  {Printf("#");}

			DataArrayPP(toSendCmd_RD);
#if SEND_PACKETBLOCK
		}while(DataEx(toSendCmd));
#endif
		if (SYNCHRO_MODE) delay_to_wait_answer = delay_measure_tx+SYNCHRO_TWAIT;  //50; //Set timer between send the packets
		delay_synchro = SYNCHRO_TIME;    //else delay_to_wait_answer = 10;	//delay_to_wait_answer = 10;  //50;

		NeedAnswer=0;	//FIXME
		RX_Mode();
	}

#if SYNCHRO_MODE

	else if (((NeedAnswer>0) || (delay_synchro>0))){      //Maintain communication by sync pulses.

		//Delay_ms(delay_measure_tx_middle);                                  //So as not to foul the broadcast.
		unsigned char wbuff[64];
		if (NeedAnswer) {wbuff[0] = NeedAnswer; NeedAnswer=0;}              // Pulse is Answer
		else if (delay_synchro>0) wbuff[0] = PACKET_ASK;                    // Pulse is Ask
		else wbuff[0] = PACKET_NONE;                                        // Pulse is Non

		ModemControl_Send(wbuff);

		delay_to_wait_answer = delay_measure_tx+SYNCHRO_TWAIT;  //delay_to_wait_answer = delay_measure_tx*4;  //50;

		RX_Mode();

	}
#endif




}
