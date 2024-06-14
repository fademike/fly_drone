/*
 * ModemControl.h
 *
 *  Created on: 06.04.2020.
 *      Author: fademike
 */

#ifndef __MODEMCONTROL_H__
#define __MODEMCONTROL_H__

#include <stdint.h>

typedef enum  {
	STATE_IDLE 				= 0,
	STATE_RX				= 1,
	STATE_TX				= 2
} STATE_TRANSIEVER;

typedef enum  {
	PACKET_NONE 		= 0,
	PACKET_DATA			= 1,
	PACKET_ASK			= 2,
	PACKET_ANSWER		= 3,
	PACKET_DATA_PART	= 4,
} PACKET_TYPE;

struct modem_struct {
	int32_t (*init)();
	int32_t (*read)(uint8_t *data, uint8_t length);
	int32_t (*write)(uint8_t *data, uint8_t length);
};



#include "main.h"

#ifdef SI4463

#include "si4463.h"
#include "radio_config_Si4463.h"
#define PACKET_LEN 64
#define PACKET_LEN_CRC 2	      // package Length
#define PACKET_DATALEN 60
#define CMD_LIST_SIZE 5       // any num RAM

#elif defined (nRF24)

#include "nRF24.h"
#define PACKET_LEN 32	      // package Length
#define PACKET_LEN_CRC 2	      // package Length
#define PACKET_DATALEN 28   // the size of the data in the package
#define CMD_LIST_SIZE 10    // number of packages

#endif

#define SYNCHRO_MODE 1	// 1- synchronization mode; 0- simple mode. More in stm8+si4463 project
#define SEND_PACKETBLOCK 0	// if not zero - the maximum number of packets in a block
// #if SYNCHRO_MODE

#define SYNCHRO_TIME 500    // sync time, ms
#define SYNCHRO_TWAIT 20	// wait answer, ms

// #endif

int32_t ModemControl_init(void);

int32_t ModemControl_getStatus(void);

int32_t ModemControl_Loop(void);	// return 1 when rx data
int32_t ModemControl_ReadOnly(void);

int32_t ModemControl_SendPacket_GetQueue(void);
int32_t ModemControl_SendPacket_SizeQueue(void);


void ModemControl_SendPacket(uint8_t * buff, uint16_t len);
void ModemControl_SendSymbol(uint8_t buff);

int32_t ModemControl_GetPacket(uint8_t * buf);
int32_t ModemControl_GetByte(uint8_t * buf);
//



#endif
