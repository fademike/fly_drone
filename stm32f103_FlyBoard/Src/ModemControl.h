

#include "stm32f1xx_hal.h"

//#define nRF24//nRF24	//SI4463

typedef enum  {
	STATE_IDLE 				= 0,
	STATE_RX				= 1,
	STATE_TX				= 2,
} STATE;


typedef enum  {
	PACKET_NONE 		= 0,
	PACKET_DATA			= 1,
	PACKET_ASK			= 2,
	PACKET_ANSWER		= 3,
	PACKET_DATA_PACK	= 4,
} PACKET_TYPE;


// Need to add to msTimer: if(delay_to_wait_answer>0) delay_to_wait_answer--; and other! //FIXME
// and:
//	extern volatile int delay_to_wait_answer;
//	extern volatile int delay_measure_tx;
//	extern volatile int delay_synchro;
//	if(delay_to_wait_answer>0) delay_to_wait_answer--;
//	if(delay_synchro>0) delay_synchro--;	// timer for synchronization modem
//	delay_measure_tx++;



//#define READCMDSTREAM_ARRAY 10

#define SYNCHRO_MODE 1	// 1- synchronization mode; 0- simple mode. More in stm8+si4463 project
////////////////////////////For SYNCHRO_MODE :
#define SYNCHRO_TIME 500                                                        // sync time
#define SYNCHRO_TWAIT 20//6*4                                                       // TIMEOUT of ANSWER = (TIME of TX 1 packet) + SYNCHRO_TWAIT    // 10bps - 100 TWAIT(57us); 100kbps - 6TWAIT(6us)

//DELETE									//circular buffer for send out by RF
#define UART_BUF_SIZE 3	//255 TODO FIXME			// buffer lenght


#define MODEM_PINCONTROL 0	// For debug pin output

int ModemControl_getStatus(void);

int ModemControl_init(void);

//void ModemControl_ReadSymbolRfomRF_Callback(unsigned char data);
//void ModemControl_ReadMSGRfomRF_Callback(unsigned char * data);

void ModemControl_SendPacket(char * buff);
void ModemControl_SendSymbol(char buff);

int ModemControl_ReadOnly(void);
void ModemControl_Work(void);



