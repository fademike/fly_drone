/*
 * Si4463.c
 *
 *  Created on: 16.04.2016.
 *      Author: fademike
 */


#include "ModemControl.h"
#include "main.h"

#include "si446x_defs.h"
#include "si4463.h"
#include "radio_config_Si4463.h"

#include "spi.h"

#define SI4463_CUT_UNUSED 1

int16_t changeState(uint8_t state);
// int32_t RF_read(uint8_t *data, uint8_t length);
// int32_t RF_write(uint8_t *data, uint8_t length);

// int32_t RF_init(void);

struct modem_struct rf_si4463 = {.init = RF_init, .read = RF_read, .write = RF_write,};

uint8_t SPI_TxRxData(uint8_t request){
    uint8_t answer=0;
    spi_txrx(&request, &answer, 1);
    return answer;
}

void SPI_Set_NSS(int set)
{
    if (set != 0) spi_cs(1);
    else spi_cs(0);
}

#define READ_CMD_BUFF 0x44

int8_t SI446X_WAIT_CTS( void )
{
	uint8_t cTemp = 0x44;
	uint8_t cts;
    uint16_t timeout = 1000;
    do
    {
    	SPI_Set_NSS(0);

        SPI_TxRxData(cTemp);
        cts = SPI_TxRxData(0x00);
        
        SPI_Set_NSS(1);
        // if (timeout > 0) timeout--;
        // else return -1;
    } while( cts != 0xFF );
    return 0;
}

int8_t SI446X_CMD(uint8_t *cmd, uint8_t cmdsize )
{
    if (SI446X_WAIT_CTS()) return -1;
    SPI_Set_NSS(0);
    spi_txrx(cmd, cmd, cmdsize);
    SPI_Set_NSS(1);
    return 0;
}

int8_t SI446X_READ_RESPONSE( uint8_t *buffer, uint8_t size )
{
	uint8_t cTemp = 0x44;
    uint8_t i=0;
	//uint8_t cts = 0x00;
    for (i=0;i<size;i++)buffer[i] = 0x00;
    i=0;
    
    if (SI446X_WAIT_CTS()) return -1;
    SPI_Set_NSS(0);
    spi_txrx(&cTemp, &cTemp, 1);
    spi_txrx(buffer, buffer, size);
    SPI_Set_NSS(1);
    return 0;
}


int8_t SI446X_TX_FIFO_RESET( void )
{
	uint8_t cmd[2+1];
    cmd[0] = FIFO_INFO;		//FIFO_INFO = 0x15
    cmd[1] = 0x01;
    if (SI446X_CMD( cmd, 2 )) return -1;
    if (SI446X_READ_RESPONSE(cmd, 2)) return -1;
    return 0;
}


int8_t SI446X_RX_FIFO_RESET( void )
{
	uint8_t cmd[2+1];
    cmd[0] = FIFO_INFO;
    cmd[1] = 0x02;
    if (SI446X_CMD( cmd, 2 )) return -1;
    if (SI446X_READ_RESPONSE(cmd, 2)) return -1;
    return 0;
}

int8_t SI446X_WRITE_TX_FIFO( uint8_t *txbuffer, uint8_t size)
{
	uint8_t cmd[1] = {WRITE_TX_FIFO};
        uint8_t i=0;
	cmd[0] = WRITE_TX_FIFO;

	if (size > 64) size = 64;

    if (SI446X_WAIT_CTS()) return -1;
          
    SPI_Set_NSS(0);
    SPI_TxRxData(cmd[0]);
    
    while(i<size){SPI_TxRxData(txbuffer[i++]);}
    SPI_Set_NSS(1);
    return 0;
}

#if !SI4463_CUT_UNUSED

int8_t SI446X_SET_PROPERTY_1( SI446X_PROPERTY GROUP_NUM, uint8_t proirity )
{
    uint8_t cmd[5];
    cmd[0] = SET_PROPERTY;
    cmd[1] = GROUP_NUM>>8;
    cmd[2] = 1;
    cmd[3] = GROUP_NUM;
    cmd[4] = proirity;
    if (SI446X_CMD( cmd, 5 )) return -1;
    return 0;
}


int8_t SI446X_GPIO_CONFIG( uint8_t G0, uint8_t G1, uint8_t G2, uint8_t G3,
		uint8_t IRQ, uint8_t SDO, uint8_t GEN_CONFIG )
{
	uint8_t cmd[10];
    cmd[0] = GPIO_PIN_CFG;
    cmd[1] = G0;
    cmd[2] = G1;
    cmd[3] = G2;
    cmd[4] = G3;
    cmd[5] = IRQ;
    cmd[6] = SDO;
    cmd[7] = GEN_CONFIG;
    if (SI446X_CMD( cmd, 8 )) return -1;
    if (SI446X_READ_RESPONSE( cmd, 8 )) return -1;
    return 0;
}

#endif

int16_t SI446X_READ_PACKET( uint8_t *buffer , uint8_t size)
{
	uint8_t i, cTemp = READ_RX_FIFO;
	if (SI446X_WAIT_CTS( )) return -1;
	SPI_Set_NSS(0);

    SPI_TxRxData(cTemp);

    i=0;
    while(i<size){buffer[i++] = SPI_TxRxData(0);}


    SPI_Set_NSS(1);
    return buffer[0]; 	//i;
}


int8_t SI446X_START_TX( uint8_t channel, uint16_t rx_len)
{
	uint8_t cmd[6];
    cmd[0] = START_TX;
    cmd[1] = channel;
    cmd[2] = 0x30;
    cmd[3] = rx_len>>8;
    cmd[4] = rx_len;
 //   cmd[5] = 0x00;
    if (SI446X_CMD( cmd, 5 )) return -1;
    if (SI446X_WAIT_CTS()) return -1;
    return 0;
}

int8_t SI446X_START_RX( uint8_t channel, uint8_t condition, uint16_t rx_len,
		uint8_t n_state1, uint8_t n_state2, uint8_t n_state3 )
{
	uint8_t cmd[8];
//    SI446X_RX_FIFO_RESET( );
//    SI446X_TX_FIFO_RESET( );
    cmd[0] = START_RX;
    cmd[1] = channel;
    cmd[2] = condition;
    cmd[3] = rx_len>>8;
    cmd[4] = rx_len;
    cmd[5] = n_state1;
    cmd[6] = n_state2;
    cmd[7] = n_state3;
    if (SI446X_CMD( cmd, 8 )) return -1;
    if (SI446X_WAIT_CTS()) return -1;
    return 0;
}

// void SI446X_READ_FIFO( uint8_t * buffer )
// {
// 	uint8_t cmd[2];
//     cmd[0] = FIFO_INFO;
//     cmd[1] = 0x00;
//     SI446X_CMD( cmd, 2 );
//     //HAL_SPI_Receive(&hspi2, buffer, 3, 100);
//     SI446X_READ_RESPONSE(buffer, 3);
// }

int8_t SI446X_FIFOINFO(uint8_t  *rxCount, uint8_t  *txSpace, uint8_t  resetTx, uint8_t  resetRx)
{
	uint8_t cmd[2], response[3];

	cmd[0] = FIFO_INFO;
	cmd[1] = (resetRx ? 1 << 1 : 0) | (resetTx ? 1 << 0 : 0);

	if (SI446X_CMD( cmd, 2 )) return -1;	//_write(command, sizeof(command));

	if (SI446X_READ_RESPONSE(response, 3)) return -1;//_pollDataReply(response, sizeof(response));
	if (rxCount)
		*rxCount = response[1];
	if (txSpace)
		*txSpace = response[2];
    return 0;
}

int8_t SI446X_INT_STATUS( uint8_t *buffer )
{
	uint8_t cmd[4];
    cmd[0] = GET_INT_STATUS;
    cmd[1] = 0;
    cmd[2] = 0;
    cmd[3] = 0;
    if (SI446X_CMD( cmd, 4 )) return -1;
    if (SI446X_READ_RESPONSE( buffer, 9 )) return -1;
    return 0;
}

int8_t SI446X_PART_INFO( uint8_t *buffer )
{
	uint8_t cmd[16];
	cmd[0] = 0x01;										//PART_INFO
	if (SI446X_CMD(cmd, 1)) return -1;
	if (SI446X_READ_RESPONSE( &buffer[0], 9 )) return -1;
    return 0;

}
int8_t SI446X_POWER_UP(void)
{
	uint8_t cmd[16];

	  cmd[0] = POWER_UP;   //CMD_POWER_UP
	  cmd[1] = 0x01;		// FUNC=1
	  cmd[2] = 0x00;
	  cmd[3] = 0x01;		//30MHz
	  cmd[4] = 0xC9;
	  cmd[5] = 0xC3;
	  cmd[6] = 0x80;

	  if (SI446X_CMD(cmd, 7)) return -1;
	  if (SI446X_WAIT_CTS()) return -1;
    return 0;
}

int8_t cmdReadFRR(uint8_t number, uint8_t *data)
{
	uint8_t request = 0xFF;

	switch (number) {
		case 0:
			request = (uint8_t)FRR_A_READ;
			break;
		case 1:
			request = (uint8_t)FRR_B_READ;
			break;
		case 2:
			request = (uint8_t)FRR_C_READ;
			break;
		case 3:
			request = (uint8_t)FRR_D_READ;
			break;
	}

	SPI_Set_NSS(0);
    SPI_TxRxData(request);
    data[0] = SPI_TxRxData(0x00);

    SPI_Set_NSS(1);

	return 0;
}

int16_t getLevel(void)
{
	uint8_t level = 0;
	if (cmdReadFRR(0, &level)) return -1;
	//return -((int8_t)(127 - level / 2));
	return level;
}

#define REGISTRS 1

int16_t getStatus(void)
{
#if REGISTRS
	uint8_t status;
	if (cmdReadFRR(1, (uint8_t *)&status)) return -1;
	return status;
#else
	uint8_t buffer[9];
	SI446X_INT_STATUS(buffer);
	return buffer[3];
#endif
}

int16_t changeState(uint8_t state)
{
	switch (state) {
		case STATE_RX:
			if (SI446X_START_RX(0,0,0,0,3,3)) return -1;//cmdStartRx(0, 0);
			break;
		case 2:
			if (SI446X_START_TX(0,0)) return -1;
			break;
		default:
			if (SI446X_START_RX(0,0,0,0,3,3)) return -1;
			break;
	}

	return 0;
}


 int32_t RF_read(uint8_t *data, uint8_t length)
//int RFread(char * buf)
{

    uint8_t buffer[9];
    if (SI446X_INT_STATUS(buffer)) return -1;
    uint8_t xStatus = buffer[3];

// #if 1		//Add Carrier receive handle to future
//     if ((xStatus&(1<<4))==0){	// GET carrier!?!? defect! need to clear bit.
//     	if ((buffer[6]&0x3) != 0) { return 1;}//thread_ModemControl.t_counter = 0; return;}
//     }
// #endif
    if ((xStatus&(1<<4))!=0){	 // if there is new package

        if (SI446X_READ_PACKET(data , length) < 0) return -1;
        if (SI446X_FIFOINFO(0, 0, 1, 1)) return -1;		// BUFFER CLEAR

        if (changeState(STATE_RX)) return -1;
        return length;
    }
	return 0;
}



 int32_t RF_write(uint8_t *data, uint8_t length)
{
    int ret = 0;
        if (SI446X_WRITE_TX_FIFO(data, length)) return -1;
        if (changeState(STATE_TX)) return -1;
        do {
            ret = getStatus();
            if (ret < 0) return -1;
        }
        while((getStatus()&0x20) == 0x00);		// run thread while wait
        if (changeState(STATE_RX)) return -1;
        return 0;
}


const uint8_t config_table[] = RADIO_CONFIGURATION_DATA_ARRAY;



 int32_t RF_init(void)
{
    spi_ce(1);
    msleep(200);
    spi_ce(0);
    msleep(200);

    unsigned char buffer[16];

    if (SI446X_PART_INFO(buffer)) return -2;

    if ((buffer[3]!= 0x63)) return -1;

    unsigned char len;
    unsigned int j = 0;
    while( ( len = config_table[j++] ) != 0 )
    {
        uint8_t b[128], t=0;
        for (t=0; t<len; t++) b[t] = config_table[j+t];
        if (SI446X_CMD(b, len)) return -1;
        j += len;
    }

    //setFrequency(433.3*1000*1000);
    if (setPower(0x20)) return -1;//(0x08);	//(0x7F);
    if (SI446X_FIFOINFO(0, 0, 1, 1)) return -1;		// BUFFER CLEAR
    if (changeState(STATE_RX)) return -1;			// Set state to RX

// #if 1	// carrier mode
//         setCarrier(1);
// #endif

    return 0;
}

#if !SI4463_CUT_UNUSED

int8_t setFrequency(int32_t f)
{
	uint8_t data_cmd[] = {RF_FREQ_CONTROL_INTE_8};
	  //unsigned long Constant = 1<<19;
	uint8_t Array_RF_MODEM_CLKGEN_BAND_1[] = {RF_MODEM_CLKGEN_BAND_1};
	uint32_t OUTDIV = Array_RF_MODEM_CLKGEN_BAND_1[4]&0x7;
	  if (OUTDIV == 0) OUTDIV=4;
	  else if (OUTDIV == 1) OUTDIV=6;
	  else if (OUTDIV == 2) OUTDIV=8;
	  else if (OUTDIV == 3) OUTDIV=12;
	  else if (OUTDIV == 4) OUTDIV=16;
	  else OUTDIV=24;

	  uint8_t NPRESC = 0;
	  if(Array_RF_MODEM_CLKGEN_BAND_1[4]&0x8) NPRESC = 2;
	  else NPRESC = 4;

	  uint32_t freq_xo = RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ;

	  uint32_t d = 524288;//1<<19;
	  uint32_t C = NPRESC*freq_xo / OUTDIV;

	  uint32_t result = f%C;
	  uint32_t inte = (f/C)-1;

	  uint32_t frac_float = ((float)d/(float)C)* ((float)C + (float)result);

	  uint8_t frac_1 = (frac_float>>16)&0xFF;
	  uint8_t frac_2 = (frac_float>>8)&0xFF;
	  uint8_t frac_3 = (frac_float>>0)&0xFF;

	  data_cmd[4] = inte&0xFF;

	  data_cmd[5] = frac_1;
	  data_cmd[6] = frac_2;
	  data_cmd[7] = frac_3;

	  if (SI446X_CMD(&data_cmd[0], 0x0C )) return -1;

    return 0;
}




int8_t setCarrier(int set)
{
	uint8_t data_cmd[] = {RF_MODEM_MOD_TYPE_12};
	if (set != 0) data_cmd[4] = 2|0x8;
	if (SI446X_CMD(&data_cmd[0], 0x10 )) return -1;
    return 0;
}

#endif

int8_t setPower(unsigned char PA)
{
	uint8_t data_cmd[] = {RF_PA_MODE_4};
	  data_cmd[5] = PA;

	  if (SI446X_CMD(&data_cmd[0], 0x08 )) return -1;
    return 0;

}

