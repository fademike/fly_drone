/*
 * nRF24L01P.c
 *
 *  Created on: 31 ���. 2020 �.
 *      Author: NASA
 */


#include "nRF24.h"
#include "main.h"
#include "system.h"

extern SPI_HandleTypeDef hspi1;

//
//
//#define	RX_DR			0x40
//#define	TX_DS			0x20
//#define	MAX_RT			0x10

//#define TX_PLOAD_WIDTH 32

unsigned char TX_ADDRESS[TX_ADR_WIDTH] = {0xb2,0xb2,0xb3,0xb4,0x01};
unsigned char RX_BUF[TX_PLOAD_WIDTH];
unsigned char TX_BUF[TX_PLOAD_WIDTH];





unsigned char SPI_Receive_byte(unsigned char reg)
{
	unsigned char rxdata=0;
	HAL_SPI_TransmitReceive(&hspi1, &reg, &rxdata, 1, 100);
   return rxdata;
}

unsigned char SPI_Send_byte(unsigned char reg)
{
	HAL_SPI_Transmit(&hspi1, &reg, 1, 100);
   return reg;
}

unsigned char  SPI_Write_Buf(unsigned char  reg, unsigned char  *pBuf, unsigned char  bytes)
{
	unsigned char  status,byte_ctr;
	CSN(0);
	status= SPI_Receive_byte(reg);
	//HAL_Delay(1); //delayMicroseconds(10);//delay1us(1);
	//uDelay(10);
	for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
		SPI_Send_byte(*pBuf++);
	CSN(1);
	return(status);
}

unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
	unsigned char status;//,byte_ctr;
	CSN(0);
	status=SPI_Receive_byte(reg);
	unsigned char * bufer[32];
	HAL_SPI_TransmitReceive(&hspi1, (unsigned char *)&bufer[0], pBuf, bytes, 100);
	CSN(1);
	return(status);
}

unsigned char SPI_RW_Reg(unsigned char reg, unsigned char value)
{
	unsigned char status;
	CSN(0);
	status=SPI_Receive_byte(reg);   //select register  and write value to it
	SPI_Send_byte(value);
	CSN(1);
	return(status);
}

unsigned char SPI_Read_Reg(unsigned char reg)
{
	unsigned char status, data;
	CSN(0);

	status=SPI_Receive_byte(reg); //SPI_Send_byte(reg);
	data=SPI_Receive_byte(0);   //select register  and write value to it

	//Printf("reg: 0x%x, status: 0x%x, data: 0x%x\n\r", reg, status, data);

	CSN(1);
	return(data);
}

void CRC_PacketCalculate(unsigned char * buff){
	unsigned char mCRC = 87;
	int x=0;
	for (x=0;x<30;x++){mCRC += buff[x]*3;}
	buff[30] = mCRC;
}

int CRC_PacketCheck(unsigned char * buff){
	unsigned char mCRC = 87;
	int x=0;
	for (x=0;x<30;x++){mCRC += buff[x]*3;}
	//Printf("CRC 0x%x, 0x%x ",buff[30], mCRC);
	return  (buff[30] == mCRC);
}


void TX_Mode(unsigned char * tx_buf)
{//HAL_GPIO_WritePin(PIN_TEST_GPIO_Port, PIN_TEST_Pin, GPIO_PIN_SET);
	CE(0);
	CRC_PacketCalculate(tx_buf);

  	SPI_Write_Buf(WRITE_REG_NRF24L01 + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);     // {0xb2,0xb2,0xb3,0xb4,0x01}
  	SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // address 5 byte is {0xb2,0xb2,0xb3,0xb4,0x01} to pipe 0
  	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 // wr payload
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x00);//0x00);//0x3f);       // Enable AA to 1-5 pipe
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);   // Enable rx data to 1-5 pipe
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + SETUP_RETR, 0x0a);  // number retr to error
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 125);         // work channel
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);    // no carrier;  no encoder; 1Mbps; 0dbm
	//  SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x87); SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0e); CE(1); while(1){};
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // num width payload
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + NRF24_CONFIG, 0x0e);      // Enable CRC; 2byte CRC; Power UP; PTX;

	CE(1);
//uDelay(130);
	while((SPI_Read_Reg(FIFO_STATUS)&0x10) == 0x0){}
	//HAL_GPIO_WritePin(PIN_TEST_GPIO_Port, PIN_TEST_Pin, GPIO_PIN_RESET);
}

void RX_Mode(void)
{
	CE(0);
  	SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // address 5 byte is {0xb2,0xb2,0xb3,0xb4,0x01} to pipe 0
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ��

  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x00);//0x00);//0x3f);               // enable ask
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);           // set address rx
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 125);                 // channel

  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);            // 0dbm 1MBps
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + NRF24_CONFIG, 0x0f);               // Enable CRC; 2byte CRC; Power UP; PRX;
  	CE(1);
  	//system_Delay_us(150);//uDelay(150);
}








int nRF24_init(void){
	CE(0);

	unsigned char config =  0x0F, st1, st2;		//0x0f;

	st1 = SPI_RW_Reg(WRITE_REG_NRF24L01 + NRF24_CONFIG, config);               // Enable CRC; 2byte CRC; Power UP; PRX;

	st2 = SPI_Read_Reg(NRF24_CONFIG);
	//Printf("st1=%d, st2=%d\n\r", st1, st2);
  	if (st2 != config) {CE(1); return -1;}

  	RX_Mode();
  	CE(1);
  	return 0;
}

