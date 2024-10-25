/*
 * spi.c
 *
 *  Created on: 16.04.2016.
 *      Author: fademike
 * 
 */

#include "spi.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

int spi_cs_init(void){return 0;}

void spi_cs(int pos){
	if (pos != 0)HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);	//Shut down on (si4463)
	else HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);	//Shut down on (si4463)
}

int spi_ce_init(void){return 0;}

void spi_ce(int pos){
	if (pos != 0)HAL_GPIO_WritePin(SI_SDN_GPIO_Port, SI_SDN_Pin, GPIO_PIN_SET);	//Shut down on (si4463)
	else HAL_GPIO_WritePin(SI_SDN_GPIO_Port, SI_SDN_Pin, GPIO_PIN_RESET);	//Shut down on (si4463)
}

int spi_txrx(unsigned char * buf_tx, unsigned char* buf_rx, int num){
	HAL_SPI_TransmitReceive(&hspi1, buf_tx, buf_rx, num, 100);
    return num;
}

void spi_init(){}

