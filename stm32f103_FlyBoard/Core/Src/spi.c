/*
 * spi.c
 *
 * Copyright 2016 Edward V. Emelianov <eddy@sao.ru, edward.emelianoff@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
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

