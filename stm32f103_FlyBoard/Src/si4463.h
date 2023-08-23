/*
 * Si4463.h
 *
 *  Created on: 16.04.2016.
 *      Author: fademike
 */

#ifndef SRC_SI4463_H_
#define SRC_SI4463_H_

///////////////////////////////////////////////
/////////PART TO CHANGE////////////////////////
///////////////////////////////////////////////

#include "si446x_defs.h"

//#define STM32 // STM32
// #define STM32


// #ifdef STM8
// #include "stm8l.h"
// #define int8_t signed char
// #define uint8_t unsigned char
// #define uint16_t unsigned short
// #define uint32_t unsigned int
// #define int32_t signed int
// #else
#include <stdint.h>

//#endif
int32_t RF_read(uint8_t *data, uint8_t length);
int32_t RF_write(uint8_t *data, uint8_t length);

int32_t RF_init(void);

int16_t getLevel(void);
int16_t getStatus(void);

int8_t setFrequency(int32_t f);
int8_t setPower(unsigned char PA);
int8_t setCarrier(int set);

int8_t SI446X_WAIT_CTS(void);
int8_t SI446X_CMD(uint8_t *cmd, uint8_t cmdsize);
int8_t SI446X_READ_RESPONSE(uint8_t *buffer, uint8_t size);
int8_t SI446X_TX_FIFO_RESET(void);
int8_t SI446X_RX_FIFO_RESET(void);

int8_t SI446X_WRITE_TX_FIFO(uint8_t *txbuffer, uint8_t size);
int8_t SI446X_SET_PROPERTY_1(SI446X_PROPERTY GROUP_NUM, uint8_t proirity);
int8_t SI446X_GPIO_CONFIG(uint8_t G0, uint8_t G1, uint8_t G2, uint8_t G3,
                        uint8_t IRQ, uint8_t SDO, uint8_t GEN_CONFIG);
int8_t SI446X_START_TX(uint8_t channel, uint16_t rx_len);
int16_t SI446X_READ_PACKET(uint8_t *buffer, uint8_t size);
int8_t SI446X_START_RX(uint8_t channel, uint8_t condition, uint16_t rx_len,
                     uint8_t n_state1, uint8_t n_state2, uint8_t n_state3);
// void SI446X_READ_FIFO( unsigned char * buffer );
int8_t SI446X_FIFOINFO(uint8_t *rxCount, uint8_t *txSpace, uint8_t resetTx, uint8_t resetRx);
int8_t SI446X_INT_STATUS(uint8_t *buffer);
int8_t SI446X_PART_INFO(uint8_t *buffer);
int8_t SI446X_POWER_UP(void);

int8_t cmdReadFRR(uint8_t number, uint8_t *data);

#endif /* SRC_SI4463_H_ */
