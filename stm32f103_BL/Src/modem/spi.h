#pragma once
#ifndef __SPI_H__
#define __SPI_H__


int spi_cs_init(void);
void spi_cs(int pos);
int spi_ce_init(void);
void spi_ce(int pos);

void spi_init();
int spi_txrx (unsigned char *buf_tx, unsigned char  *buf_rx, int num);

#endif // __SPI_H__
