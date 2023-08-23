
#ifndef __CB_H__
#define __CB_H__

#include <stdint.h>

#define CB_SIZE CMD_LIST_SIZE

enum {
    cb_RD = 0,
    cb_WR = 1,
};

typedef struct {
    uint8_t * buf;
    uint16_t b_size;    // number packages in buffer
    uint16_t p_size;    // number bytes in package
    int32_t ptr[2];
} circular_buffer;//toTx;// = {.buf={{1,2,3},{1,2,3},{1,2,3}}, .ptrRx=0, .ptrTx=0};

int32_t cb_init(circular_buffer * cb, uint8_t *buf, uint16_t bsize, uint16_t psize);

int32_t cb_isEmpty(circular_buffer * cb);
int32_t cb_isFull(circular_buffer * cb);
uint32_t cb_getCnt(circular_buffer * cb);
int32_t cb_pass(circular_buffer * cb, uint8_t RW);
uint8_t * cb_getBuf(circular_buffer * cb, uint8_t RW);
int32_t cb_write(circular_buffer * cb, uint8_t *d, uint16_t len);
int32_t cb_read(circular_buffer * cb, uint8_t *d, uint16_t len);


#endif