
#ifndef __CB_H__
#define __CB_H__

#include <stdint.h>

#define CB_USE_LEN

enum {
    cb_RD = 0,
    cb_WR = 1,
};

typedef struct {
    uint8_t * buf;      // pointer to the buffer itself
#ifdef CB_USE_LEN
    uint16_t * plen;     // len current package uint16_t len[b_size]
#endif
    uint16_t b_size;    // max number packages in buffer (buffer size)
    uint16_t p_size;    // max number bytes in package (package size)
    int32_t ptr[2];     // current buf to read/write
} circular_buffer;//toTx;// = {.buf={{1,2,3},{1,2,3},{1,2,3}}, .ptrRx=0, .ptrTx=0};

// #ifdef CB_USE_LEN
// int32_t cb_init(circular_buffer * cb, uint8_t *buf, uint8_t * plen, uint16_t bsize, uint16_t psize);
// #else 
// int32_t cb_init(circular_buffer * cb, uint8_t *buf, uint16_t bsize, uint16_t psize);
// #endif


int32_t cb_init(circular_buffer * cb, uint16_t bsize, uint16_t psize);


int32_t cb_isEmpty(circular_buffer * cb);
int32_t cb_isFull(circular_buffer * cb);
int32_t cb_getSize(circular_buffer * cb);
uint32_t cb_getCnt(circular_buffer * cb);
int32_t cb_pass(circular_buffer * cb, uint8_t RW);
uint8_t * cb_getBuf(circular_buffer * cb, uint8_t RW);
int32_t cb_getBufLen(circular_buffer * cb);
int32_t cb_write(circular_buffer * cb, uint8_t *d, uint16_t len);
int32_t cb_read(circular_buffer * cb, uint8_t *d, uint16_t len);
int32_t cb_read_pack(circular_buffer * cb, uint8_t *d);

#endif