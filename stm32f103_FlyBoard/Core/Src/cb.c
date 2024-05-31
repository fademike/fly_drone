
// #include <string.h>
#include "cb.h"

int32_t cb_init(circular_buffer * cb, uint16_t bsize, uint16_t psize){
    while (bsize>0){

        cb->ptr[cb_RD] = 0;
        cb->ptr[cb_WR] = 0;
        if (cb->plen) free(cb->plen);
        if (cb->buf) free(cb->buf);

        cb->plen = (uint16_t *)malloc(bsize * sizeof(uint16_t));
        cb->buf = malloc(bsize * psize);
        if(cb->buf != 0) {
            cb->b_size = bsize;
            cb->p_size = psize;
            return bsize;
        }

        bsize--;
    }
    return -1;
}

// get pointer to next package
int32_t cb_getPtrPP(circular_buffer * cb, uint16_t p){
    return (((p+1)>=cb->b_size) ? 0 : (p+1));
}
int32_t cb_isEmpty(circular_buffer * cb){
    return (cb->ptr[cb_RD] == cb->ptr[cb_WR]);
}
int32_t cb_isFull(circular_buffer * cb){
    return (cb_getPtrPP(cb, cb->ptr[cb_WR]) == cb->ptr[cb_RD]);
}
int32_t cb_getSize(circular_buffer * cb){
    return cb->b_size;
}
uint32_t cb_getCnt(circular_buffer * cb){
    if (cb->ptr[cb_RD] > cb->ptr[cb_WR])return ((cb->ptr[cb_WR] + cb->b_size) - cb->ptr[cb_RD]);
    else return (cb->ptr[cb_WR] - cb->ptr[cb_RD]);
}
int32_t cb_pass(circular_buffer * cb, uint8_t RW){
    if (RW == cb_RD) {
        cb->ptr[cb_RD] = cb_getPtrPP(cb, cb->ptr[cb_RD]);
    }
    else if (RW == cb_WR) {
        if (cb_isFull(cb)) return -1;
        cb->ptr[cb_WR] = cb_getPtrPP(cb, cb->ptr[cb_WR]);
    }
    return 0;
}
uint8_t * cb_getBuf(circular_buffer * cb, uint8_t RW){
    if (RW == cb_RD) {
        return &cb->buf[cb->ptr[cb_RD]*cb->p_size];
    }
    else {
        return &cb->buf[cb->ptr[cb_WR]*cb->p_size];
    }
}
int32_t cb_getBufLen(circular_buffer * cb){
    return cb->plen[cb->ptr[cb_RD]];
}

static inline void my_memcpy(uint8_t * a, uint8_t * b, uint16_t len){
    int i=0;
    for (i=0;i<len;i++) a[i] = b[i];
}

int32_t cb_write(circular_buffer * cb, uint8_t *d, uint16_t len){
    uint8_t *ptr;
    ptr = cb_getBuf(cb, cb_WR);
    my_memcpy(ptr, d, len);
    cb->plen[cb->ptr[cb_WR]] = len;
    // printf("write %d: %d, %d, %d\n\r", cb->ptr[cb_WR],ptr[0],ptr[1],ptr[2]);
    if(cb_pass(cb, cb_WR)) return 1;
    return 0;
}

int32_t cb_read(circular_buffer * cb, uint8_t *d, uint16_t len){
    uint8_t *ptr;
    if (cb_isEmpty(cb)) return -1;
    ptr = cb_getBuf(cb, cb_RD);
    my_memcpy(d,ptr,len);
    // printf("read %d: %d, %d, %d\n\r", cb->ptr[cb_RD],ptr[0],ptr[1],ptr[2]);
    cb_pass(cb, cb_RD);
    return 1;
}
int32_t cb_read_pack(circular_buffer * cb, uint8_t *d){
    uint8_t *ptr;
    if (cb_isEmpty(cb)) return -1;
    ptr = cb_getBuf(cb, cb_RD);
    int len = cb->plen[cb->ptr[cb_RD]];
    my_memcpy(d,ptr,len);
    // printf("read %d: %d, %d, %d\n\r", cb->ptr[cb_RD],ptr[0],ptr[1],ptr[2]);
    cb_pass(cb, cb_RD);
    return len;
}



