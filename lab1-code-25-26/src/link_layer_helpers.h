#ifndef LINK_LAYER_HELPERS_H
#define LINK_LAYER_HELPERS_H

#include <stddef.h>

#define FLAG 0x7E
#define A_TX 0x03
#define A_RX 0x01

#define C_SET  0x03
#define C_UA   0x07
#define C_RR0  0xAA
#define C_RR1  0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55
#define C_DISC 0x0B
#define C_I0   0x00
#define C_I1   0x80

#define ESC      0x7D
#define ESC_FLAG 0x5E
#define ESC_ESC  0x5D

unsigned char ll_compute_bcc1(unsigned char a, unsigned char c);
unsigned char ll_compute_bcc2(const unsigned char *data, int length);
void ll_build_supervision_frame(unsigned char *out5, unsigned char A, unsigned char C);
int ll_wait_for_byte(int fd, int timeout_ds, unsigned char *byte);
int ll_read_supervision(int fd, unsigned char *A, unsigned char *C, int timeout_ds);
int ll_write_all(const unsigned char *buf, int len);
int ll_stuff_bytes(const unsigned char *src, int n, unsigned char *dst);
unsigned char ll_rr_for_expected(int expectedSeq);
unsigned char ll_rr_for_next(int seqJustReceived);
unsigned char ll_rej_for_expected(int expectedSeq);

#endif // LINK_LAYER_HELPERS_H
