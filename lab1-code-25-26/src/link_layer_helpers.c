#include "link_layer_helpers.h"
#include "serial_port.h"

#include <errno.h>
#include <sys/select.h>
#include <unistd.h>

unsigned char ll_compute_bcc1(unsigned char a, unsigned char c) {
    return (unsigned char)(a ^ c);
}

unsigned char ll_compute_bcc2(const unsigned char *data, int length) {
    unsigned char result = 0;
    if (!data || length <= 0) return result;
    for (int i = 0; i < length; ++i) result ^= data[i];
    return result;
}

void ll_build_supervision_frame(unsigned char *out5, unsigned char A, unsigned char C) {
    if (!out5) return;
    out5[0] = FLAG;
    out5[1] = A;
    out5[2] = C;
    out5[3] = ll_compute_bcc1(A, C);
    out5[4] = FLAG;
}

int ll_wait_for_byte(int fd, int timeout_ds, unsigned char *byte) {
    if (fd < 0) return -1;

    while (1) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);

        struct timeval tv;
        struct timeval *ptv = NULL;
        if (timeout_ds >= 0) {
            tv.tv_sec = timeout_ds / 10;
            tv.tv_usec = (timeout_ds % 10) * 100000;
            ptv = &tv;
        }

        int ret = select(fd + 1, &readfds, NULL, NULL, ptv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (ret == 0) return 0;

        int r = readByteSerialPort(byte);
        if (r == -1) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            if (errno == EIO || errno == ENXIO) {
                usleep(10000);
                continue;
            }
            return -1;
        }
        if (r == 0) continue;
        return 1;
    }
}

int ll_read_supervision(int fd, unsigned char *A, unsigned char *C, int timeout_ds) {
    enum {
        SUP_WAIT_FLAG = 0,
        SUP_WAIT_A,
        SUP_WAIT_C,
        SUP_WAIT_BCC1,
        SUP_WAIT_FLAG_END
    };

    int state = SUP_WAIT_FLAG;
    unsigned char byte = 0;
    int elapsed = 0;

    while (elapsed < timeout_ds) {
        int r = ll_wait_for_byte(fd, 1, &byte);
        if (r == -1) return -1;
        if (r == 0) {
            elapsed++;
            continue;
        }
        switch (state) {
        case SUP_WAIT_FLAG:
            if (byte == FLAG) state = SUP_WAIT_A;
            break;
        case SUP_WAIT_A:
            if (A) *A = byte;
            state = SUP_WAIT_C;
            break;
        case SUP_WAIT_C:
            if (C) *C = byte;
            state = SUP_WAIT_BCC1;
            break;
        case SUP_WAIT_BCC1:
            if (byte == ll_compute_bcc1(A ? *A : 0, C ? *C : 0)) state = SUP_WAIT_FLAG_END;
            else state = SUP_WAIT_FLAG;
            break;
        case SUP_WAIT_FLAG_END:
            if (byte == FLAG) return 1;
            state = SUP_WAIT_FLAG;
            break;
        }
    }
    return 0;
}

int ll_write_all(const unsigned char *buf, int len) {
    if (!buf || len <= 0) return -1;
    int written = writeBytesSerialPort(buf, len);
    return (written == len) ? 0 : -1;
}

int ll_stuff_bytes(const unsigned char *src, int n, unsigned char *dst) {
    if (!src || !dst || n < 0) return -1;
    int k = 0;
    for (int i = 0; i < n; ++i) {
        unsigned char b = src[i];
        if (b == FLAG) {
            dst[k++] = ESC;
            dst[k++] = ESC_FLAG;
        } else if (b == ESC) {
            dst[k++] = ESC;
            dst[k++] = ESC_ESC;
        } else {
            dst[k++] = b;
        }
    }
    return k;
}

unsigned char ll_rr_for_expected(int expectedSeq) {
    return expectedSeq == 0 ? C_RR0 : C_RR1;
}

unsigned char ll_rr_for_next(int seqJustReceived) {
    return ll_rr_for_expected(seqJustReceived ^ 1);
}

unsigned char ll_rej_for_expected(int expectedSeq) {
    return expectedSeq == 0 ? C_REJ0 : C_REJ1;
}
