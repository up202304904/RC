// Link layer protocol implementation
#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdarg.h>
#include <stdlib.h>
#include <sys/select.h>

static int log_enabled = -1;

/*static void logMessage(const char *fmt, ...) {
    if (log_enabled == -1) {
        const char *env = getenv("LL_DEBUG");
        log_enabled = (env == NULL || (env[0] != '0' && env[0] != '\0')) ? 1 : 0;
    }
    if (!log_enabled) return;

    va_list args;
    va_start(args, fmt);
    fputs("[ll] ", stderr);
    vfprintf(stderr, fmt, args);
    fputc('\n', stderr);
    va_end(args);
}

#define LOG(...) logMessage(__VA_ARGS__)*/

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

#define WAIT_FLAG     0
#define WAIT_A        1
#define WAIT_C        2
#define WAIT_BCC1     3
#define READ_DATA     4
#define WAIT_FLAG_END 5

static LinkLayerRole linkRole;
static int tx_seq = 0;
static int rx_expected_seq = 0;
static int link_timeout_ds = 10;
static int link_max_retries = 3;
static int serial_fd = -1;

static inline unsigned char bcc1(unsigned char a, unsigned char c) {
    return (unsigned char)(a ^ c);
}

static void buildSupervisionFrame(unsigned char *out5, unsigned char A, unsigned char C) {
    out5[0] = FLAG;
    out5[1] = A;
    out5[2] = C;
    out5[3] = bcc1(A, C);
    out5[4] = FLAG;
}

static int waitForByte(unsigned char *byte, int timeout_ds) {
    if (serial_fd < 0) return -1;

    while (1) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(serial_fd, &readfds);
        struct timeval tv;
        struct timeval *ptv = NULL;
        if (timeout_ds >= 0) {
            tv.tv_sec = timeout_ds / 10;
            tv.tv_usec = (timeout_ds % 10) * 100000;
            ptv = &tv;
        }
        int ret = select(serial_fd + 1, &readfds, NULL, NULL, ptv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (ret == 0) return 0;

        int r = readByteSerialPort(byte);
        if (r == -1) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            return -1;
        }
        if (r == 0) continue;
        return 1;
    }
}

static int readSupervisionAC(unsigned char *A, unsigned char *C, int timeout_ds) {
    int state = WAIT_FLAG;
    unsigned char byte = 0;
    int elapsed = 0;

    while (elapsed < timeout_ds) {
        int r = waitForByte(&byte, 1);
        if (r == -1) return -1;
        if (r == 0) {
            elapsed++;
            continue;
        }
        if (r == 1) {
            switch (state) {
            case WAIT_FLAG:
                if (byte == FLAG) state = WAIT_A;
                break;
            case WAIT_A:
                *A = byte;
                state = WAIT_C;
                break;
            case WAIT_C:
                *C = byte;
                state = WAIT_BCC1;
                break;
            case WAIT_BCC1:
                if (byte == ((*A) ^ (*C))) state = WAIT_FLAG_END;
                else state = WAIT_FLAG;
                break;
            case WAIT_FLAG_END:
                if (byte == FLAG) return 1;
                state = WAIT_FLAG;
                break;
            }
        }
    }
    return 0;
}

static int writeAll(const unsigned char *buf, int len) {
    int written = writeBytesSerialPort(buf, len);
    return (written == len) ? 0 : -1;
}

static int stuffBytes(const unsigned char *src, int n, unsigned char *dst) {
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

static unsigned char rrForExpected(int expectedSeq) {
    return expectedSeq == 0 ? C_RR0 : C_RR1;
}

static unsigned char rrForNext(int seqJustReceived) {
    return rrForExpected(seqJustReceived ^ 1);
}

static unsigned char rejForExpected(int expectedSeq) {
    return expectedSeq == 0 ? C_REJ0 : C_REJ1;
}

int llopen(LinkLayer connectionParameters) {
    int fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
    if (fd < 0) return -1;

    linkRole = connectionParameters.role;
    serial_fd = fd;
    tx_seq = 0;
    rx_expected_seq = 0;
    link_timeout_ds = (connectionParameters.timeout > 0) ? connectionParameters.timeout * 10 : 10;
    link_max_retries = (connectionParameters.nRetransmissions >= 0) ? connectionParameters.nRetransmissions : 0;

    if (linkRole == LlTx) {
        unsigned char setFrame[5];
        buildSupervisionFrame(setFrame, A_TX, C_SET);

        for (int attempt = 0; attempt <= link_max_retries; ++attempt) {
            LOG("llopen TX attempt=%d sending SET", attempt);
            if (writeAll(setFrame, 5) != 0) {
                LOG("llopen TX failed writing SET");
                closeSerialPort();
                serial_fd = -1;
                return -1;
            }
            unsigned char A = 0, C = 0;
            int ok = readSupervisionAC(&A, &C, link_timeout_ds);
            LOG("llopen TX wait UA -> status=%d A=0x%02X C=0x%02X", ok, A, C);
            if (ok == 1 && A == A_RX && C == C_UA) return fd;
            if (ok == -1) {
                closeSerialPort();
                serial_fd = -1;
                return -1;
            }
        }

        LOG("llopen TX exhausted retries");
        closeSerialPort();
        serial_fd = -1;
        return -1;
    }

    if (linkRole == LlRx) {
        int state = WAIT_FLAG;
        unsigned char byte = 0;
        int elapsed = 0;
        int limit = link_timeout_ds * (link_max_retries + 1);

        while (elapsed < limit) {
            int r = waitForByte(&byte, 1);
            if (r == -1) {
                closeSerialPort();
                serial_fd = -1;
                return -1;
            }
            if (r == 0) {
                elapsed++;
                continue;
            }
            if (r == 1) {
                LOG("llopen RX state=%d byte=0x%02X", state, byte);
                switch (state) {
                case WAIT_FLAG:
                    if (byte == FLAG) state = WAIT_A;
                    break;
                case WAIT_A:
                    if (byte == A_TX) state = WAIT_C;
                    else state = WAIT_FLAG;
                    break;
                case WAIT_C:
                    if (byte == C_SET) state = WAIT_BCC1;
                    else state = WAIT_FLAG;
                    break;
                case WAIT_BCC1:
                    if (byte == (A_TX ^ C_SET)) state = WAIT_FLAG_END;
                    else state = WAIT_FLAG;
                    break;
                case WAIT_FLAG_END:
                    if (byte == FLAG) {
                        LOG("llopen RX handshake complete, sending UA");
                        unsigned char ua[5];
                        buildSupervisionFrame(ua, A_RX, C_UA);
                        if (writeAll(ua, 5) != 0) {
                            closeSerialPort();
                            serial_fd = -1;
                            return -1;
                        }
                        return fd;
                    }
                    state = WAIT_FLAG;
                    break;
                }
            }
        }

        LOG("llopen RX timeout waiting for SET");
        closeSerialPort();
        serial_fd = -1;
        return -1;
    }

    LOG("llopen invalid role");
    closeSerialPort();
    serial_fd = -1;
    LOG("llwrite exhausted retries");
    return -1;
}

int llwrite(int fd, const unsigned char *buf, int bufSize) {
    (void)fd;
    if (!buf || bufSize < 0 || bufSize > MAX_PAYLOAD_SIZE) return -1;

    unsigned char control = (tx_seq == 0) ? C_I0 : C_I1;
    unsigned char header[4];
    header[0] = FLAG;
    header[1] = A_TX;
    header[2] = control;
    header[3] = bcc1(A_TX, control);

    unsigned char payloadWithBcc[MAX_PAYLOAD_SIZE + 1];
    memcpy(payloadWithBcc, buf, (size_t)bufSize);

    unsigned char bcc2 = 0;
    for (int i = 0; i < bufSize; ++i) bcc2 ^= buf[i];
    payloadWithBcc[bufSize] = bcc2;

    unsigned char stuffed[2 * (MAX_PAYLOAD_SIZE + 1)];
    int stuffedLen = stuffBytes(payloadWithBcc, bufSize + 1, stuffed);

    unsigned char frame[5 + 2 * (MAX_PAYLOAD_SIZE + 1)];
    int offset = 0;
    memcpy(frame + offset, header, sizeof(header));
    offset += (int)sizeof(header);
    memcpy(frame + offset, stuffed, stuffedLen);
    offset += stuffedLen;
    frame[offset++] = FLAG;

    for (int attempt = 0; attempt <= link_max_retries; ++attempt) {
        LOG("llwrite seq=%d size=%d attempt=%d", tx_seq, bufSize, attempt);
        if (writeAll(frame, offset) != 0) {
            LOG("llwrite failed to send frame");
            return -1;
        }

        unsigned char A = 0, C = 0;
        int r = readSupervisionAC(&A, &C, link_timeout_ds);
        LOG("llwrite ack status=%d A=0x%02X C=0x%02X", r, A, C);
        if (r == -1) return -1;
        if (r == 0) continue;
        if (A != A_RX) continue;

        if ((tx_seq == 0 && C == C_RR1) || (tx_seq == 1 && C == C_RR0)) {
            tx_seq ^= 1;
            LOG("llwrite accepted, new seq=%d", tx_seq);
            return bufSize;
        }
        if ((tx_seq == 0 && C == C_REJ0) || (tx_seq == 1 && C == C_REJ1)) continue;
        if ((tx_seq == 0 && C == C_RR0) || (tx_seq == 1 && C == C_RR1)) continue;
    }

    LOG("llwrite exhausted retries");
    return -1;
}

int llread(unsigned char *packet) {
    if (!packet) return -1;

    int state = WAIT_FLAG;
    unsigned char byte = 0;
    unsigned char frameData[MAX_PAYLOAD_SIZE + 1];
    int dataLen = 0;
    int escape = 0;
    unsigned char currentControl = 0;

    while (1) {
        int r = waitForByte(&byte, 1);
        if (r == -1) return -1;
        if (r == 0) {
            continue;
        }

        LOG("llread state=%d byte=0x%02X", state, byte);
        LOG("llclose RX state=%d byte=0x%02X", state, byte);
        switch (state) {
        case WAIT_FLAG:
            if (byte == FLAG) {
                LOG("llread saw FLAG");
                state = WAIT_A;
                escape = 0;
                dataLen = 0;
            }
            break;
        case WAIT_A:
            if (byte == A_TX) {
                state = WAIT_C;
            } else {
                state = WAIT_FLAG;
            }
            break;
        case WAIT_C:
            if (byte == C_I0 || byte == C_I1) {
                currentControl = byte;
                int seq = (byte == C_I0) ? 0 : 1;
                LOG("llread control=0x%02X seq=%d expected=%d", byte, seq, rx_expected_seq);
                if (seq == rx_expected_seq) {
                    state = WAIT_BCC1;
                } else if (seq == (rx_expected_seq ^ 1)) {
                    unsigned char rr[5];
                    buildSupervisionFrame(rr, A_RX, rrForExpected(rx_expected_seq));
                    writeAll(rr, 5);
                    state = WAIT_FLAG;
                } else {
                    state = WAIT_FLAG;
                }
            } else {
                state = WAIT_FLAG;
            }
            break;
        case WAIT_BCC1:
            if (byte == bcc1(A_TX, currentControl)) {
                state = READ_DATA;
                escape = 0;
                dataLen = 0;
            } else {
                state = WAIT_FLAG;
            }
            break;
        case READ_DATA:
            if (!escape && byte == FLAG) {
                if (dataLen < 1) {
                    state = WAIT_FLAG;
                    break;
                }
                int seq = (currentControl == C_I0) ? 0 : 1;
                unsigned char calc = 0;
                for (int i = 0; i < dataLen - 1; ++i) calc ^= frameData[i];
                if (calc == frameData[dataLen - 1]) {
                    unsigned char rr[5];
                    buildSupervisionFrame(rr, A_RX, rrForNext(seq));
                    writeAll(rr, 5);
                    int payloadSize = dataLen - 1;
                    memcpy(packet, frameData, (size_t)payloadSize);
                    rx_expected_seq ^= 1;
                    LOG("llread delivered seq=%d payload=%d nextExpected=%d", seq, payloadSize, rx_expected_seq);
                    return payloadSize;
                } else {
                    LOG("llread BCC2 mismatch seq=%d", seq);
                    unsigned char rej[5];
                    buildSupervisionFrame(rej, A_RX, rejForExpected(rx_expected_seq));
                    writeAll(rej, 5);
                    state = WAIT_FLAG;
                    dataLen = 0;
                    escape = 0;
                }
                break;
            }
            if (!escape && byte == ESC) {
                escape = 1;
                break;
            }
            if (escape) {
                if (byte == ESC_FLAG) {
                    byte = FLAG;
                } else if (byte == ESC_ESC) {
                    byte = ESC;
                } else {
                    state = WAIT_FLAG;
                    dataLen = 0;
                    escape = 0;
                    break;
                }
                escape = 0;
            }
            if (dataLen < (int)sizeof(frameData)) {
                frameData[dataLen++] = byte;
            } else {
                state = WAIT_FLAG;
                dataLen = 0;
                escape = 0;
            }
            break;
        default:
            state = WAIT_FLAG;
            break;
        }
    }
}

int llclose(int fd) {
    (void)fd;
    unsigned char byte = 0;
    int state = WAIT_FLAG;

    if (linkRole == LlTx) {
        unsigned char disc[5];
        buildSupervisionFrame(disc, A_TX, C_DISC);
        unsigned char ua[5];
        buildSupervisionFrame(ua, A_RX, C_UA);

        for (int attempt = 0; attempt <= link_max_retries; ++attempt) {
            LOG("llclose TX attempt=%d sending DISC", attempt);
            if (writeAll(disc, 5) != 0) {
                closeSerialPort();
                serial_fd = -1;
                return -1;
            }
            state = WAIT_FLAG;
            int elapsed = 0;
            while (elapsed < link_timeout_ds) {
                int r = waitForByte(&byte, 1);
                if (r == -1) {
                    closeSerialPort();
                    serial_fd = -1;
                    return -1;
                }
                if (r == 0) {
                    elapsed++;
                    continue;
                }
                if (r == 1) {
                    LOG("llclose TX state=%d byte=0x%02X", state, byte);
                    switch (state) {
                    case WAIT_FLAG:
                        if (byte == FLAG) state = WAIT_A;
                        break;
                    case WAIT_A:
                        if (byte == A_RX) state = WAIT_C;
                        else state = WAIT_FLAG;
                        break;
                    case WAIT_C:
                        if (byte == C_DISC) state = WAIT_BCC1;
                        else state = WAIT_FLAG;
                        break;
                    case WAIT_BCC1:
                        if (byte == (A_RX ^ C_DISC)) state = WAIT_FLAG_END;
                        else state = WAIT_FLAG;
                        break;
                    case WAIT_FLAG_END:
                        if (byte == FLAG) {
                            LOG("llclose TX received DISC, sending UA");
                            writeAll(ua, 5);
                            closeSerialPort();
                            serial_fd = -1;
                            return 0;
                        }
                        state = WAIT_FLAG;
                        break;
                    default:
                        state = WAIT_FLAG;
                        break;
                    }
                }
            }
        }

        closeSerialPort();
        serial_fd = -1;
        return -1;
    }

    unsigned char discResp[5];
    buildSupervisionFrame(discResp, A_RX, C_DISC);

    while (1) {
        int r = waitForByte(&byte, 1);
        if (r == -1) {
            closeSerialPort();
            serial_fd = -1;
            return -1;
        }
        if (r == 0) continue;

        switch (state) {
        case WAIT_FLAG:
            if (byte == FLAG) state = WAIT_A;
            break;
        case WAIT_A:
            if (byte == A_TX) state = WAIT_C;
            else state = WAIT_FLAG;
            break;
        case WAIT_C:
            if (byte == C_DISC) state = WAIT_BCC1;
            else state = WAIT_FLAG;
            break;
        case WAIT_BCC1:
            if (byte == (A_TX ^ C_DISC)) state = WAIT_FLAG_END;
            else state = WAIT_FLAG;
            break;
        case WAIT_FLAG_END:
            if (byte == FLAG) {
                LOG("llclose RX received DISC, replying");
                if (writeAll(discResp, 5) != 0) {
                    closeSerialPort();
                    serial_fd = -1;
                    return -1;
                }
                unsigned char b = 0;
                int innerState = WAIT_FLAG;
                int elapsed = 0;
                while (elapsed < link_timeout_ds) {
                    int r2 = waitForByte(&b, 1);
                    if (r2 == -1) {
                        closeSerialPort();
                        serial_fd = -1;
                        return -1;
                    }
                    if (r2 == 0) {
                        elapsed++;
                        continue;
                    }
                    if (r2 == 1) {
                        LOG("llclose RX UA state=%d byte=0x%02X", innerState, b);
                        switch (innerState) {
                        case WAIT_FLAG:
                            if (b == FLAG) innerState = WAIT_A;
                            break;
                        case WAIT_A:
                            if (b == A_RX) innerState = WAIT_C;
                            else innerState = WAIT_FLAG;
                            break;
                        case WAIT_C:
                            if (b == C_UA) innerState = WAIT_BCC1;
                            else innerState = WAIT_FLAG;
                            break;
                        case WAIT_BCC1:
                            if (b == (A_RX ^ C_UA)) innerState = WAIT_FLAG_END;
                            else innerState = WAIT_FLAG;
                            break;
                        case WAIT_FLAG_END:
                            if (b == FLAG) {
                                LOG("llclose RX got UA, closing");
                                closeSerialPort();
                                serial_fd = -1;
                                return 0;
                            }
                            innerState = WAIT_FLAG;
                            break;
                        default:
                            innerState = WAIT_FLAG;
                            break;
                        }
                    }
                }
                closeSerialPort();
                serial_fd = -1;
                return -1;
            }
            state = WAIT_FLAG;
            break;
        default:
            state = WAIT_FLAG;
            break;
        }
    }

    return -1;
}
