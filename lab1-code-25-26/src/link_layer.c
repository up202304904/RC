// Link layer protocol implementation
#include "link_layer.h"
#include "serial_port.h"
#include "link_layer_helpers.h"

#include <stdio.h>
#include <string.h>


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

//variáveis para as estatísticas finais
static unsigned long total_frames_sent = 0;
static unsigned long total_retransmissions = 0;
static unsigned long total_bytes_sent = 0;
static unsigned long total_bytes_received = 0;
static unsigned long timeout_count = 0;

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
        ll_build_supervision_frame(setFrame, A_TX, C_SET);

        for (int attempt = 0; attempt <= link_max_retries; ++attempt) {
            fprintf(stderr, "[llopen][TX] attempt=%d -> enviar SET\n", attempt);

            if (ll_write_all(setFrame, 5) != 0) {
                fprintf(stderr, "[llopen][TX] erro a enviar SET\n");
                closeSerialPort();
                serial_fd = -1;
                return -1;
            }

            unsigned char A = 0, C = 0;
            int ok = ll_read_supervision(serial_fd, &A, &C, link_timeout_ds);
            fprintf(stderr, "[llopen][TX] à espera de UA -> status=%d A=0x%02X C=0x%02X\n", ok, A, C);

            if (ok == 1 && A == A_RX && C == C_UA) {
                fprintf(stderr, "[llopen][TX] UA recebido, ligação bem sucedida\n");
                return fd;
            }
            if (ok == -1) {
                fprintf(stderr, "[llopen][TX] erro a ler supervision\n");
                closeSerialPort();
                serial_fd = -1;
                return -1;
            }
        }

        fprintf(stderr, "[llopen][TX] erro a efetuar ligação\n");
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
            int r = ll_wait_for_byte(serial_fd, 1, &byte);
            if (r == -1) {
                fprintf(stderr, "[llopen][RX] erro em wait_for_byte\n");
                closeSerialPort();
                serial_fd = -1;
                return -1;
            }
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
                        fprintf(stderr, "[llopen][RX] SET recebido ->  a enviar UA\n");
                        unsigned char ua[5];
                        ll_build_supervision_frame(ua, A_RX, C_UA);
                        if (ll_write_all(ua, 5) != 0) {
                            fprintf(stderr, "[llopen][RX] erro a enviar UA\n");
                            closeSerialPort();
                            serial_fd = -1;
                            return -1;
                        }
                        fprintf(stderr, "[llopen][RX] ligação bem sucedida\n");
                        return fd;
                    }
                    state = WAIT_FLAG;
                    break;
                }
            }
        }

        fprintf(stderr, "[llopen][RX] timeout à espera de SET\n");
        closeSerialPort();
        serial_fd = -1;
        return -1;
    }

    closeSerialPort();
    serial_fd = -1;
    return -1;
}

/* ---------------------------------------------------- */
int llwrite(int fd, const unsigned char *buf, int bufSize) {
    (void)fd;
    if (!buf || bufSize < 0 || bufSize > MAX_PAYLOAD_SIZE) return -1;

    unsigned char control = (tx_seq == 0) ? C_I0 : C_I1;
    unsigned char header[4];
    header[0] = FLAG;
    header[1] = A_TX;
    header[2] = control;
    header[3] = ll_compute_bcc1(A_TX, control);

    unsigned char payloadWithBcc[MAX_PAYLOAD_SIZE + 1];
    memcpy(payloadWithBcc, buf, (size_t)bufSize);

    unsigned char bcc2 = ll_compute_bcc2(buf, bufSize);
    payloadWithBcc[bufSize] = bcc2;

    unsigned char stuffed[2 * (MAX_PAYLOAD_SIZE + 1)];
    int stuffedLen = ll_stuff_bytes(payloadWithBcc, bufSize + 1, stuffed);

    unsigned char frame[5 + 2 * (MAX_PAYLOAD_SIZE + 1)];
    int offset = 0;
    memcpy(frame + offset, header, sizeof(header));
    offset += (int)sizeof(header);
    memcpy(frame + offset, stuffed, stuffedLen);
    offset += stuffedLen;
    frame[offset++] = FLAG;

    for (int attempt = 0; attempt <= link_max_retries; ++attempt) {
        fprintf(stderr, "[llwrite] seq=%d attempt=%d -> enviar I-frame (%d bytes)\n",
                tx_seq, attempt, bufSize);

        if (ll_write_all(frame, offset) != 0) {
            fprintf(stderr, "[llwrite] erro ao enviar frame\n");
            return -1;
        }

        unsigned char A = 0, C = 0;
        int r = ll_read_supervision(serial_fd, &A, &C, link_timeout_ds);
        fprintf(stderr, "[llwrite] ack status=%d A=0x%02X C=0x%02X\n", r, A, C);

        if (r == -1) return -1;
        if (r == 0) {
            fprintf(stderr, "[llwrite] timeout à espera de RR/REJ\n");
            continue;
        }
        if (A != A_RX) {
            fprintf(stderr, "[llwrite] A inesperado (A=0x%02X) -> ignorar\n", A);
            continue;
        }

        if ((tx_seq == 0 && C == C_RR1) || (tx_seq == 1 && C == C_RR0)) {
            fprintf(stderr, "[llwrite] RR(next) recebido -> avançar seq %d → %d\n", tx_seq, tx_seq ^ 1);
            tx_seq ^= 1;
            return bufSize;
        }
        if ((tx_seq == 0 && C == C_REJ0) || (tx_seq == 1 && C == C_REJ1)) {
            fprintf(stderr, "[llwrite] REJ(expected) recebido -> retransmitir mesma seq=%d\n", tx_seq);
            continue;
        }
        if ((tx_seq == 0 && C == C_RR0) || (tx_seq == 1 && C == C_RR1)) {
            fprintf(stderr, "[llwrite] RR(expected) recebido (duplicado provável no RX) -> retransmitir\n");
            continue;
        }

        fprintf(stderr, "[llwrite] controlo inesperado C=0x%02X\n", C);
    }

    fprintf(stderr, "[llwrite] esgotou tentativas\n");
    return -1;
}

/* ---------------------------------------------------- */
int llread(unsigned char *packet) {
    if (!packet) return -1;

    int state = WAIT_FLAG;
    unsigned char byte = 0;
    unsigned char frameData[MAX_PAYLOAD_SIZE + 1];
    int dataLen = 0;
    int escape = 0;
    unsigned char currentControl = 0;

    while (1) {
        int r = ll_wait_for_byte(serial_fd, 1, &byte);
        if (r == -1) {
            fprintf(stderr, "[llread] erro em wait_for_byte\n");
            return -1;
        }
        if (r == 0) {
            continue;
        }

        switch (state) {
        case WAIT_FLAG:
            if (byte == FLAG) {
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
                fprintf(stderr, "[llread] CONTROL=0x%02X seq=%d expected=%d\n",
                        byte, seq, rx_expected_seq);

                if (seq == rx_expected_seq) {
                    state = WAIT_BCC1;
                } else if (seq == (rx_expected_seq ^ 1)) {
                    unsigned char rr[5];
                    ll_build_supervision_frame(rr, A_RX, ll_rr_for_expected(rx_expected_seq));
                    fprintf(stderr, "[llread] DUPLICATE seq=%d -> enviar RR(expected=%d) e descartar\n",
                            seq, rx_expected_seq);
                    ll_write_all(rr, 5);
                    state = WAIT_FLAG;
                } else {
                    fprintf(stderr, "[llread] seq inválido\n");
                    state = WAIT_FLAG;
                }
            } else {
                state = WAIT_FLAG;
            }
            break;
        case WAIT_BCC1:
            if (byte == ll_compute_bcc1(A_TX, currentControl)) {
                state = READ_DATA;
                escape = 0;
                dataLen = 0;
            } else {
                fprintf(stderr, "[llread] BCC1 inválido -> reset\n");
                state = WAIT_FLAG;
            }
            break;
        case READ_DATA:
            if (!escape && byte == FLAG) {
                if (dataLen < 1) {
                    fprintf(stderr, "[llread] frame demasiado curto\n");
                    state = WAIT_FLAG;
                    break;
                }
                {
                    int seq = (currentControl == C_I0) ? 0 : 1;
                    unsigned char calc = ll_compute_bcc2(frameData, dataLen - 1);

                    if (calc == frameData[dataLen - 1]) {
                        unsigned char rr[5];
                        ll_build_supervision_frame(rr, A_RX, ll_rr_for_next(seq));
                        fprintf(stderr, "[llread] BCC2 OK seq=%d, payload=%d -> enviar RR(next=%d)\n",
                                seq, dataLen - 1, seq ^ 1);
                        ll_write_all(rr, 5);

                        int payloadSize = dataLen - 1;
                        memcpy(packet, frameData, (size_t)payloadSize);
                        rx_expected_seq ^= 1;
                        fprintf(stderr, "[llread] entregue payload size=%d, nextExpected=%d\n",
                                payloadSize, rx_expected_seq);
                        return payloadSize;
                    } else {
                        unsigned char rej[5];
                        ll_build_supervision_frame(rej, A_RX, ll_rej_for_expected(rx_expected_seq));
                        fprintf(stderr, "[llread] BCC2 ERR seq=%d -> enviar REJ(expected=%d)\n",
                                seq, rx_expected_seq);
                        ll_write_all(rej, 5);
                        state = WAIT_FLAG;
                        dataLen = 0;
                        escape = 0;
                    }
                }
                break;
            }
            if (!escape && byte == ESC) {
                fprintf(stderr, "[llread] ESC -> próxima será unescaped\n");
                escape = 1;
                break;
            }
            if (escape) {
                fprintf(stderr, "[llread] unescape byte=0x%02X\n", byte);
                if (byte == ESC_FLAG) {
                    byte = FLAG;
                } else if (byte == ESC_ESC) {
                    byte = ESC;
                } else {
                    fprintf(stderr, "[llread] sequência de escape inválida -> reset\n");
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
                fprintf(stderr, "[llread] overflow de frameData -> reset\n");
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

/* ---------------------------------------------------- */
int llclose(int fd) {
    (void)fd;
    unsigned char byte = 0;
    int state = WAIT_FLAG;

    fprintf(stderr, "[llclose][%s] iniciar encerramento\n",
            (linkRole == LlTx) ? "TX" : "RX");

    if (linkRole == LlTx) {
        unsigned char disc[5];
        ll_build_supervision_frame(disc, A_TX, C_DISC);
        unsigned char ua[5];
        ll_build_supervision_frame(ua, A_RX, C_UA);

        for (int attempt = 0; attempt <= link_max_retries; ++attempt) {
            fprintf(stderr, "[llclose][TX] attempt=%d -> enviar DISC\n", attempt);

            if (ll_write_all(disc, 5) != 0) {
                fprintf(stderr, "[llclose][TX] erro a enviar DISC\n");
                closeSerialPort();
                serial_fd = -1;
                return -1;
            }
            state = WAIT_FLAG;
            int elapsed = 0;
            while (elapsed < link_timeout_ds) {
                int r = ll_wait_for_byte(serial_fd, 1, &byte);
                if (r == -1) {
                    fprintf(stderr, "[llclose][TX] erro em wait_for_byte\n");
                    closeSerialPort();
                    serial_fd = -1;
                    return -1;
                }
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
                            fprintf(stderr, "[llclose][TX] DISC recebido -> a enviar UA e fechar\n");
                            ll_write_all(ua, 5);
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

        fprintf(stderr, "[llclose][TX] esgotou tentativas\n");
        closeSerialPort();
        serial_fd = -1;
        return -1;
    }

    // RX 
    unsigned char discResp[5];
    ll_build_supervision_frame(discResp, A_RX, C_DISC);

    while (1) {
        int r = ll_wait_for_byte(serial_fd, 1, &byte);
        if (r == -1) {
            fprintf(stderr, "[llclose][RX] erro em wait_for_byte\n");
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
                fprintf(stderr, "[llclose][RX] DISC recebido -> enviar DISC de resposta e esperar UA\n");
                if (ll_write_all(discResp, 5) != 0) {
                    fprintf(stderr, "[llclose][RX] erro a enviar DISC de resposta\n");
                    closeSerialPort();
                    serial_fd = -1;
                    return -1;
                }
                unsigned char b = 0;
                int innerState = WAIT_FLAG;
                int elapsed = 0;
                while (elapsed < link_timeout_ds) {
                    int r2 = ll_wait_for_byte(serial_fd, 1, &b);
                    if (r2 == -1) {
                        fprintf(stderr, "[llclose][RX] erro a esperar UA\n");
                        closeSerialPort();
                        serial_fd = -1;
                        return -1;
                    }
                    if (r2 == 0) {
                        elapsed++;
                        continue;
                    }
                    if (r2 == 1) {
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
                                fprintf(stderr, "[llclose][RX] UA recebido -> fechar ligação\n");
                                fprintf(stderr, "\n=== Transmission Statistics ===\n");
                                fprintf(stderr, "Bytes sent:          %lu\n", total_bytes_sent);
                                fprintf(stderr, "Bytes received:      %lu\n", total_bytes_received);
                                fprintf(stderr, "Frames sent:         %lu\n", total_frames_sent);
                                fprintf(stderr, "Retransmissions:     %lu\n", total_retransmissions);
                                fprintf(stderr, "Timeouts:            %lu\n", timeout_count);
                                fprintf(stderr, "===============================\n");
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
                fprintf(stderr, "[llclose][RX] timeout à espera de UA\n");
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
