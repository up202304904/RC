// Application layer protocol implementation
#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#define CTRL_DATA  0x01
#define CTRL_START 0x02
#define CTRL_END   0x03

#define TLV_FILESIZE 0x00
#define TLV_FILENAME 0x01

#define DATA_HEADER_SIZE 4
#define DATA_CHUNK 256

static int isTransmitter(const char *role) {
    return role && (strcmp(role, "tx") == 0 || strcmp(role, "TX") == 0);
}

static size_t encodeFileSize(size_t value, unsigned char *buffer) {
    unsigned char tmp[sizeof(size_t)];
    size_t len = 0;

    if (value == 0) {
        buffer[0] = 0;
        return 1;
    }

    while (value > 0) {
        tmp[len++] = (unsigned char)(value & 0xFF);
        value >>= 8;
    }
    for (size_t i = 0; i < len; ++i) {
        buffer[i] = tmp[len - 1 - i];
    }
    return len;
}

static size_t decodeFileSize(const unsigned char *value, size_t length) {
    size_t out = 0;
    for (size_t i = 0; i < length; ++i) {
        out = (out << 8) | value[i];
    }
    return out;
}

static int buildControlPacket(unsigned char *out, size_t capacity, unsigned char ctrl,
                              const char *filename, size_t fileSize) {
    unsigned char sizeField[sizeof(size_t)];
    size_t sizeLen = encodeFileSize(fileSize, sizeField);
    size_t nameLen = filename ? strlen(filename) : 0;

    size_t needed = 1                                       // control
                    + 1 + 1 + sizeLen                       // TLV filesize
                    + (filename ? (1 + 1 + nameLen) : 0);   // TLV filename

    if (needed > capacity) return -1;

    size_t idx = 0;
    out[idx++] = ctrl;

    out[idx++] = TLV_FILESIZE;
    out[idx++] = (unsigned char)sizeLen;
    memcpy(&out[idx], sizeField, sizeLen);
    idx += sizeLen;

    if (filename) {
        out[idx++] = TLV_FILENAME;
        out[idx++] = (unsigned char)nameLen;
        memcpy(&out[idx], filename, nameLen);
        idx += nameLen;
    }

    return (int)idx;
}

static int parseControlPacket(const unsigned char *packet, size_t length,
                              size_t *outSize, char *outName, size_t outNameSize) {
    if (length < 3) return -1;
    size_t idx = 1;
    size_t fileSize = 0;
    char nameBuffer[256] = {0};
    int haveSize = 0;
    int haveName = 0;

    while (idx + 2 <= length) {
        unsigned char type = packet[idx++];
        unsigned char len = packet[idx++];
        if (idx + len > length) return -1;

        if (type == TLV_FILESIZE) {
            fileSize = decodeFileSize(&packet[idx], len);
            haveSize = 1;
        } else if (type == TLV_FILENAME) {
            size_t copyLen = (len < sizeof(nameBuffer) - 1) ? len : sizeof(nameBuffer) - 1;
            memcpy(nameBuffer, &packet[idx], copyLen);
            nameBuffer[copyLen] = '\0';
            haveName = 1;
        }
        idx += len;
    }

    if (!haveSize) return -1;
    if (outSize) *outSize = fileSize;
    if (outName && outNameSize > 0) {
        if (haveName) {
            strncpy(outName, nameBuffer, outNameSize - 1);
            outName[outNameSize - 1] = '\0';
        } else {
            outName[0] = '\0';
        }
    }
    return 0;
}

static int buildDataPacket(unsigned char *out, size_t capacity, unsigned char seq,
                           const unsigned char *data, size_t dataLen) {
    if (DATA_HEADER_SIZE + dataLen > capacity) return -1;
    out[0] = CTRL_DATA;
    out[1] = seq;
    out[2] = (unsigned char)((dataLen >> 8) & 0xFF);
    out[3] = (unsigned char)(dataLen & 0xFF);
    memcpy(&out[4], data, dataLen);
    return (int)(DATA_HEADER_SIZE + dataLen);
}

static int sendControlFrame(int fd, unsigned char ctrl, const char *filename, size_t fileSize) {
    unsigned char packet[MAX_PAYLOAD_SIZE];
    int len = buildControlPacket(packet, sizeof(packet), ctrl, filename, fileSize);
    if (len < 0) return -1;
    return llwrite(fd, packet, len);
}

static int sendFileData(int fd, FILE *file) {
    unsigned char packet[DATA_HEADER_SIZE + DATA_CHUNK];
    unsigned char chunk[DATA_CHUNK];
    unsigned int seq = 0;
    size_t bytesRead;

    while ((bytesRead = fread(chunk, 1, sizeof(chunk), file)) > 0) {
        int packetLen = buildDataPacket(packet, sizeof(packet), (unsigned char)(seq % 256), chunk, bytesRead);
        if (packetLen < 0) return -1;
        if (llwrite(fd, packet, packetLen) < 0) return -1;
        seq = (seq + 1) % 256;
    }

    return ferror(file) ? -1 : 0;
}

static int receiveStartFrame(size_t *expectedSize, char *remoteName, size_t nameSize) {
    unsigned char packet[MAX_PAYLOAD_SIZE];
    while (1) {
        int len = llread(packet);
        if (len <= 0) return -1;
        if (packet[0] == CTRL_START) {
            if (parseControlPacket(packet, (size_t)len, expectedSize, remoteName, nameSize) == 0) return 0;
            return -1;
        }
    }
}

static int receiveUntilEnd(FILE *out, size_t expectedSize) {
    unsigned char packet[MAX_PAYLOAD_SIZE];
    unsigned int expectedSeq = 0;
    size_t bytesWritten = 0;

    while (1) {
        int len = llread(packet);
        if (len <= 0) return -1;

        if (packet[0] == CTRL_DATA) {
            if (len < DATA_HEADER_SIZE) continue;
            unsigned int seq = packet[1];
            size_t dataLen = ((size_t)packet[2] << 8) | packet[3];
            if (dataLen != (size_t)(len - DATA_HEADER_SIZE)) continue;
            if (seq != expectedSeq) continue;
            size_t wrote = fwrite(&packet[DATA_HEADER_SIZE], 1, dataLen, out);
            if (wrote != dataLen) return -1;
            bytesWritten += dataLen;
            expectedSeq = (expectedSeq + 1) % 256;
        } else if (packet[0] == CTRL_END) {
            size_t sizeFromEnd = 0;
            if (parseControlPacket(packet, (size_t)len, &sizeFromEnd, NULL, 0) != 0) return -1;
            if (expectedSize != 0 && sizeFromEnd != 0 && sizeFromEnd != expectedSize) return -1;
            if (expectedSize != 0 && bytesWritten != expectedSize) return -1;
            return 0;
        }
    }
}

static void closeAndReport(int fd, int hadError) {
    if (llclose(fd) != 0 || hadError) {
        fprintf(stderr, "applicationLayer: error closing link\n");
    }
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename) {
    if (!serialPort || !role || !filename) {
        fprintf(stderr, "applicationLayer: invalid arguments\n");
        return;
    }

    LinkLayer connectionParameters;
    memset(&connectionParameters, 0, sizeof(connectionParameters));
    strncpy(connectionParameters.serialPort, serialPort, sizeof(connectionParameters.serialPort) - 1);
    connectionParameters.serialPort[sizeof(connectionParameters.serialPort) - 1] = '\0';
    connectionParameters.role = isTransmitter(role) ? LlTx : LlRx;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

    int fd = llopen(connectionParameters);
    if (fd < 0) {
        fprintf(stderr, "applicationLayer: llopen failed\n");
        return;
    }

    if (connectionParameters.role == LlTx) {
        struct stat st;
        if (stat(filename, &st) != 0) {
            perror("stat");
            closeAndReport(fd, 1);
            return;
        }

        FILE *file = fopen(filename, "rb");
        if (!file) {
            perror("fopen");
            closeAndReport(fd, 1);
            return;
        }

        int error = 0;
        if (sendControlFrame(fd, CTRL_START, filename, (size_t)st.st_size) < 0) {
            fprintf(stderr, "Error sending START control frame\n");
            error = 1;
        } else if (sendFileData(fd, file) != 0) {
            fprintf(stderr, "Error sending file data\n");
            error = 1;
        } else if (sendControlFrame(fd, CTRL_END, filename, (size_t)st.st_size) < 0) {
            fprintf(stderr, "Error sending END control frame\n");
            error = 1;
        } else {
            printf("Transmission complete (%lld bytes).\n", (long long)st.st_size);
        }

        fclose(file);
        closeAndReport(fd, error);
    } else {
        size_t expectedSize = 0;
        char remoteName[128];
        if (receiveStartFrame(&expectedSize, remoteName, sizeof(remoteName)) != 0) {
            fprintf(stderr, "Failed to receive START control frame\n");
            closeAndReport(fd, 1);
            return;
        }

        FILE *out = fopen(filename, "wb");
        if (!out) {
            perror("fopen");
            closeAndReport(fd, 1);
            return;
        }

        int error = receiveUntilEnd(out, expectedSize);
        fclose(out);

        if (error != 0) {
            fprintf(stderr, "Error receiving file data\n");
            closeAndReport(fd, 1);
            return;
        }

        printf("Reception complete (%lld bytes).\n", (long long)expectedSize);
        closeAndReport(fd, 0);
    }
}
