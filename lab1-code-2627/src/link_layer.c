// RCOM 2026/2027
//
// Link layer protocol implementation with Statistics

#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>

// MISC
#define _POSIX_SOURCE 1
#define BUF_SIZE 2048

#define FLAG 0x7E
#define A_TX 0x03
#define A_RX 0x01

#define C_SET 0x03
#define C_UA 0x07
#define C_DISC 0x0B
#define C_RR0 0xAA
#define C_RR1 0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55
#define C_I0 0x00
#define C_I1 0x80

#define ESC 0x7D
#define ESC_7E 0x5E
#define ESC_7D 0x5D

typedef enum
{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    READ_DATA,
    STOP_STATE
} State;

static LinkLayer connectionParameters;
static int frameTxNum = 0; // N(s)
static int frameRxNum = 0; // N(r)

// Protocol Statistics
typedef struct {
    int iFramesSent;
    int iFramesReceived;
    int retransmissions;
    int timeouts;
    int bcc2Errors;
    int rejSent;
    int rejReceived;
} ProtocolStats;

static ProtocolStats stats = {0};

// Alarm variables
volatile sig_atomic_t alarmEnabled = FALSE;
volatile sig_atomic_t alarmCount = 0;

void alarmHandler(int signal)
{
    (void)signal;
    alarmEnabled = FALSE;
    alarmCount++;
    stats.timeouts++;
    printf("Alarm #%d received (Timeout)\n", (int)alarmCount);
}

static int setupAlarm()
{
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    sigemptyset(&act.sa_mask);
    act.sa_flags = 0;

    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        return -1;
    }
    return 0;
}

static int readFrame(unsigned char *address, unsigned char *control, unsigned char *payload, int *payloadSize)
{
    State state = START;
    unsigned char byte;
    int payloadIdx = 0;
    int inEscape = FALSE;

    while (state != STOP_STATE)
    {
        int res = readByteSerialPort(&byte);
        if (res < 0)
        {
            if (errno == EINTR)
                continue;
            return -1;
        }
        if (res == 0)
        {
            if (!alarmEnabled && alarmCount > 0)
                return -1;
            continue;
        }

        switch (state)
        {
        case START:
            if (byte == FLAG)
                state = FLAG_RCV;
            break;

        case FLAG_RCV:
            if (byte == FLAG)
            {
                // Wait for valid byte
            }
            else if (byte == A_TX || byte == A_RX)
            {
                *address = byte;
                state = A_RCV;
            }
            else
            {
                state = START;
            }
            break;

        case A_RCV:
            if (byte == FLAG)
            {
                state = FLAG_RCV;
            }
            else
            {
                *control = byte;
                state = C_RCV;
            }
            break;

        case C_RCV:
            if (byte == FLAG)
            {
                state = FLAG_RCV;
            }
            else if (byte == (*address ^ *control))
            {
                if (*control == C_I0 || *control == C_I1)
                {
                    payloadIdx = 0;
                    inEscape = FALSE;
                    state = READ_DATA;
                }
                else
                {
                    state = BCC_OK;
                }
            }
            else
            {
                state = START;
            }
            break;

        case BCC_OK:
            if (byte == FLAG)
            {
                state = STOP_STATE;
            }
            else
            {
                state = START;
            }
            break;

        case READ_DATA:
            if (byte == FLAG)
            {
                if (payloadIdx < 1)
                {
                    state = START;
                    break;
                }
                
                unsigned char receivedBcc2 = payload[payloadIdx - 1];
                int dataSize = payloadIdx - 1;

                unsigned char expectedBcc2 = 0;
                for (int i = 0; i < dataSize; i++)
                {
                    expectedBcc2 ^= payload[i];
                }

                if (receivedBcc2 != expectedBcc2)
                {
                    stats.bcc2Errors++;
                    return -2; // BCC2 Error
                }

                if (payloadSize != NULL)
                {
                    *payloadSize = dataSize;
                }
                state = STOP_STATE;
            }
            else if (byte == ESC)
            {
                inEscape = TRUE;
            }
            else
            {
                if (inEscape)
                {
                    if (byte == ESC_7E)
                        payload[payloadIdx++] = FLAG;
                    else if (byte == ESC_7D)
                        payload[payloadIdx++] = ESC;
                    else
                    {
                        state = START;
                        break;
                    }
                    inEscape = FALSE;
                }
                else
                {
                    payload[payloadIdx++] = byte;
                }
            }
            break;

        default:
            state = START;
            break;
        }
    }

    return 0;
}

int llOpenTx(LinkLayer llParameters)
{
    connectionParameters = llParameters;
    frameTxNum = 0;
    memset(&stats, 0, sizeof(stats));

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    if (setupAlarm() < 0)
        return -1;

    unsigned char setFrame[5] = {FLAG, A_TX, C_SET, (unsigned char)(A_TX ^ C_SET), FLAG};

    alarmCount = 0;
    alarmEnabled = FALSE;

    while (alarmCount <= llParameters.nRetransmissions)
    {
        writeBytesSerialPort(setFrame, 5);

        alarmEnabled = TRUE;
        alarm(llParameters.timeout);

        while (alarmEnabled)
        {
            unsigned char addr, ctrl;
            int res = readFrame(&addr, &ctrl, NULL, NULL);
            if (res == 0 && addr == A_TX && ctrl == C_UA)
            {
                alarm(0);
                alarmEnabled = FALSE;
                printf("Connection established (SET -> UA)\n");
                return 0;
            }
        }

        if (alarmCount > 0)
            stats.retransmissions++;
    }

    alarm(0);
    printf("Connection failed.\n");
    return -1;
}

int llOpenRx(LinkLayer llParameters)
{
    connectionParameters = llParameters;
    frameRxNum = 0;
    memset(&stats, 0, sizeof(stats));

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    unsigned char addr, ctrl;
    while (1)
    {
        int res = readFrame(&addr, &ctrl, NULL, NULL);
        if (res == 0 && addr == A_TX && ctrl == C_SET)
        {
            break;
        }
    }

    unsigned char uaFrame[5] = {FLAG, A_TX, C_UA, (unsigned char)(A_TX ^ C_UA), FLAG};
    writeBytesSerialPort(uaFrame, 5);
    printf("Connection established (received SET, sent UA)\n");

    return 0;
}

int llSend(const unsigned char *buf, int bufSize)
{
    if (bufSize > MAX_PAYLOAD_SIZE)
        return -1;

    unsigned char frame[BUF_SIZE];
    int idx = 0;

    unsigned char cByte = (frameTxNum == 0) ? C_I0 : C_I1;

    frame[idx++] = FLAG;
    frame[idx++] = A_TX;
    frame[idx++] = cByte;
    frame[idx++] = A_TX ^ cByte;

    unsigned char bcc2 = 0;
    for (int i = 0; i < bufSize; i++)
    {
        bcc2 ^= buf[i];
    }

    for (int i = 0; i < bufSize + 1; i++)
    {
        unsigned char b = (i < bufSize) ? buf[i] : bcc2;
        if (b == FLAG)
        {
            frame[idx++] = ESC;
            frame[idx++] = ESC_7E;
        }
        else if (b == ESC)
        {
            frame[idx++] = ESC;
            frame[idx++] = ESC_7D;
        }
        else
        {
            frame[idx++] = b;
        }
    }

    frame[idx++] = FLAG;

    alarmCount = 0;
    alarmEnabled = FALSE;

    while (alarmCount <= connectionParameters.nRetransmissions)
    {
        writeBytesSerialPort(frame, idx);
        stats.iFramesSent++;

        if (alarmCount > 0)
            stats.retransmissions++;

        alarmEnabled = TRUE;
        alarm(connectionParameters.timeout);

        while (alarmEnabled)
        {
            unsigned char addr, ctrl;
            int res = readFrame(&addr, &ctrl, NULL, NULL);
            if (res == 0 && addr == A_TX)
            {
                unsigned char expectedRr = (frameTxNum == 0) ? C_RR1 : C_RR0;
                unsigned char expectedRej = (frameTxNum == 0) ? C_REJ0 : C_REJ1;

                if (ctrl == expectedRr)
                {
                    alarm(0);
                    alarmEnabled = FALSE;
                    frameTxNum = 1 - frameTxNum;
                    return bufSize;
                }
                else if (ctrl == expectedRej)
                {
                    stats.rejReceived++;
                    alarm(0);
                    alarmEnabled = FALSE;
                    break;
                }
            }
        }
    }

    alarm(0);
    return -1;
}

int llReceive(unsigned char *packet)
{
    unsigned char payload[BUF_SIZE];
    int payloadSize = 0;
    unsigned char addr, ctrl;

    while (1)
    {
        int res = readFrame(&addr, &ctrl, payload, &payloadSize);

        if (res == -2)
        {
            // BCC2 Error: Send REJ
            stats.rejSent++;
            unsigned char rejCtrl = (frameRxNum == 0) ? C_REJ0 : C_REJ1;
            unsigned char rejFrame[5] = {FLAG, A_TX, rejCtrl, (unsigned char)(A_TX ^ rejCtrl), FLAG};
            writeBytesSerialPort(rejFrame, 5);
            continue;
        }

        if (res == 0 && addr == A_TX && (ctrl == C_I0 || ctrl == C_I1))
        {
            int receivedNs = (ctrl == C_I0) ? 0 : 1;

            if (receivedNs == frameRxNum)
            {
                stats.iFramesReceived++;
                memcpy(packet, payload, payloadSize);
                frameRxNum = 1 - frameRxNum;

                unsigned char rrCtrl = (frameRxNum == 0) ? C_RR0 : C_RR1;
                unsigned char rrFrame[5] = {FLAG, A_TX, rrCtrl, (unsigned char)(A_TX ^ rrCtrl), FLAG};
                writeBytesSerialPort(rrFrame, 5);

                return payloadSize;
            }
            else
            {
                // Duplicate frame
                unsigned char rrCtrl = (frameRxNum == 0) ? C_RR0 : C_RR1;
                unsigned char rrFrame[5] = {FLAG, A_TX, rrCtrl, (unsigned char)(A_TX ^ rrCtrl), FLAG};
                writeBytesSerialPort(rrFrame, 5);
            }
        }
    }

    return -1;
}

int llCloseTx()
{
    unsigned char discFrame[5] = {FLAG, A_TX, C_DISC, (unsigned char)(A_TX ^ C_DISC), FLAG};

    alarmCount = 0;
    alarmEnabled = FALSE;

    while (alarmCount <= connectionParameters.nRetransmissions)
    {
        writeBytesSerialPort(discFrame, 5);

        alarmEnabled = TRUE;
        alarm(connectionParameters.timeout);

        while (alarmEnabled)
        {
            unsigned char addr, ctrl;
            int res = readFrame(&addr, &ctrl, NULL, NULL);
            if (res == 0 && ctrl == C_DISC)
            {
                alarm(0);
                alarmEnabled = FALSE;

                unsigned char uaFrame[5] = {FLAG, A_RX, C_UA, (unsigned char)(A_RX ^ C_UA), FLAG};
                writeBytesSerialPort(uaFrame, 5);

                closeSerialPort();

                printf("\n=========================================\n");
                printf("       DATA LINK LAYER STATISTICS (TX)    \n");
                printf("=========================================\n");
                printf("  Total I Frames Transmitted : %d\n", stats.iFramesSent);
                printf("  Total Retransmissions      : %d\n", stats.retransmissions);
                printf("  Total Timeouts             : %d\n", stats.timeouts);
                printf("  Total REJ Received         : %d\n", stats.rejReceived);
                printf("=========================================\n\n");

                return 0;
            }
        }
    }

    alarm(0);
    closeSerialPort();
    return -1;
}

int llCloseRx()
{
    unsigned char addr, ctrl;

    while (1)
    {
        int res = readFrame(&addr, &ctrl, NULL, NULL);
        if (res == 0 && ctrl == C_DISC)
        {
            break;
        }
    }

    unsigned char discFrame[5] = {FLAG, A_RX, C_DISC, (unsigned char)(A_RX ^ C_DISC), FLAG};
    writeBytesSerialPort(discFrame, 5);

    while (1)
    {
        int res = readFrame(&addr, &ctrl, NULL, NULL);
        if (res == 0 && ctrl == C_UA)
        {
            break;
        }
    }

    closeSerialPort();

    printf("\n=========================================\n");
    printf("       DATA LINK LAYER STATISTICS (RX)    \n");
    printf("=========================================\n");
    printf("  Total I Frames Received    : %d\n", stats.iFramesReceived);
    printf("  Total BCC2 Errors Detected : %d\n", stats.bcc2Errors);
    printf("  Total REJ Sent             : %d\n", stats.rejSent);
    printf("=========================================\n\n");

    return 0;
}