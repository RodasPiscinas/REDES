// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters)
{
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    // SET frame
    unsigned char set[5];

    set[0] = 0x7E;
    set[1] = 0x03;
    set[2] = 0x03;
    set[3] = set[1] ^ set[2];
    set[4] = 0x7E;

    printf("Sending SET: ");
    for (int i = 0; i < 5; i++)
    {
        printf("0x%02X ", set[i]);
    }
    printf("\n");

    int bytes = writeBytesSerialPort(set, 5);

    if (bytes < 0)
    {
        perror("writeBytesSerialPort");
        return -1;
    }

    printf("%d bytes written to serial port\n", bytes);

    // Receive UA
    unsigned char ua[5];

    for (int i = 0; i < 5; i++)
    {
        if (readByteSerialPort(&ua[i]) < 0)
        {
            perror("readByteSerialPort");
            return -1;
        }

        printf("Received byte: 0x%02X\n", ua[i]);
    }

    // Check UA frame
    if (ua[0] == 0x7E &&
        ua[1] == 0x01 &&
        ua[2] == 0x07 &&
        ua[3] == (ua[1] ^ ua[2]) &&
        ua[4] == 0x7E)
    {
        printf("UA frame received correctly!\n");
        return 0;
    }

    printf("Invalid UA frame received.\n");
    return -1;
}


int llOpenRx(LinkLayer llParameters)
{
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    // Receive SET
    unsigned char set[5];

    for (int i = 0; i < 5; i++)
    {
        if (readByteSerialPort(&set[i]) < 0)
        {
            perror("readByteSerialPort");
            return -1;
        }

        printf("Received byte: 0x%02X\n", set[i]);
    }

    // Check SET frame
    if (set[0] != 0x7E ||
        set[1] != 0x03 ||
        set[2] != 0x03 ||
        set[3] != (set[1] ^ set[2]) ||
        set[4] != 0x7E)
    {
        printf("Invalid SET frame received.\n");
        return -1;
    }

    printf("SET frame received correctly!\n");

    // Build UA
    unsigned char ua[5];

    ua[0] = 0x7E;
    ua[1] = 0x01;
    ua[2] = 0x07;
    ua[3] = ua[1] ^ ua[2];
    ua[4] = 0x7E;

    printf("Sending UA: ");

    for (int i = 0; i < 5; i++)
    {
        printf("0x%02X ", ua[i]);
    }

    printf("\n");

    int bytes = writeBytesSerialPort(ua, 5);

    if (bytes < 0)
    {
        perror("writeBytesSerialPort");
        return -1;
    }

    printf("%d bytes written to serial port\n", bytes);

    return 0;
}


////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
int llSend(const unsigned char *buf, int bufSize)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llCloseTx()
{
    // TODO: Implement this function

    return 0;
}

int llCloseRx()
{
    // TODO: Implement this function

    return 0;
}

