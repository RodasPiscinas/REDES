// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256

// Alarm variables
volatile sig_atomic_t alarmEnabled = FALSE;
volatile sig_atomic_t alarmCount = 0;

// Called automatically when the alarm expires
void alarmHandler(int signal)
{
    (void) signal;

    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d received\n", (int) alarmCount);
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters)
{
    // Open serial port
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    //////////////////////////////////////////////////
    // Configure SIGALRM
    //////////////////////////////////////////////////

    struct sigaction act = {0};

    act.sa_handler = &alarmHandler;
    sigemptyset(&act.sa_mask);
    act.sa_flags = 0;

    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        return -1;
    }

    //////////////////////////////////////////////////
    // Build SET frame
    //////////////////////////////////////////////////

    unsigned char set[5];

    set[0] = 0x7E;
    set[1] = 0x03;
    set[2] = 0x03;
    set[3] = set[1] ^ set[2];
    set[4] = 0x7E;

    //////////////////////////////////////////////////
    // Retransmission mechanism
    //////////////////////////////////////////////////

    alarmCount = 0;
    alarmEnabled = FALSE;

    /*
     * nRetransmissions is the number of RETRIES.
     *
     * Therefore, if nRetransmissions = 3:
     *
     * initial transmission
     * + retry 1
     * + retry 2
     * + retry 3
     *
     * = maximum of 4 transmissions
     */
    while (alarmCount <= llParameters.nRetransmissions)
    {
        //////////////////////////////////////////////////
        // Send SET
        //////////////////////////////////////////////////

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
            alarm(0);
            return -1;
        }

        if (bytes != 5)
        {
            printf("Error: only %d of 5 SET bytes were written.\n", bytes);
            alarm(0);
            return -1;
        }

        printf("%d bytes written to serial port\n", bytes);

        //////////////////////////////////////////////////
        // Start timeout
        //////////////////////////////////////////////////

        alarmEnabled = TRUE;

        alarm(llParameters.timeout);

        printf(
            "Waiting for UA (timeout = %d seconds)...\n",
            llParameters.timeout
        );

        //////////////////////////////////////////////////
        // Wait for UA
        //////////////////////////////////////////////////

        unsigned char ua[5];
        int uaIndex = 0;

        /*
         * Remain here until:
         *
         * 1. a valid UA is received
         * OR
         * 2. SIGALRM occurs
         */
        while (alarmEnabled)
        {
            unsigned char byte;

            int result = readByteSerialPort(&byte);

            if (result > 0)
            {
                printf("Received byte: 0x%02X\n", byte);

                ua[uaIndex] = byte;
                uaIndex++;

                // We have collected 5 bytes
                if (uaIndex == 5)
                {
                    //////////////////////////////////////////////////
                    // Check UA frame
                    //////////////////////////////////////////////////

                    if (ua[0] == 0x7E &&
                        ua[1] == 0x03 &&
                        ua[2] == 0x07 &&
                        ua[3] == (ua[1] ^ ua[2]) &&
                        ua[4] == 0x7E)
                    {
                        // UA arrived successfully, so cancel timeout
                        alarm(0);

                        alarmEnabled = FALSE;

                        printf("UA frame received correctly!\n");

                        return 0;
                    }

                    /*
                     * The 5 bytes were not a valid UA.
                     *
                     * For now, reset and keep waiting until timeout.
                     *
                     * Later, the state machine will replace this
                     * simplistic way of receiving frames.
                     */
                    printf("Invalid UA frame received.\n");

                    uaIndex = 0;
                }
            }
            else if (result < 0)
            {
                /*
                 * If SIGALRM interrupted read(), errno will normally
                 * be EINTR. This is expected, not a real error.
                 */
                if (errno == EINTR)
                {
                    continue;
                }

                perror("readByteSerialPort");

                alarm(0);

                return -1;
            }
        }

        //////////////////////////////////////////////////
        // We only reach here if the alarm expired
        //////////////////////////////////////////////////

        if (alarmCount <= llParameters.nRetransmissions)
        {
            printf("Timeout. Retransmitting SET...\n");
        }
    }

    //////////////////////////////////////////////////
    // Maximum number of retransmissions reached
    //////////////////////////////////////////////////

    alarm(0);

    printf("Maximum number of retransmissions reached.\n");
    printf("Connection could not be established.\n");

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

    //////////////////////////////////////////////////
    // Receive SET
    //
    // NOTE:
    // This is still your First Class implementation.
    // The state machine from the Second Class will
    // replace this part next.
    //////////////////////////////////////////////////

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

    //////////////////////////////////////////////////
    // Check SET
    //////////////////////////////////////////////////

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

    //////////////////////////////////////////////////
    // Build UA
    //////////////////////////////////////////////////

    unsigned char ua[5];

    ua[0] = 0x7E;
    ua[1] = 0x03;
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