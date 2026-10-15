// RCOM 2026/2027
//
// Application layer protocol implementation with Efficiency Metrics

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

static double getTimeInSeconds()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer llParameters = {
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout,
    };
    strncpy(llParameters.serialPort, serialPort, sizeof(llParameters.serialPort) - 1);

    if (strcmp(role, "tx") == 0)
    {
        double startTime = getTimeInSeconds();

        if (llOpenTx(llParameters) < 0)
        {
            printf("Error opening Tx connection\n");
            return;
        }

        FILE *file = fopen(filename, "rb");
        if (!file)
        {
            perror("fopen tx");
            return;
        }

        struct stat st;
        if (stat(filename, &st) < 0)
        {
            perror("stat");
            fclose(file);
            return;
        }

        off_t fileSize = st.st_size;

        // START control packet
        unsigned char startPacket[MAX_PAYLOAD_SIZE];
        int idx = 0;

        startPacket[idx++] = 1; // C = 1 (START)

        // TLV 0: File Size
        startPacket[idx++] = 0;
        int sizeLen = sizeof(fileSize);
        startPacket[idx++] = sizeLen;
        memcpy(&startPacket[idx], &fileSize, sizeLen);
        idx += sizeLen;

        // TLV 1: File Name
        startPacket[idx++] = 1;
        int nameLen = strlen(filename);
        startPacket[idx++] = nameLen;
        memcpy(&startPacket[idx], filename, nameLen);
        idx += nameLen;

        if (llSend(startPacket, idx) < 0)
        {
            printf("Error sending START packet\n");
            fclose(file);
            return;
        }

        // Send Data packets
        unsigned char buffer[MAX_PAYLOAD_SIZE - 4];
        int bytesRead;

        while ((bytesRead = fread(buffer, 1, sizeof(buffer), file)) > 0)
        {
            unsigned char dataPacket[MAX_PAYLOAD_SIZE];
            dataPacket[0] = 2; // C = 2 (DATA)
            dataPacket[1] = (bytesRead >> 8) & 0xFF;
            dataPacket[2] = bytesRead & 0xFF;
            memcpy(&dataPacket[3], buffer, bytesRead);

            if (llSend(dataPacket, 3 + bytesRead) < 0)
            {
                printf("Error sending data packet\n");
                fclose(file);
                return;
            }
        }

        fclose(file);

        // END control packet
        startPacket[0] = 3; // C = 3 (END)
        if (llSend(startPacket, idx) < 0)
        {
            printf("Error sending END packet\n");
            return;
        }

        if (llCloseTx() < 0)
        {
            printf("Error closing Tx connection\n");
            return;
        }

        double endTime = getTimeInSeconds();
        double elapsedTime = endTime - startTime;
        double bitrate = (fileSize * 8.0) / elapsedTime;
        double efficiency = bitrate / baudRate;

        printf("=========================================\n");
        printf("         APPLICATION PERFORMANCE (TX)     \n");
        printf("=========================================\n");
        printf("  File Size          : %ld bytes\n", (long)fileSize);
        printf("  Transfer Time      : %.3f seconds\n", elapsedTime);
        printf("  Throughput (R)     : %.2f bps\n", bitrate);
        printf("  Capacity (C)       : %d bps\n", baudRate);
        printf("  Efficiency (S=R/C) : %.4f (%.2f%%)\n", efficiency, efficiency * 100.0);
        printf("=========================================\n\n");
    }
    else if (strcmp(role, "rx") == 0)
    {
        double startTime = getTimeInSeconds();

        if (llOpenRx(llParameters) < 0)
        {
            printf("Error opening Rx connection\n");
            return;
        }

        FILE *file = NULL;
        int done = 0;
        long totalBytesReceived = 0;

        while (!done)
        {
            unsigned char packet[MAX_PAYLOAD_SIZE * 2];
            int packetSize = llReceive(packet);

            if (packetSize < 0)
            {
                printf("Error in llReceive\n");
                break;
            }

            if (packetSize == 0)
                continue;

            unsigned char controlByte = packet[0];

            if (controlByte == 1) // START
            {
                char rxFilename[256] = {0};
                int pIdx = 1;

                while (pIdx < packetSize)
                {
                    unsigned char type = packet[pIdx++];
                    unsigned char len = packet[pIdx++];

                    if (type == 1) // File Name
                    {
                        memcpy(rxFilename, &packet[pIdx], len);
                        rxFilename[len] = '\0';
                    }
                    pIdx += len;
                }

                file = fopen(filename, "wb");
                if (!file)
                {
                    perror("fopen rx");
                    return;
                }
            }
            else if (controlByte == 2) // DATA
            {
                int dataLen = 256 * packet[1] + packet[2];
                if (file)
                {
                    fwrite(&packet[3], 1, dataLen, file);
                    totalBytesReceived += dataLen;
                }
            }
            else if (controlByte == 3) // END
            {
                if (file)
                {
                    fclose(file);
                    file = NULL;
                }
                done = 1;
            }
        }

        if (llCloseRx() < 0)
        {
            printf("Error closing Rx connection\n");
            return;
        }

        double endTime = getTimeInSeconds();
        double elapsedTime = endTime - startTime;
        double bitrate = (totalBytesReceived * 8.0) / elapsedTime;
        double efficiency = bitrate / baudRate;

        printf("=========================================\n");
        printf("         APPLICATION PERFORMANCE (RX)     \n");
        printf("=========================================\n");
        printf("  Received Size      : %ld bytes\n", totalBytesReceived);
        printf("  Transfer Time      : %.3f seconds\n", elapsedTime);
        printf("  Throughput (R)     : %.2f bps\n", bitrate);
        printf("  Efficiency (S=R/C) : %.4f (%.2f%%)\n", efficiency, efficiency * 100.0);
        printf("=========================================\n\n");
    }
}