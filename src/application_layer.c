// RCOM 2026/2027
//
// Application layer protocol implementation

// This layer implements the file-transfer protocol only. It creates/parses
// START, DATA and END packets and accesses the file system. It deliberately
// does not know anything about I frames, RR/REJ, BCC, stuffing, sequence
// numbers or retransmissions; those mechanisms belong exclusively to the
// Data Link Layer.
//
// Packet formats used by this implementation:
//   START/END -> C | T | L | V | ... (TLV parameters)
//   DATA      -> C | L2 | L1 | P1 ... Pk

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <string.h>

// Application packet control fields
#define C_START 0x01
#define C_DATA  0x02
#define C_END   0x03

// TLV types
#define T_FILE_SIZE 0x00
#define T_FILE_NAME 0x01

// Maximum application packet size and file chunk size
#define APP_PACKET_MAX 255
#define FILE_CHUNK_SIZE 252


////////////////////////////////////////////////
// AUXILIARY FUNCTIONS
////////////////////////////////////////////////

// Helpers below only manipulate Application Layer information: file names,
// file sizes and TLV encoding/decoding. They never inspect link-layer frames.

// Get the file name without the directory path.
static const char *getBaseName(const char *path)
{
    const char *base = path;

    const char *slash = strrchr(path, '/');
    const char *backslash = strrchr(path, '\\');

    if (slash != NULL)
        base = slash + 1;

    if (backslash != NULL && backslash + 1 > base)
        base = backslash + 1;

    return base;
}


// Calculate how many bytes are needed to store the file size.
static int fileSizeLength(unsigned long long fileSize)
{
    int length = 1;

    while (fileSize > 0xFF)
    {
        length++;
        fileSize >>= 8;
    }

    return length;
}


// Build a START or END packet with the file information.
//
// Format:
//
// C | T1 | L1 | V1 | T2 | L2 | V2
//
// T = 0 -> file size
// T = 1 -> file name

static int buildControlPacket(unsigned char control,
                              unsigned long long fileSize,
                              const char *filename,
                              unsigned char *packet)
{
    int index = 0;

    packet[index++] = control;

    ////////////////////////////////////////////////
    // FILE SIZE TLV
    ////////////////////////////////////////////////

    int sizeLength = fileSizeLength(fileSize);

    packet[index++] = T_FILE_SIZE;
    packet[index++] = (unsigned char) sizeLength;

    // Store the file size from the most significant byte.
    for (int i = sizeLength - 1; i >= 0; i--)
    {
        packet[index++] =
            (unsigned char) ((fileSize >> (8 * i)) & 0xFF);
    }

    ////////////////////////////////////////////////
    // FILE NAME TLV
    ////////////////////////////////////////////////

    const char *baseName = getBaseName(filename);

    int nameLength = strlen(baseName);

    if (nameLength > 255)
    {
        printf("File name is too long\n");
        return -1;
    }

    if (index + 2 + nameLength > APP_PACKET_MAX)
    {
        printf("Control packet is too large\n");
        return -1;
    }

    packet[index++] = T_FILE_NAME;
    packet[index++] = (unsigned char) nameLength;

    memcpy(&packet[index], baseName, nameLength);

    index += nameLength;

    return index;
}


// Read the file information from a START or END packet.
static int parseControlPacket(const unsigned char *packet,
                              int packetSize,
                              unsigned char expectedControl,
                              unsigned long long *fileSize,
                              char *fileName,
                              int fileNameCapacity)
{
    if (packetSize < 1)
        return -1;

    if (packet[0] != expectedControl)
        return -1;

    int index = 1;
    int foundFileSize = 0;

    *fileSize = 0;

    if (fileNameCapacity > 0)
        fileName[0] = '\0';

    while (index < packetSize)
    {
        // Check that the type and length fields are present.
        if (index + 2 > packetSize)
            return -1;

        unsigned char type = packet[index++];
        unsigned char length = packet[index++];

        if (index + length > packetSize)
            return -1;

        ////////////////////////////////////////////////
        // FILE SIZE
        ////////////////////////////////////////////////

        if (type == T_FILE_SIZE)
        {
            if (length == 0 ||
                length > sizeof(unsigned long long))
            {
                return -1;
            }

            unsigned long long size = 0;

            // Rebuild the file size from its bytes.
            for (int i = 0; i < length; i++)
            {
                size =
                    (size << 8) |
                    packet[index + i];
            }

            *fileSize = size;
            foundFileSize = 1;
        }

        ////////////////////////////////////////////////
        // FILE NAME
        ////////////////////////////////////////////////

        else if (type == T_FILE_NAME)
        {
            if (fileNameCapacity > 0)
            {
                int copyLength = length;

                if (copyLength >= fileNameCapacity)
                    copyLength = fileNameCapacity - 1;

                memcpy(fileName,
                       &packet[index],
                       copyLength);

                fileName[copyLength] = '\0';
            }
        }

        index += length;
    }

    // The file size must be present in the packet.
    if (!foundFileSize)
        return -1;

    return 0;
}


////////////////////////////////////////////////
// APPLICATION LAYER
////////////////////////////////////////////////

// TX: open link -> START -> DATA packets -> END -> close link.
// RX: open link -> receive START -> DATA packets -> END -> validate -> close.

void applicationLayer(const char *serialPort,
                      const char *role,
                      int baudRate,
                      int nTries,
                      int timeout,
                      const char *filename)
{
    // Set the parameters used by the link layer.
    LinkLayer llParameters = {
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout,
    };

    strcpy(llParameters.serialPort, serialPort);


    ////////////////////////////////////////////////////////////
    // TRANSMITTER
    ////////////////////////////////////////////////////////////

    if (strcmp(role, "tx") == 0)
    {
        ////////////////////////////////////////////////
        // Open data link connection
        ////////////////////////////////////////////////

        if (llOpenTx(llParameters) < 0)
        {
            printf("Error opening link layer\n");
            return;
        }

        ////////////////////////////////////////////////
        // Open input file
        ////////////////////////////////////////////////

        FILE *file = fopen(filename, "rb");

        if (file == NULL)
        {
            perror("fopen");

            llCloseTx();

            return;
        }

        ////////////////////////////////////////////////
        // Obtain file size
        ////////////////////////////////////////////////

        // Move to the end of the file to get its size.
        if (fseek(file, 0, SEEK_END) != 0)
        {
            perror("fseek");

            fclose(file);
            llCloseTx();

            return;
        }

        long fileSizeLong = ftell(file);

        if (fileSizeLong < 0)
        {
            perror("ftell");

            fclose(file);
            llCloseTx();

            return;
        }

        unsigned long long fileSize =
            (unsigned long long) fileSizeLong;

        rewind(file);

        printf("Sending file: %s\n", filename);
        printf("File size: %llu bytes\n", fileSize);


        ////////////////////////////////////////////////
        // Send START packet
        ////////////////////////////////////////////////

        // Send the file name and size before the file data.
        unsigned char controlPacket[APP_PACKET_MAX];

        int controlPacketSize =
            buildControlPacket(C_START,
                               fileSize,
                               filename,
                               controlPacket);

        if (controlPacketSize < 0)
        {
            fclose(file);
            llCloseTx();

            return;
        }

        if (llSend(controlPacket,
                   controlPacketSize) < 0)
        {
            printf("Error sending START packet\n");

            fclose(file);
            llCloseTx();

            return;
        }

        ////////////////////////////////////////////////
        // Send DATA packets
        ////////////////////////////////////////////////

        unsigned char fileData[FILE_CHUNK_SIZE];

        // Read the file and send it in chunks.
        while (1)
        {
            size_t bytesRead =
                fread(fileData,
                      1,
                      FILE_CHUNK_SIZE,
                      file);

            if (bytesRead > 0)
            {
                unsigned char dataPacket[APP_PACKET_MAX];

                int dataSize = (int) bytesRead;

                // C = DATA
                dataPacket[0] = C_DATA;

                // K = 256 * L2 + L1
                dataPacket[1] =
                    (unsigned char) (dataSize / 256);

                dataPacket[2] =
                    (unsigned char) (dataSize % 256);

                // Add the file data after the header.
                memcpy(&dataPacket[3],
                       fileData,
                       dataSize);

                int packetSize =
                    dataSize + 3;

                if (llSend(dataPacket,
                           packetSize) < 0)
                {
                    printf("Error sending DATA packet\n");

                    fclose(file);
                    llCloseTx();

                    return;
                }
            }

            if (bytesRead < FILE_CHUNK_SIZE)
            {
                if (ferror(file))
                {
                    perror("fread");

                    fclose(file);
                    llCloseTx();

                    return;
                }

                // End of file
                break;
            }
        }


        ////////////////////////////////////////////////
        // Send END packet
        ////////////////////////////////////////////////

        // Send the file information again to mark the end.
        controlPacketSize =
            buildControlPacket(C_END,
                               fileSize,
                               filename,
                               controlPacket);

        if (controlPacketSize < 0)
        {
            fclose(file);
            llCloseTx();

            return;
        }

        if (llSend(controlPacket,
                   controlPacketSize) < 0)
        {
            printf("Error sending END packet\n");

            fclose(file);
            llCloseTx();

            return;
        }

        ////////////////////////////////////////////////
        // Finish
        ////////////////////////////////////////////////

        fclose(file);

        if (llCloseTx() < 0)
        {
            printf("Error closing link layer\n");
            return;
        }

        printf("File sent successfully\n");
    }


    ////////////////////////////////////////////////////////////
    // RECEIVER
    ////////////////////////////////////////////////////////////

    else if (strcmp(role, "rx") == 0)
    {
        ////////////////////////////////////////////////
        // Open data link connection
        ////////////////////////////////////////////////

        if (llOpenRx(llParameters) < 0)
        {
            printf("Error opening link layer\n");
            return;
        }


        ////////////////////////////////////////////////
        // Receive START packet
        ////////////////////////////////////////////////

        unsigned char packet[APP_PACKET_MAX + 1];

        // Get the file information from the transmitter.
        int packetSize =
            llReceive(packet);

        if (packetSize < 0)
        {
            printf("Error receiving START packet\n");
            return;
        }

        unsigned long long expectedFileSize;

        char receivedFileName[256];

        if (parseControlPacket(packet,
                               packetSize,
                               C_START,
                               &expectedFileSize,
                               receivedFileName,
                               sizeof(receivedFileName)) < 0)
        {
            printf("Invalid START packet\n");
            return;
        }

        printf("Original file: %s\n",
               receivedFileName);
        printf("Expected file size: %llu bytes\n",
               expectedFileSize);


        ////////////////////////////////////////////////
        // Create output file
        ////////////////////////////////////////////////

        // Create the file where the received data will be saved.
        FILE *file =
            fopen(filename, "wb");

        if (file == NULL)
        {
            perror("fopen");
            return;
        }

        unsigned long long bytesReceived = 0;

        int endReceived = 0;
        int transferValid = 1;

        char endFileName[256];
        unsigned long long endFileSize = 0;


        ////////////////////////////////////////////////
        // Receive DATA packets until END
        ////////////////////////////////////////////////

        while (!endReceived)
        {
            packetSize =
                llReceive(packet);

            if (packetSize < 0)
            {
                printf("Error receiving packet\n");

                transferValid = 0;
                break;
            }

            ////////////////////////////////////////////
            // DATA PACKET
            ////////////////////////////////////////////

            if (packet[0] == C_DATA)
            {
                if (packetSize < 3)
                {
                    printf("Invalid DATA packet\n");

                    transferValid = 0;
                    break;
                }

                // Calculate the number of data bytes in the packet.
                int dataSize =
                    256 * packet[1] +
                    packet[2];

                // Packet must contain:
                // C + L2 + L1 + K bytes
                if (packetSize != dataSize + 3)
                {
                    printf("Invalid DATA packet size\n");

                    transferValid = 0;
                    break;
                }

                // Write the received data to the output file.
                size_t written =
                    fwrite(&packet[3],
                           1,
                           dataSize,
                           file);

                if (written != (size_t) dataSize)
                {
                    perror("fwrite");

                    transferValid = 0;
                    break;
                }

                bytesReceived += dataSize;
            }


            ////////////////////////////////////////////
            // END PACKET
            ////////////////////////////////////////////

            else if (packet[0] == C_END)
            {
                // Read the final file information.
                if (parseControlPacket(packet,
                                       packetSize,
                                       C_END,
                                       &endFileSize,
                                       endFileName,
                                       sizeof(endFileName)) < 0)
                {
                    printf("Invalid END packet\n");

                    transferValid = 0;
                    break;
                }

                endReceived = 1;
            }


            ////////////////////////////////////////////
            // UNEXPECTED PACKET
            ////////////////////////////////////////////

            else
            {
                printf("Unexpected application packet\n");

                transferValid = 0;
                break;
            }
        }


        ////////////////////////////////////////////////
        // Validate transfer
        ////////////////////////////////////////////////

        fclose(file);

        if (transferValid)
        {
            // Check that the START and END information matches.
            if (endFileSize != expectedFileSize)
            {
                printf("START/END file size mismatch\n");
                transferValid = 0;
            }

            if (strcmp(endFileName,
                       receivedFileName) != 0)
            {
                printf("START/END file name mismatch\n");
                transferValid = 0;
            }

            // Check that all the expected bytes were received.
            if (bytesReceived != expectedFileSize)
            {
                printf("Received file size is incorrect\n");
                printf("Expected: %llu bytes\n",
                       expectedFileSize);
                printf("Received: %llu bytes\n",
                       bytesReceived);

                transferValid = 0;
            }
        }


        ////////////////////////////////////////////////
        // Close data link connection
        ////////////////////////////////////////////////

        if (llCloseRx() < 0)
        {
            printf("Error closing link layer\n");
            return;
        }


        ////////////////////////////////////////////////
        // Final result
        ////////////////////////////////////////////////

        if (!transferValid)
        {
            printf("File transfer failed\n");

            // Remove the output file if the transfer failed.
            remove(filename);

            return;
        }

        printf("File received successfully\n");
        printf("Saved as: %s\n", filename);
    }


    ////////////////////////////////////////////////////////////
    // INVALID ROLE
    ////////////////////////////////////////////////////////////

    else
    {
        printf("Invalid role: %s. Must be 'tx' or 'rx'.\n",
               role);

        return;
    }
}