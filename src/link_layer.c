// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include "state_machine.h"

#include <stdio.h>
#include <unistd.h>
#include <signal.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256

#define FLAG 0x7E
#define A_TX 0x03
#define A_RX 0x01
#define C_SET 0x03
#define C_UA 0x07

#define C_DISC 0x0B

#define C_I0 0x00
#define C_I1 0x80

#define C_RR0 0xAA
#define C_RR1 0xAB

#define C_REJ0 0x54
#define C_REJ1 0x55

#define ESCAPE 0x7D
#define ESC_FLAG 0x5E
#define ESC_ESCAPE 0x5D

int alarmEnabled = FALSE;

static int txSequenceNumber = 0;
static int expectedNs = 0;

static int protocolTimeout = 3;
static int protocolRetransmissions = 3;

void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    printf("Timeout. Retrying...\n");
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters)
{
    // Save the connection parameters.
    protocolTimeout = llParameters.timeout;
    protocolRetransmissions = llParameters.nRetransmissions;
    txSequenceNumber = 0;

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    // Set the alarm handler.
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;

    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        return -1;
    }

    // Build the 5-byte SET frame.
    unsigned char SET[5];

    SET[0] = 0x7E; // Start flag
    SET[1] = 0x03; // Transmitter address
    SET[2] = 0x03; // Control field: SET
    SET[3] = SET[1] ^ SET[2]; // BCC1: XOR of A and C
    SET[4] = 0x7E; // End flag

    int attempts = 0;
    int connected = FALSE;
    int state;

    // Send SET and wait for UA, retrying on timeout.
    while (attempts < protocolRetransmissions && connected == FALSE)
    {
        attempts++;

        // Send the SET frame to the receiver.
        writeBytesSerialPort(SET, 5);

        // Start the timeout alarm.
        alarmEnabled = TRUE;
        alarm(protocolTimeout);

        // Wait for UA.
        state = 0;

        while (alarmEnabled && state != 5)
        {
            unsigned char byte;
            int bytesRead = readByteSerialPort(&byte);

            if (bytesRead != 1)
                continue;

            switch (state)
            {
                case 0:
                    if (byte == FLAG)
                        state = 1;
                    break;

                case 1:
                    if (byte == A_TX)
                        state = 2;
                    else if (byte != FLAG)
                        state = 0;
                    break;

                case 2:
                    if (byte == C_UA)
                        state = 3;
                    else if (byte == FLAG)
                        state = 1;
                    else
                        state = 0;
                    break;

                case 3:
                    if (byte == (A_TX ^ C_UA))
                        state = 4;
                    else if (byte == FLAG)
                        state = 1;
                    else
                        state = 0;
                    break;

                case 4:
                    if (byte == FLAG)
                        state = 5;
                    else
                        state = 0;
                    break;
            }
        }

        if (state == 5)
        {
            connected = TRUE;
            alarm(0);
            alarmEnabled = FALSE;
            printf("UA received correctly\n");
        }

    }

    // Check if UA was received after all attempts.
    if (connected == FALSE)
    {
        printf("Failed to receive UA after 3 attempts\n");
        closeSerialPort();
        return -1;
    }

    return 0;
}

int llOpenRx(LinkLayer llParameters)
{
    protocolTimeout = llParameters.timeout;
    protocolRetransmissions = llParameters.nRetransmissions;
    expectedNs = 0;

    // Open serial port.
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    // Wait for the transmitter to start the connection.
    StateMachine sm;
    stateMachineInit(&sm, A_TX, C_SET);

    int nBytesBuf = 0;

    // Wait until a valid SET frame is received.
    while (sm.state != STOP)
    {
        unsigned char byte;

        int bytes = readByteSerialPort(&byte);

        if (bytes < 0)
        {
            perror("readByteSerialPort");
            return -1;
        }

        if (bytes == 0)
        {
            continue;
        }

        nBytesBuf += bytes;

        // Let the state machine process the received byte.
        stateMachineProcessByte(&sm, byte);
    }

    printf("SET received correctly\n");

    // Build the UA frame.
    unsigned char uaFrame[5];

    uaFrame[0] = FLAG;

    uaFrame[1] = A_TX;
    uaFrame[2] = C_UA;
    uaFrame[3] = A_TX ^ C_UA;

    uaFrame[4] = FLAG;

    // Confirm the connection by sending UA.
    int bytesWritten = writeBytesSerialPort(uaFrame, 5);

    if (bytesWritten < 0)
    {
        perror("writeBytesSerialPort");
        return -1;
    }

    printf("UA sent\n");

    return 0;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
int llSend(const unsigned char *buf, int bufSize)
{
    // Calculate the XOR of all packet bytes.
    unsigned char bcc2 = 0x00;

    for (int i = 0; i < bufSize; i++)
    {
        bcc2 ^= buf[i];
    }

    // Choose I0 or I1 according to the current sequence number.
    unsigned char control;

    if (txSequenceNumber == 0)
        control = C_I0;
    else
        control = C_I1;

    // Worst case: every data byte and BCC2 need stuffing.
    unsigned char frame[2 * bufSize + 7];
    int frameSize = 0;

    // I frame header.
    frame[frameSize++] = FLAG;
    frame[frameSize++] = A_TX;
    frame[frameSize++] = control;
    frame[frameSize++] = A_TX ^ control;

    // Escape special bytes so they are not confused with FLAG.
    for (int i = 0; i < bufSize; i++)
    {
        if (buf[i] == FLAG)
        {
            frame[frameSize++] = ESCAPE;
            frame[frameSize++] = ESC_FLAG;
        }
        else if (buf[i] == ESCAPE)
        {
            frame[frameSize++] = ESCAPE;
            frame[frameSize++] = ESC_ESCAPE;
        }
        else
        {
            frame[frameSize++] = buf[i];
        }
    }

    // BCC2 must also be stuffed if necessary.
    if (bcc2 == FLAG)
    {
        frame[frameSize++] = ESCAPE;
        frame[frameSize++] = ESC_FLAG;
    }
    else if (bcc2 == ESCAPE)
    {
        frame[frameSize++] = ESCAPE;
        frame[frameSize++] = ESC_ESCAPE;
    }
    else
    {
        frame[frameSize++] = bcc2;
    }

    frame[frameSize++] = FLAG;

    unsigned char expectedRR =
        (txSequenceNumber == 0) ? C_RR1 : C_RR0;

    unsigned char expectedREJ =
        (txSequenceNumber == 0) ? C_REJ0 : C_REJ1;

    int attempts = 0;

    // Wait for acknowledgement and retry if needed.
    while (attempts < protocolRetransmissions)
    {
        attempts++;

        int bytesWritten =
            writeBytesSerialPort(frame, frameSize);

        if (bytesWritten < 0)
        {
            perror("writeBytesSerialPort");
            return -1;
        }

        alarmEnabled = TRUE;
        alarm(protocolTimeout);

        int state = 0;
        unsigned char responseControl = 0;

        // Receive RR or REJ byte by byte.
        while (alarmEnabled && state != 5)
        {
            unsigned char byte;

            int bytesRead =
                readByteSerialPort(&byte);

            if (bytesRead < 0)
            {
                // read may be interrupted by SIGALRM.
                if (!alarmEnabled)
                    break;

                perror("readByteSerialPort");
                alarm(0);
                alarmEnabled = FALSE;
                return -1;
            }

            if (bytesRead == 0)
                continue;

            switch (state)
            {
                case 0:
                    if (byte == FLAG)
                        state = 1;
                    break;

                case 1:
                    if (byte == A_TX)
                        state = 2;
                    else if (byte != FLAG)
                        state = 0;
                    break;

                case 2:
                    if (byte == expectedRR ||
                        byte == expectedREJ)
                    {
                        responseControl = byte;
                        state = 3;
                    }
                    else if (byte == FLAG)
                    {
                        state = 1;
                    }
                    else
                    {
                        state = 0;
                    }

                    break;

                case 3:
                    if (byte ==
                        (A_TX ^ responseControl))
                    {
                        state = 4;
                    }
                    else if (byte == FLAG)
                    {
                        state = 1;
                    }
                    else
                    {
                        state = 0;
                    }

                    break;

                case 4:
                    if (byte == FLAG)
                        state = 5;
                    else
                        state = 0;

                    break;
            }
        }

        alarm(0);
        alarmEnabled = FALSE;

        if (state == 5)
        {
            if (responseControl == expectedRR)
            {
                // Next I frame will use the other sequence number.
                txSequenceNumber =
                    1 - txSequenceNumber;

                return bufSize;
            }

            if (responseControl == expectedREJ)
            {
                printf("REJ received. Retransmitting...\n");
                continue;
            }
        }

        printf("No valid acknowledgement received\n");
    }

    printf("Failed to send I frame\n");

    return -1;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{
    while (1)
    {
        unsigned char byte;
        unsigned char control = 0;

        unsigned char stuffedData[2 * BUF_SIZE + 2];
        int stuffedSize = 0;

        int state = 0;

        // Receive one complete I frame.
        while (state != 5)
        {
            int bytesRead =
                readByteSerialPort(&byte);

            if (bytesRead < 0)
            {
                perror("readByteSerialPort");
                return -1;
            }

            if (bytesRead == 0)
                continue;

            switch (state)
            {
                // Waiting for initial FLAG.
                case 0:
                    if (byte == FLAG)
                        state = 1;

                    break;

                // Waiting for address.
                case 1:
                    if (byte == FLAG)
                    {
                        // Multiple FLAGs are allowed.
                        state = 1;
                    }
                    else if (byte == A_TX)
                    {
                        state = 2;
                    }
                    else
                    {
                        state = 0;
                    }

                    break;

                // Waiting for I0 or I1.
                case 2:
                    if (byte == C_I0 ||
                        byte == C_I1)
                    {
                        control = byte;
                        state = 3;
                    }
                    else if (byte == FLAG)
                    {
                        state = 1;
                    }
                    else
                    {
                        state = 0;
                    }

                    break;

                // Validate BCC1.
                case 3:
                    if (byte == (A_TX ^ control))
                    {
                        stuffedSize = 0;
                        state = 4;
                    }
                    else if (byte == FLAG)
                    {
                        state = 1;
                    }
                    else
                    {
                        // Wrong BCC1: ignore the frame.
                        state = 0;
                    }

                    break;

                // Receive DATA + BCC2.
                case 4:
                    if (byte == FLAG)
                    {
                        state = 5;
                    }
                    else
                    {
                        if (stuffedSize >=
                            2 * BUF_SIZE + 2)
                        {
                            printf("Frame too large\n");
                            return -1;
                        }

                        stuffedData[stuffedSize++] =
                            byte;
                    }

                    break;
            }
        }

        int receivedNs =
            (control == C_I0) ? 0 : 1;

        int isNewFrame =
            (receivedNs == expectedNs);

        // Restore the original data bytes.
        unsigned char data[BUF_SIZE + 1];
        int dataSize = 0;
        int destuffError = FALSE;

        for (int i = 0; i < stuffedSize; i++)
        {
            if (stuffedData[i] == ESCAPE)
            {
                if (i + 1 >= stuffedSize)
                {
                    destuffError = TRUE;
                    break;
                }

                i++;

                if (stuffedData[i] == ESC_FLAG)
                {
                    data[dataSize++] = FLAG;
                }
                else if (stuffedData[i] == ESC_ESCAPE)
                {
                    data[dataSize++] = ESCAPE;
                }
                else
                {
                    destuffError = TRUE;
                    break;
                }
            }
            else
            {
                data[dataSize++] =
                    stuffedData[i];
            }

            if (dataSize > BUF_SIZE)
            {
                printf("Frame too large after destuffing\n");
                return -1;
            }
        }

        // Invalid data field.
        if (destuffError || dataSize < 1)
        {
            unsigned char responseControl;

            if (isNewFrame)
            {
                responseControl =
                    (expectedNs == 0)
                        ? C_REJ0
                        : C_REJ1;
            }
            else
            {
                responseControl =
                    (expectedNs == 0)
                        ? C_RR0
                        : C_RR1;
            }

            unsigned char response[5] = {
                FLAG,
                A_TX,
                responseControl,
                A_TX ^ responseControl,
                FLAG
            };

            if (writeBytesSerialPort(response, 5) < 0)
            {
                perror("writeBytesSerialPort");
                return -1;
            }

            continue;
        }

        // The last byte after destuffing is BCC2.
        unsigned char receivedBCC2 =
            data[dataSize - 1];

        int packetSize =
            dataSize - 1;

        // Check if the received packet is correct.
        unsigned char calculatedBCC2 = 0x00;

        for (int i = 0; i < packetSize; i++)
        {
            calculatedBCC2 ^= data[i];
        }

        // Handle a BCC2 error.
        if (calculatedBCC2 != receivedBCC2)
        {
            unsigned char responseControl;
            if (isNewFrame)
            {
                // Ask for retransmission of the expected frame.
                responseControl =
                    (expectedNs == 0)
                        ? C_REJ0
                        : C_REJ1;

                printf("BCC2 error. Sending REJ%d\n",
                       expectedNs);
            }
            else
            {
                // Duplicate frame: send RR again.
                responseControl =
                    (expectedNs == 0)
                        ? C_RR0
                        : C_RR1;
            }

            unsigned char response[5] = {
                FLAG,
                A_TX,
                responseControl,
                A_TX ^ responseControl,
                FLAG
            };

            if (writeBytesSerialPort(response, 5) < 0)
            {
                perror("writeBytesSerialPort");
                return -1;
            }

            continue;
        }

        // Discard duplicate frames and acknowledge them again.
        if (!isNewFrame)
        {
            unsigned char rrControl =
                (expectedNs == 0)
                    ? C_RR0
                    : C_RR1;

            unsigned char rrFrame[5] = {
                FLAG,
                A_TX,
                rrControl,
                A_TX ^ rrControl,
                FLAG
            };

            if (writeBytesSerialPort(rrFrame, 5) < 0)
            {
                perror("writeBytesSerialPort");
                return -1;
            }

            printf("Duplicate I frame discarded\n");

            continue;
        }

        // Copy the valid packet to the Application Layer.
        for (int i = 0; i < packetSize; i++)
        {
            packet[i] = data[i];
        }

        // Expect the other sequence number next.
        expectedNs = 1 - expectedNs;

        unsigned char rrControl =
            (expectedNs == 0)
                ? C_RR0
                : C_RR1;

        unsigned char rrFrame[5] = {
            FLAG,
            A_TX,
            rrControl,
            A_TX ^ rrControl,
            FLAG
        };

        if (writeBytesSerialPort(rrFrame, 5) < 0)
        {
            perror("writeBytesSerialPort");
            return -1;
        }

        return packetSize;
    }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llCloseTx()
{
    // Start the connection closing procedure.
    unsigned char discFrame[5] = {
        FLAG, A_TX, C_DISC, A_TX ^ C_DISC, FLAG
    };

    // UA frame sent by the transmitter to finish closing.
    unsigned char uaFrame[5] = {
        FLAG, A_RX, C_UA, A_RX ^ C_UA, FLAG
    };

    // Try to close the connection up to 3 times.
    int attempts = 0;
    int discReceived = FALSE;

    while (attempts < 3 && !discReceived)
    {
        attempts++;

        // Send DISC to the receiver.
        int bytesWritten = writeBytesSerialPort(discFrame, 5);
        if (bytesWritten < 0)
        {
            perror("writeBytesSerialPort");
            return -1;
        }

        printf("DISC sent (attempt %d)\n", attempts);

        // Wait for the receiver's DISC response.
        StateMachine sm;
        stateMachineInit(&sm, A_RX, C_DISC);

        alarmEnabled = TRUE;
        alarm(3);

        while (alarmEnabled && sm.state != STOP)
        {
            unsigned char byte;
            int bytesRead = readByteSerialPort(&byte);

            if (bytesRead < 0)
            {
                // The alarm may interrupt the read operation.
                if (!alarmEnabled)
                    break;

                perror("readByteSerialPort");
                alarm(0);
                alarmEnabled = FALSE;
                return -1;
            }

            if (bytesRead == 0)
                continue;

            stateMachineProcessByte(&sm, byte);
        }

        alarm(0);
        alarmEnabled = FALSE;

        if (sm.state == STOP)
        {
            discReceived = TRUE;
            printf("DISC received correctly\n");
        }
        else
        {
            printf("Timeout waiting for DISC\n");
        }
    }

    // If DISC is not received, the connection cannot be closed.
    if (!discReceived)
    {
        fprintf(stderr, "Failed to close connection: DISC not received\n");
        return -1;
    }

    // Send UA to confirm that the connection is closing.
    int bytesWritten = writeBytesSerialPort(uaFrame, 5);
    if (bytesWritten < 0)
    {
        perror("writeBytesSerialPort");
        return -1;
    }

    printf("UA sent\n");

    // Close the serial port and restore the original settings.
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port closed\n");
    return 0;
}

int llCloseRx()
{
    // Wait for the transmitter to start closing the connection.
    // 1. Receive DISC from transmitter byte by byte
    StateMachine smDisc;
    stateMachineInit(&smDisc, A_TX, C_DISC);

    while (smDisc.state != STOP)
    {
        unsigned char byte;

        int bytesRead = readByteSerialPort(&byte);

        if (bytesRead < 0)
        {
            perror("readByteSerialPort");
            return -1;
        }

        if (bytesRead == 0)
        {
            continue;
        }

        // Validate each byte using the state machine.
        stateMachineProcessByte(&smDisc, byte);
    }

    printf("DISC received correctly\n");

    // 2. Build DISC frame to send back to transmitter
    unsigned char discFrame[5];

    discFrame[0] = FLAG;
    discFrame[1] = A_RX;
    discFrame[2] = C_DISC;
    discFrame[3] = A_RX ^ C_DISC;
    discFrame[4] = FLAG;

    // 3. Send DISC
    int bytesWritten = writeBytesSerialPort(discFrame, 5);

    if (bytesWritten < 0)
    {
        perror("writeBytesSerialPort");
        return -1;
    }

    printf("DISC sent\n");

    // 4. Receive final UA from transmitter byte by byte
    StateMachine smUa;
    stateMachineInit(&smUa, A_RX, C_UA);

    while (smUa.state != STOP)
    {
        unsigned char byte;

        int bytesRead = readByteSerialPort(&byte);

        if (bytesRead < 0)
        {
            perror("readByteSerialPort");
            return -1;
        }

        if (bytesRead == 0)
        {
            continue;
        }

        // Validate each byte using the state machine.
        stateMachineProcessByte(&smUa, byte);
    }

    printf("UA received correctly\n");

    // 5. Close serial port
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port closed\n");

    return 0;
}
