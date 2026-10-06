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

    // Cria o frame SET com 5 bytes
    unsigned char SET[5];

    SET[0] = 0x7E; // FLAG inicial
    SET[1] = 0x03; // Endereço do emissor
    SET[2] = 0x03; // Campo de controlo: SET
    SET[3] = SET[1] ^ SET[2]; // BCC1: XOR entre A e C
    SET[4] = 0x7E; // FLAG final

    // Envia o frame SET para o recetor
    int bytesWritten = writeBytesSerialPort(SET, 5);
    printf("%d bytes written to serial port\n", bytesWritten);

    // Cria espaço para guardar o frame UA recebido
    unsigned char UA[5];

    // Lê os 5 bytes do frame UA
    for (int i = 0; i < 5; i++)
    {
        int bytesRead = readByteSerialPort(&UA[i]);
        // Verifica se foi lido um byte
        if (bytesRead != 1)
        {
            printf("Error reading UA\n");
            return -1;
        }
    }

    // Mostra o UA recebido em hexadecimal
    printf("UA received: ");

    for (int i = 0; i < 5; i++)
    {
        printf("0x%02X ", UA[i]);
    }

    printf("\n");

    // Verifica se o frame UA recebido está correto
    if (UA[0] == 0x7E &&
    UA[1] == 0x01 &&
    UA[2] == 0x07 &&
    UA[3] == (UA[1] ^ UA[2]) &&
    UA[4] == 0x7E)
    {
        printf("UA received correctly\n");
    }
    else
    {
        printf("Invalid UA\n");
        return -1;
    }

    // Close serial port
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port %s closed\n", llParameters.serialPort);

    return 0;
}

int llOpenRx(LinkLayer llParameters)
{
    // ----------------------------------------------------
    // This example code shows how to open the serial port and receive a string.
    // TODO: Adapt and extend this code according to the specifications of the project.
    // ----------------------------------------------------

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    // Read from serial port until the 'z' char is received.

    // NOTE: This while() cycle is a simple example showing how to read from the serial port.
    // It must be changed in order to respect the specifications of the protocol indicated in the Lab guide.

    // TODO: Save the received bytes in a buffer array and print it at the end of the program.
    volatile int STOP = FALSE;
    int nBytesBuf = 0;

    while (STOP == FALSE)
    {
        // Read one byte from serial port.
        // NOTE: You must check how many bytes were actually read by reading the return value.
        // In this example, we assume that the byte is always read, which may not be true.
        unsigned char byte;
        int bytes = readByteSerialPort(&byte);
        nBytesBuf += bytes;

        printf("Byte received: %c\n", byte);

        if (byte == 'z')
        {
            printf("Received 'z' char. Stop reading from serial port.\n");
            STOP = TRUE;
        }
    }

    printf("Total bytes received: %d\n", nBytesBuf);

    // Close serial port
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port %s closed\n", llParameters.serialPort);

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
