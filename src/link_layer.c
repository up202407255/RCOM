// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

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

int alarmEnabled = FALSE;

void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
}

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

    // Configura o alarme
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;

    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        return -1;
    }

    // Cria o frame SET com 5 bytes
    unsigned char SET[5];

    SET[0] = 0x7E; // FLAG inicial
    SET[1] = 0x03; // Endereço do emissor
    SET[2] = 0x03; // Campo de controlo: SET
    SET[3] = SET[1] ^ SET[2]; // BCC1: XOR entre A e C
    SET[4] = 0x7E; // FLAG final

    // Cria espaço para guardar o frame UA recebido
    unsigned char UA[5];

    int attempts = 0;
    int connected = FALSE;

    while (attempts < 3  && connected == FALSE)
    {
        attempts++;

        // Envia o frame SET para o recetor
        int bytesWritten = writeBytesSerialPort(SET, 5);
        printf("%d bytes written to serial port\n", bytesWritten);
    
        // Ativa o alarme durante 3 segundos
        alarmEnabled = TRUE;
        alarm(3);  

        // Tenta receber o UA 
        for (int i = 0; i < 5; i++)
        {
            int bytesRead = readByteSerialPort(&UA[i]);
            // Verifica se foi lido um byte
            if (bytesRead != 1)
            {
                break;
            }
        }

        // Verifica se ocorreu timeout
        if (alarmEnabled == FALSE)
        {
            printf("Timeout. Retrying...\n");
            continue;
        }

        // Verifica se o frame UA recebido está correto
        if (UA[0] == 0x7E &&
            UA[1] == 0x01 &&
            UA[2] == 0x07 &&
            UA[3] == (UA[1] ^ UA[2]) &&
            UA[4] == 0x7E)
        {
            connected = TRUE;
            // O UA chegou, por isso já não precisamos do alarme
            alarm(0);
            alarmEnabled = FALSE;
        }

    }

    // Se não conseguiu receber um UA válido após 3 tentativas
    if (connected == FALSE)
    {
        printf("Failed to receive UA after 3 attempts\n");
        closeSerialPort();
        return -1;
    }

    // Mostra o UA recebido em hexadecimal
    printf("UA received: ");
    for (int i = 0; i < 5; i++)
    {
        printf("0x%02X ", UA[i]);
    }
    printf("\n");

    printf("UA received correctly\n");

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
    // Open the serial port and wait for a valid SET frame.
    // ----------------------------------------------------

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);


    // States used to recognize the SET frame
    enum
    {
        START,
        FLAG_RCV,
        A_RCV,
        C_RCV,
        BCC_OK
    };

    int state = START;

    volatile int STOP = FALSE;
    int nBytesBuf = 0;

    while (STOP == FALSE)
    {
        // Read one byte from serial port
        unsigned char byte;
        int bytes = readByteSerialPort(&byte);

        // Check if a byte was actually read
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

        printf("Byte received: 0x%02X\n", byte);

        // State machine to detect SET:
        // 7E 03 03 00 7E
        switch (state)
        {
            case START:
                if (byte == FLAG)
                    state = FLAG_RCV;
                break;

            case FLAG_RCV:
                if (byte == A_TX)
                    state = A_RCV;
                else if (byte != FLAG)
                    state = START;
                break;

            case A_RCV:
                if (byte == C_SET)
                    state = C_RCV;
                else if (byte == FLAG)
                    state = FLAG_RCV;
                else
                    state = START;
                break;

            case C_RCV:
                if (byte == (A ^ C_SET))
                    state = BCC_OK;
                else if (byte == FLAG)
                    state = FLAG_RCV;
                else
                    state = START;
                break;

            case BCC_OK:
                if (byte == FLAG)
                    STOP = TRUE;
                else
                    state = START;
                break;
        }
    }

    printf("SET received correctly\n");
    printf("Total bytes received: %d\n", nBytesBuf);

    // Build UA frame
    unsigned char uaFrame[5];

    uaFrame[0] = FLAG;
    uaFrame[1] = A_RX;
    uaFrame[2] = C_UA;
    uaFrame[3] = A_RX ^ C_UA;
    uaFrame[4] = FLAG;

    // Send UA
    int bytesWritten = writeBytesSerialPort(uaFrame, 5);

    if (bytesWritten < 0)
    {
        perror("writeBytesSerialPort");
        return -1;
    }   

    printf("%d bytes written to serial port\n", bytesWritten);

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
