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

        printf("UA received: ");

        for (int i = 0; i < 5; i++)
        {
            printf("0x%02X ", UA[i]);
        }

        printf("\n");

        // Verifica se ocorreu timeout
        if (alarmEnabled == FALSE)
        {
            printf("Timeout. Retrying...\n");
            continue;
        }

        // Verifica se o frame UA recebido está correto
        if (UA[0] == FLAG &&
            UA[1] == A_RX &&
            UA[2] == C_UA &&
            UA[3] == (A_RX ^ C_UA) &&
            UA[4] == FLAG)
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


    return 0;
}

int llOpenRx(LinkLayer llParameters)
{
    // Open serial port
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    // Initialize state machine to receive a SET frame
    StateMachine sm;
    stateMachineInit(&sm, A_TX, C_SET);

    int nBytesBuf = 0;

    // Wait until a valid SET frame is received
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

        printf("Byte received: 0x%02X\n", byte);

        // Let the state machine process the received byte
        stateMachineProcessByte(&sm, byte);
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

    // Send UA frame
    int bytesWritten = writeBytesSerialPort(uaFrame, 5);

    if (bytesWritten < 0)
    {
        perror("writeBytesSerialPort");
        return -1;
    }

    printf("%d bytes written to serial port\n", bytesWritten);
    printf("UA sent\n");

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
