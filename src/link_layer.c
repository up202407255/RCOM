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

int alarmEnabled = FALSE;

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

    int attempts = 0;
    int connected = FALSE;
    int state;

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

        state = 0;

        while (alarmEnabled && state != 5)
        {
            unsigned char byte;
            int bytesRead = readByteSerialPort(&byte);

            if (bytesRead != 1)
                continue;

            printf("Byte received: 0x%02X\n", byte);

            switch (state)
            {
                case 0:
                    if (byte == FLAG)
                        state = 1;
                    break;

                case 1:
                    if (byte == A_RX)
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
                    if (byte == (A_RX ^ C_UA))
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

    // Se não conseguiu receber um UA válido após 3 tentativas
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
    unsigned char byte;

    unsigned char control = 0;
    unsigned char stuffedData[2 * BUF_SIZE + 2];
    int stuffedSize = 0;

    int state = 0;

     while (1)
    {
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

        printf("Byte received: 0x%02X\n", byte);

        switch (state)
        {
            // Waiting for initial FLAG
            case 0:
                if (byte == FLAG)
                {
                    state = 1;
                }
                break;

            // Waiting for address
            case 1:
                if (byte == FLAG)
                {
                    // Multiple FLAGs are allowed
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

            // Waiting for control field
            case 2:
                if (byte == C_I0 || byte == C_I1)
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

            // Validate BCC1
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
                    // Wrong BCC1 -> ignore frame
                    state = 0;
                }
                break;

            // Receive DATA + BCC2 until final FLAG
            case 4:
                if (byte == FLAG)
                {
                    state = 5;
                }
                else
                {
                    if (stuffedSize >= 2 * BUF_SIZE + 2)
                    {
                        printf("Frame too large\n");
                        return -1;
                    }

                    stuffedData[stuffedSize++] = byte;
                }
                break;
        }

        if (state == 5)
        {
            break;
        }
    }
    printf("I frame received\n");

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llCloseTx()
{
    // Trama DISC enviada pelo emissor.
    unsigned char discFrame[5] = {
        FLAG, A_TX, C_DISC, A_TX ^ C_DISC, FLAG
    };

    // Trama UA enviada pelo emissor no final do fecho.
    unsigned char uaFrame[5] = {
        FLAG, A_TX, C_UA, A_TX ^ C_UA, FLAG
    };

    // Tenta estabelecer o fecho da ligação até 3 vezes.
    int attempts = 0;
    int discReceived = FALSE;

    while (attempts < 3 && !discReceived)
    {
        attempts++;

        // Envia DISC ao recetor.
        int bytesWritten = writeBytesSerialPort(discFrame, 5);
        if (bytesWritten < 0)
        {
            perror("writeBytesSerialPort");
            return -1;
        }

        printf("DISC sent (attempt %d)\n", attempts);

        // Aguarda a resposta DISC do recetor.
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
                // O alarme pode interromper a leitura.
                if (!alarmEnabled)
                    break;

                perror("readByteSerialPort");
                alarm(0);
                alarmEnabled = FALSE;
                return -1;
            }

            if (bytesRead == 0)
                continue;

            printf("Byte received: 0x%02X\n", byte);
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

    // Se não recebemos DISC, não podemos concluir o fecho.
    if (!discReceived)
    {
        fprintf(stderr, "Failed to close connection: DISC not received\n");
        return -1;
    }

    // Envia UA para confirmar o fecho da ligação.
    int bytesWritten = writeBytesSerialPort(uaFrame, 5);
    if (bytesWritten < 0)
    {
        perror("writeBytesSerialPort");
        return -1;
    }

    printf("UA sent\n");

    // Fecha a porta série e restaura as configurações originais.
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

        printf("Byte received: 0x%02X\n", byte);

        // Validate each byte using the state machine
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
    stateMachineInit(&smUa, A_TX, C_UA);

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

        printf("Byte received: 0x%02X\n", byte);

        // Again, validate byte by byte
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
