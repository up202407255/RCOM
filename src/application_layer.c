// RCOM 2026/2027
//
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <string.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // ----------------------------------------------------
    // TODO: Adapt and extend this code according to the specifications of the project.
    // ----------------------------------------------------

    LinkLayer llParameters = {
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout,
    };
    strcpy(llParameters.serialPort, serialPort);

    if (strcmp(role, "tx") == 0)
    {
        // Estabelece a ligação como emissor, enviando SET e aguardando UA.
        // Se a abertura falhar, termina sem tentar fechar uma ligação inexistente.
        if (llOpenTx(llParameters) < 0)
        {
            return;
        }

        // Fecha a porta série após o estabelecimento da ligação.
        // NOTA: por enquanto, ainda não existe transferência do ficheiro.
        if (llCloseTx() < 0)
        {
            return;
        }

        printf("Ligação TX fechada com sucesso.\n");
    }
    else if (strcmp(role, "rx") == 0)
    {
        if (llOpenRx(llParameters) < 0)
        {
            return;
        }

        if (llCloseRx() < 0)
        {
            return;
        }

        printf("Ligação RX fechada com sucesso.\n");
    }
    else
    {
        printf("Invalid role: %s. Must be 'tx' or 'rx'.\n", role);
        return;
    }
}
