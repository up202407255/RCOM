#include "state_machine.h"

#define FLAG 0x7E

void stateMachineInit(StateMachine *sm,
                      unsigned char address,
                      unsigned char control)
{
    sm->state = START;
    sm->expectedAddress = address;
    sm->expectedControl = control;
}

int stateMachineProcessByte(StateMachine *sm,
                            unsigned char byte)
{
    switch (sm->state)
    {
        case START:
            if (byte == FLAG)
                sm->state = FLAG_RCV;
            break;

        case FLAG_RCV:
            if (byte == sm->expectedAddress)
                sm->state = A_RCV;
            else if (byte != FLAG)
                sm->state = START;
            break;

        case A_RCV:
            if (byte == sm->expectedControl)
                sm->state = C_RCV;
            else if (byte == FLAG)
                sm->state = FLAG_RCV;
            else
                sm->state = START;
            break;

        case C_RCV:
            if (byte ==
                (sm->expectedAddress ^ sm->expectedControl))
            {
                sm->state = BCC_OK;
            }
            else if (byte == FLAG)
            {
                sm->state = FLAG_RCV;
            }
            else
            {
                sm->state = START;
            }
            break;

        case BCC_OK:
            if (byte == FLAG)
            {
                sm->state = STOP;
                return 1;
            }
            else
            {
                sm->state = START;
            }
            break;

        case STOP:
            return 1;
    }

    return 0;
}