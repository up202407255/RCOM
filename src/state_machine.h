#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

typedef enum
{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP
} State;

typedef struct
{
    State state;
    unsigned char expectedAddress;
    unsigned char expectedControl;
} StateMachine;

void stateMachineInit(StateMachine *sm,
                      unsigned char address,
                      unsigned char control);

int stateMachineProcessByte(StateMachine *sm,
                            unsigned char byte);

#endif