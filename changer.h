#ifndef CHANGER_H
#define CHANGER_H

#include <stdint.h>

void Changer_Init(void);

// Called from the SRQ timer ISR, once per ~524ms tick.
void Changer_Tick(void);

// Called from the RST-watchdog ISR.
void Changer_EnterRecovery(void);
uint8_t Changer_InRecovery(void);

// Reads and handles one "to CD" (0x06) message body from the bus, then
// replies with the appropriate status message.
void Changer_HandleCdMessage(void);

#endif
