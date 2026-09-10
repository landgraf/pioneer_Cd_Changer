#ifndef MBUS_H
#define MBUS_H

#include <stdint.h>

// One-time hardware setup: I/O directions, idle pin states.
void Mbus_InitHardware(void);

// Arms the SRQ-pacing timer and the RST-line watchdog.
void Mbus_SetupTimers(void);

// Blocks until the head unit releases RST for the first time, then
// asserts SRQ so it knows a CD changer is present.
void Mbus_WaitForFirstReset(void);

// SRQ is active low, idle high. Assert = pull low (request service).
void Mbus_AssertServiceRequest(void);
// Release = drive high (done requesting, e.g. once the head unit
// starts talking to us).
void Mbus_ReleaseServiceRequest(void);

// Sends one byte and returns whatever the bus reads back.
uint8_t Mbus_SendByte(uint8_t byte);

// Reads one byte, LSB first, as bit-banged on BDATA/BCLK.
uint8_t Mbus_ReadByte(void);

// Reads and discards a length-prefixed message we don't care about.
void Mbus_SkipMessage(void);

// Lights the error indicator on PORTD.
void Mbus_SignalError(void);

#endif
