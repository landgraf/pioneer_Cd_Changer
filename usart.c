// Debug-only UART, not part of the M-bus protocol. Currently unused by
// the rest of the firmware but kept available for future debug logging.

#include "usart.h"
#include "config.h"

#include <avr/io.h>

#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

void USART_Init(void)
{
  UBRRL = BAUD_PRESCALE;
  UBRRH = (BAUD_PRESCALE >> 8);
  // TX only: RX is never read, and no USART_RXC_vect handler is
  // installed. Enabling RXCIE without a handler used to mean any
  // received byte triggered avr-libc's unhandled-interrupt trap and
  // reset the MCU.
  UCSRB = (1 << TXEN);
}

void USART_SendByte(uint8_t data)
{
  while (!(UCSRA & (1 << UDRE)))
    ;
  UDR = data;
}
