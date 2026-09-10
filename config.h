#ifndef CONFIG_H
#define CONFIG_H

#define F_CPU 8000000UL
#define USART_BAUDRATE 9600

// M-bus wiring on PORTB.
// Black  - BDATA   Yellow - BSRQ   Green - BRST
// Red    - BRXEN   Blue   - BCLK   Ground - GND
#define BDATA  5
#define NODATA 6
#define BCLK   7
#define BRXEN  4
#define BSRQ   3
#define BRST   2

// Delay before replying to the head unit, matches the timing of a
// genuine Pioneer CD changer.
#define REPLY_DELAY_MS 9

#endif
