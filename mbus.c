// Physical/bit-bang layer for the Pioneer M-bus. Knows about pins, SPI
// hardware and timing, but nothing about CD-changer protocol semantics.

#include "mbus.h"
#include "config.h"
#include "changer.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define set_bit(reg, bit) ((reg) |= (1 << (bit)))
#define clr_bit(reg, bit) ((reg) &= ~(1 << (bit)))

// Loops while `bit` is set / clear in `reg`.
#define wait_low(reg, bit)  while (bit_is_set((reg), (bit))) {}
#define wait_high(reg, bit) while (bit_is_clear((reg), (bit))) {}

void Mbus_SignalError(void)
{
  // Requires DDRD to be configured as output; see Mbus_InitHardware().
  PORTD = 0xff;
}

static void Mbus_InitTransmitter(void)
{
  PORTB = 0xff;
  DDRB = (1 << BRXEN) | (1 << BDATA) | (1 << BCLK) | (1 << BSRQ); // MOSI/SCK/SS outputs
  clr_bit(PORTB, BRXEN);
  _delay_us(20);
  clr_bit(PORTB, BDATA);
  _delay_us(20);

  SPSR |= (1 << SPI2X);
  SPCR = (1 << SPE) | (1 << MSTR) | (1 << DORD) | (1 << SPR0) |
         (1 << SPI2X) | (1 << SPR1) | (1 << CPOL) | (1 << CPHA); // Master, f/16
}

static void Mbus_DisableTransmitter(void)
{
  clr_bit(SPCR, SPE);
}

uint8_t Mbus_SendByte(uint8_t byte)
{
  wait_high(PINB, BRXEN);

  Mbus_InitTransmitter();

  SPDR = byte;
  wait_high(SPSR, SPIF);
  Mbus_DisableTransmitter();
  _delay_us(6);

  clr_bit(DDRB, BRXEN);
  DDRB = 1 << BSRQ;
  PORTB = 0xff;
  _delay_us(440);

  return SPDR;
}

uint8_t Mbus_ReadByte(void)
{
  wait_low(PINB, BRXEN);

  uint8_t byte = 0;
  for (uint8_t bit = 0; bit < 8; bit++)
    {
      if (bit_is_set(PINB, BRXEN))
        {
          Mbus_SignalError();
          return 0xBB;
        }
      wait_low(PINB, BCLK);
      wait_high(PINB, BCLK);

      if (bit_is_set(PINB, BDATA))
        {
          set_bit(byte, bit);
        }
      else
        {
          clr_bit(byte, bit);
        }
    }
  return byte;
}

void Mbus_SkipMessage(void)
{
  uint8_t length = Mbus_ReadByte();
  while (length--)
    {
      Mbus_ReadByte();
    }
}

void Mbus_AssertServiceRequest(void)
{
  clr_bit(PORTB, BSRQ);
}

void Mbus_ReleaseServiceRequest(void)
{
  set_bit(PORTB, BSRQ);
}

void Mbus_WaitForFirstReset(void)
{
  wait_high(PINB, BRST);
  _delay_ms(1.5);
  Mbus_AssertServiceRequest();
}

void Mbus_InitHardware(void)
{
  PORTB = 1 << BSRQ;
  DDRB  = 1 << BSRQ;
  // PORTD drives the error-indicator LEDs; it must be an output for
  // Mbus_SignalError() to actually be visible (previously never set).
  DDRD  = 0xff;
}

void Mbus_SetupTimers(void)
{
  // SRQ pacing: while playing/paused, the changer must keep re-asserting
  // SRQ to get re-polled by the head unit. Prescale 64 -> ~524ms/overflow.
  TCCR1B = (1 << CS11) | (1 << CS10);
  TIMSK  = (1 << TOIE1);

  // RST-line watchdog, sampled on every timer2 overflow.
  TCCR2  = (1 << CS20);
  TIMSK |= (1 << TOIE2);
}

ISR(TIMER1_OVF_vect)
{
  static uint8_t srq_counter = 0;

  Changer_Tick();
  if (++srq_counter == 10)
    {
      Mbus_AssertServiceRequest();
      srq_counter = 0;
    }
}

ISR(TIMER2_OVF_vect)
{
  if (bit_is_clear(PINB, BRST))
    {
      Changer_EnterRecovery();
    }
  if (Changer_InRecovery())
    {
      Mbus_AssertServiceRequest();
    }
}
