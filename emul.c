#define F_CPU 2000000UL  // 2 MHz
#define TIMER_FREQ     44100

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define wait_SPI  while (!(SPSR & (1<<SPIF))){}
//#define send_SPI(byte) wait_SPI; 
#define set_bit(reg, bit)  reg |= (1<<(bit))    // Установка бита.
#define clr_bit(reg, bit)   reg &= (~(1<<(bit))) // Сброс бита.
#define switch_bit(reg, bit)  reg ^= (1<<(bit)) // Переключение бита.

#define BDATA 5
#define BCLK 7
#define BRXEN 4
#define BSRQ 3
#define BRST 2


volatile uint8_t has_message = 0;
volatile uint8_t message = 0;

volatile uint8_t us = 0;
volatile uint8_t um = 0;

uint8_t counter = 0;

ISR(USART_RXC_vect)
{
  //  has_message = 15;
  while((UCSRA &(1<<UDRE)) == 0);
  us = UDR;
  um = 1;
}

void USART_Init(void){
   UBRRL = BAUD_PRESCALE;// Load lower 8-bits into the low byte of the UBRR register
   UBRRH = (BAUD_PRESCALE >> 8); 
   UCSRB = ((1<<TXEN)|(1<<RXEN) | (1<<RXCIE));
}

void SPI_Send_Byte(uint8_t byte)
{
  uint8_t delay = 50;
  PORTB = 0xff;
  DDRB = 0xff; // PORTB is output
  PORTB = 0xff; // idle high
  clr_bit (PORTB, BRXEN);
  _delay_us(delay*2); // Because Pioneer does it (:
  for (int i = 0; i<8; i++) // while looks better (?)
    {
      _delay_us(delay);
      uint8_t set = byte & 1<<i;
      // tick clock low
      clr_bit (PORTB, BCLK);
      // set value
      if ( set )
        {
          set_bit (PORTB, BDATA);
        }
      else
        {
          clr_bit (PORTB, BDATA);
        };
      _delay_us(delay);
      // tick clock high
      // reader should pick the value up
      set_bit (PORTB, BCLK);
    }
  set_bit (PORTB, BDATA);
  _delay_us(delay*5);
  set_bit (PORTB, BRXEN);
  DDRB = 0x00; // All input
  PORTB = 0x00; // HighZ
}

void USART_Send_Byte(uint8_t u8Data){
  // Wait until last byte has been transmitted
  while((UCSRA &(1<<UDRE)) == 0);
  // Transmit data
  UDR = u8Data;
}

void Led_Init(){
  set_bit(DDRD, DDD5);
  clr_bit(PORTD, DDD5);
}

int main(void)
{
  USART_Init();
  Led_Init();
  sei();

  for (;;)
    {
      SPI_Send_Byte (counter++);
      _delay_ms(200);
      /*      if (um)
        {
          switch_bit(PORTD, DDD5);
          USART_Send_Byte(us);
          SPI_Send_Byte(counter++);
          um = 0;
          }*/
    }
}
