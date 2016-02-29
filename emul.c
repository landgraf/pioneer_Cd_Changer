#define F_CPU 8000000UL  // 8 MHz
#define TIMER_FREQ     44100

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

volatile uint8_t value = 0; // Example
uint16_t counter = 44100; // DEBUG ONLY!

ISR(USART_RXC_vect){
    value = UDR;
}

ISR(TIMER0_COMP_vect){
  if (++counter == TIMER_FREQ) //DEBUG ONLY:  We expect 1 sec here
    {
      PORTD = ~PORTD;
      counter = 0;
    };

};

void USART_Init(void){
   UBRRL = BAUD_PRESCALE;// Load lower 8-bits into the low byte of the UBRR register
   UBRRH = (BAUD_PRESCALE >> 8); 
   UCSRB = ((1<<TXEN)|(1<<RXEN) | (1<<RXCIE));
}


void USART_SendByte(uint8_t u8Data){
  // Wait until last byte has been transmitted
  while((UCSRA &(1<<UDRE)) == 0);
  // Transmit data
  UDR = u8Data;
}


// not being used but here for completeness
// Wait until a byte has been received and return received data 
uint8_t USART_ReceiveByte(){
  while((UCSRA &(1<<RXC)) == 0);
  return UDR;
}

void Led_init(void){
   //outputs, all off
  DDRB = 0xFF;
  DDRD = 0xFF;
  PORTD = 1<<4;
}

void Set_Clock_Timer(){
  TCCR0 = _BV(CS00) | _BV(WGM01); // No prescale | compare/match
  TIMSK = _BV(OCIE0); // Enable compare/match interrupts
  OCR0 = F_CPU/TIMER_FREQ; // Compare match 181 gives us 0.5us differenc
};

int main(void){
   USART_Init();  // Initialise USART
   sei();         // enable all interrupts
   Led_init();    // init LEDs for testing
   Set_Clock_Timer();
   for(;;){    // Repeat indefinitely
          USART_SendByte(F_CPU/TIMER_FREQ);  // send value
          _delay_ms(499);
   }
}
	
