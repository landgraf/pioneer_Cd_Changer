#define F_CPU 8000000UL  // 8 MHz
#define TIMER_FREQ     44100

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define SYSTEM_OFF 0x00
#define HEAD_UNIT_STATUS 0x01
#define TO_TAPE 0x03
#define TO_CD 0x06
#define TO_DASHBOARD 0x21
#define FROM_CD_NOT_PLAYING 0x60
#define FROM_CD_PLAYING 0x61
#define FROM_TUNER 0x71

#define BSRQ PORTD1
#define RST PORTD2


volatile uint8_t value = 0; // Example
// Set flag to 1 once first byte is received
// and reset to 0 if all bytes of current word are received
uint8_t incomplete_trasmission = 0; 

ISR(USART_RXC_vect){
    value = UDR;
}

ISR(TIMER0_COMP_vect){
  PORTD = ~PORTD;
};
static inline int in_transmision(){
  // FIXME real logic here
  return 0 == 1;
};


void SPI_Send_Byte(void){
  while (incomplete_trasmission || in_transmision() ){
  };
  // We have to switch SPI into master mode before talking
  // Set MOSI, SCK and SS as output
  DDRB = (1<<DDB5)|(1<<DDB7)|(1<<DDB4);
  // Enable SPI, Set master
  // Set Clock OCS/64
  // Clock is high when idle
  // Setup on failing edge
  SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<CPOL)|(1<<CPHA);

  // Switch back to slave
  // all input
  // FIXME Isn't it enough to switch clock to input only?
  DDRB = 0x00;
  SPCR &= 0<<MSTR;
}


void SPI_Switch_Slave(void){
  // Switch SPI to slave mode
  // according to atmega16 datasheet it's not needed as far as it will be switched to
  // slave ones SS pin is low
}

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
	
