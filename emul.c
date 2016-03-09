#define F_CPU 8000000UL  // 8 MHz
#define TIMER_FREQ     44100

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

//
#define DEBUG 1
//

// Command codes
#define SYSTEM_OFF 0x00
#define HEAD_UNIT_STATUS 0x01
#define TO_TAPE 0x03
#define TO_CD 0x06
#define TO_DASHBOARD 0x21
#define FROM_CD_NOT_PLAYING 0x60
#define FROM_CD_PLAYING 0x61
#define FROM_TUNER 0x71
//


#define BSRQ PORTD1
#define RST PORTD2 // Reset signal should be used for suspend/power_saving


// Set flag to 1 once first byte is received
// and reset to 0 if all bytes of current word are received
uint8_t incomplete_trasmission = 0;

//transmission buffer
volatile uint8_t SPI_to_trasnmit[256];
volatile uint8_t SPI_last = 0;

// receive buffer
volatile uint8_t messages = 0; // ??
volatile uint8_t last_received = 0x00;
volatile uint8_t last_read = 0x00;
volatile uint8_t SPI_Received[256];

ISR(SPI_STC_vect){

  while (!(SPSR & (1<<SPIF))){
  };
  // if we're master then trash received data (it'll be sent data because of SISO)
  if ( (1<<MSTR) & SPCR)
    {
      uint8_t dummy = SPDR;
    }
  else
    {
      SPI_Received[last_received++] = SPDR;
      messages++;
    };
}

void SPI_Send_Byte(void){

  // don't try to talk while previous message is incomplete
  // FIXME possible deadlock. That's why we have to use interrupts here
  while (incomplete_trasmission){
  };
  
  // We have to switch SPI into master mode before talking

  DDRB |= _BV(DDB5); // MOSI is output
  DDRB |= _BV(DDB7); // SCK is output

  DDRB |= ~(_BV(DDB4)); // SS is intput
  PINB |= _BV(DDB4); // Enable pull-up

  SPCR |= _BV(SPE);  // Enable SPI
  SPCR |= _BV(SPIE); // Enable SPI interrupts
  SPCR |= _BV(DORD); // LS bit first
  SPCR |= _BV(MSTR); // I'm master
  SPCR |= _BV(SPR1); // Scale 1/64
  SPCR |= _BV(CPOL); // Clock is idle high
  SPCR |= _BV(CPHA); // Setup of failing (leading) edge

  // Send byte here
  // FIXME

  //Disable SPI
  SPCR &= 0x00;
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

void SPI_Init_Slave(void){
  // All input. Don't output in slave mode
  // See datasheet for supported modes
  DDRB = 0x00;
  SPCR |= _BV(SPE);  // Enable SPI
  SPCR |= _BV(SPIE); // Enanle SPI interrupts
  SPCR |= _BV(DORD); // LS bit first
  SPCR |= _BV(CPOL); // Clock is idle high
  SPCR |= _BV(CPHA); // Setup of failing (leading) edge  
}

void Skip_Message(void){
  // at this point "command" shound be read and
  // next expected byte is length
  incomplete_trasmission = 1;
  // Read "length"
  while (!messages){
  };
  uint8_t length = SPI_Received[last_read++];
  messages--;
  do
    {
      // wait for message to read
      while(!messages){
      };
      // move read cursor
      // trash useless messages
      last_read++;
      messages--;
    }
  while (length--);
  incomplete_trasmission = 0;
}


void SPI_Proceed_Message(void)
{
  if (DEBUG)
    {
      for (int i = last_read; i < last_received; i++)
        {
          USART_SendByte(SPI_Received[i]);
        }
    };
  
  
  switch (SPI_Received[last_read++])
    {
    case TO_CD :
      Skip_Message();
      break;
    default :
      // ignore everything else for now
      Skip_Message();
      break;
    }
  messages--;
}



void Set_SRQ_Timer(){

  // PORTD is output
  DDRD = _BV(PD4)  | _BV(PD5);
  PORTD = 0x00;
  PORTD |= _BV(PD4);
  
  // CD-Changer should send SRQ request to head unit
  // to achieve this use simple "blink on overflow"
  TCCR1A = _BV(COM1A0) | _BV(COM1B0); // toggle OC1A OC1B on compare/match
  TCCR1B = _BV(CS11) | _BV(CS10); // prescale 64 524.288 ms 
  TIMSK = _BV(TOIE1); // Enable overflow interrupts
};

void Set_SPI_Transmitter(void)
{
}


int main(void){
   Set_SRQ_Timer();
   //Set_SPI_Transmitter;
   USART_Init();

   sei();         // enable all interrupts   

   for(;;){
     // pulling SS to check if there's transmissions
     if (! (PINB & _BV(DDB4)))
       {
         // if SPI is not enabled - enable it
         if (!(SPCR & _BV(SPE)))
           {
             SPI_Init_Slave();
           }
       }
     else
       // SS is idle high. Disabling SPI
       {
         if (!(SPCR & _BV(SPE)))
           {
             // Disable SPI
             SPCR = 0x00;
             // We have messages to process
             if (!incomplete_trasmission && messages) 
               {
                 // proceed message
                 SPI_Proceed_Message();
               }
           }
       }
   }
}

	
