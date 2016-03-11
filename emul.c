#define F_CPU 8000000UL  // 8 MHz
#define TIMER_FREQ     44100

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
// Wires
// Black - BDATA
// 1 - BSRQ
// UTP - RST
// 2 - BRXEN
// 0 - SCK


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


#define wait_SPI  while (!(SPSR & (1<<SPIF))){}


// Set flag to 1 once first byte is received
// and reset to 0 if all bytes of current word are received
uint8_t incomplete_trasmission = 0;

// test usart
volatile uint8_t sendback = 0;
volatile uint8_t sendback_value = 0;

//transmission buffer
volatile uint8_t SPI_to_trasnmit[256];
volatile uint8_t SPI_last = 0;

// receive buffer
// FIXME this is ugly
// We use half duplex mode and no reason no have buffer
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

ISR(TIMER1_OVF_vect){
  PORTA = ~PORTA;
}

ISR(USART_RXC_vect){
  sendback++;
  sendback_value = UDR;
}

void SPI_Init_Slave(void){
  // All input. Don't output in slave mode
  // See datasheet for supported modes
  DDRB = 0x00;
  PORTB = 0x00;
  SPCR |= _BV(SPE);  // Enable SPI
  SPCR |= _BV(SPIE); // Enanle SPI interrupts
  SPCR |= _BV(DORD); // LS bit first
  SPCR |= _BV(CPOL); // Clock is idle high
  SPCR |= _BV(CPHA); // Setup of failing (leading) edge  
}

void SPI_Send_Byte(uint8_t byte){

  // don't try to talk while previous message is incomplete
  // FIXME possible deadlock. That's why we have to use interrupts here
  while (incomplete_trasmission){
  };
  
  // We have to switch SPI into master mode before talking

  DDRB  |= _BV(DDB4); // SS is outtput
  PORTB |=  0<<DDB4; // pull SS down
  
  DDRB |= _BV(DDB5); // MOSI is output
  DDRB |= _BV(DDB7); // SCK is output


  //  PINB |= _BV(DDB4); // Enable pull-up

  SPCR |= _BV(SPE);  // Enable SPI
  SPCR |= _BV(SPIE); // Enable SPI interrupts
  SPCR |= _BV(DORD); // LS bit first
  SPCR |= _BV(MSTR); // I'm master
  SPCR |= _BV(SPR1); // Scale 1/64
  SPCR |= _BV(CPOL); // Clock is idle high
  SPCR |= _BV(CPHA); // Setup of failing (leading) edge

  // Send byte here
  _delay_us(500);
  SPDR = byte;
  wait_SPI;
  
  //Switch to Slave
  SPI_Init_Slave();
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
      length--;
    }
  while (length);
  incomplete_trasmission = 0;

}

void Respond_Message()
{
  // Read "length"
  while (!messages){
  };
  uint8_t length = SPI_Received[last_read++];
  messages--;

  // There are few possible messages

  // 06 00    -> first message
  //          -- Response 60 01 18

  SPI_Send_Byte(0x60);

  if ( 0x00 == length )
    {
      SPI_Send_Byte (0x01);
      SPI_Send_Byte (0x18);
      return;
    };
  
  if ( 0x01 == length ){
    uint8_t command  = SPI_Received[last_read++];
    switch (command) {
      // 06 01 F0 -> Startup "F0" message
      //          -- response 60 03 18 00 01
    case 0xf0 :
      SPI_Send_Byte(0x01);
      SPI_Send_Byte(0xFD);
      break;
      // 06 01 F6 -> Startup "F5" message
      // 06 01 00 -> Status
      //          -- response 06 0A 18 00 F1 01 01 01 00 3F 03 00
    case 0xf6 | 0x00 :
      SPI_Send_Byte(0x0a);
      uint8_t body[10] = {0x18, 0x00, 0xf1, 0x01, 0x01, 0x01, 0x00, 0x3f, 0x03, 0x00};
      for (int i = 0; i < 10; i++){
        SPI_Send_Byte(body[i]);
      };
      break;
    default:
      SPI_Send_Byte(0x00);
      break;
    };

  };
  
  
  
};

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
      Respond_Message();
      break;
    case SYSTEM_OFF | HEAD_UNIT_STATUS | TO_TAPE | TO_DASHBOARD | FROM_TUNER :
      // skip message
      Skip_Message();
      break;
    default :
      // ignore everything else for now
      // raise Error
      // it means we lost some transmissions and have to recover from this situration
      break;
    }
  messages--;
}



void Set_SRQ_Timer(){

  DDRD = 0x00;
  DDRD |= 1<<4;
  DDRD |= 1<<5;
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
  PORTA = 0x00;
  DDRA = 0xff;
  Set_SRQ_Timer();
  //Set_SPI_Transmitter;
  USART_Init();
  
  sei();         // enable all interrupts   
  SPI_Init_Slave();
  
  
  for(;;){
    if (sendback){
      USART_SendByte(sendback_value);
      sendback = 0;
    }
    
    if (!incomplete_trasmission && messages) 
      {
        // proceed message
        SPI_Proceed_Message();
      }
  }
}

	
