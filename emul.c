#define F_CPU 8000000UL  // 2 MHz
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

// looping when bit is 1
//#define wait_low(reg, bit) while(reg & (1<<bit)){}
#define wait_low(reg, bit) while(bit_is_set(reg,bit)){}
// looping when bit is 0
#define wait_high(reg, bit) while(! bit_is_set(reg,bit)){}

// Wires
// Black - BDATA
// Yello - BSRQ
// Grren - RST
// Red - BRXEN
// Blue - SCK
// Ground //


#define BDATA 5
#define NODATA 6
#define BCLK 7
#define BRXEN 4
#define BSRQ 3
#define BRST 2

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

const uint8_t interval = 9;

uint8_t is_playing = 0;
uint8_t is_paused = 0;
uint8_t is_random = 0;
uint8_t counter = 0;

uint8_t disk = 0;
uint8_t track = 0;
uint8_t mins = 0;
volatile uint8_t secs = 0;
volatile uint8_t in_recovery = 0;

ISR(USART_RXC_vect)
{
  //  has_message = 15;
  while((UCSRA &(1<<UDRE)) == 0);
  uint8_t dummy = UDR;
}

ISR(TIMER2_OVF_vect)
{
  in_recovery = bit_is_clear (PINB, BRST);
  if (in_recovery)
    {
      clr_bit (PORTB, BSRQ);
    };  
}

ISR(TIMER1_OVF_vect)
{
  secs++;
  if (++counter == 10)
    {
      clr_bit(PORTB, BSRQ);
      counter = 0;
    }
}

void USART_Init(void){
   UBRRL = BAUD_PRESCALE;// Load lower 8-bits into the low byte of the UBRR register
   UBRRH = (BAUD_PRESCALE >> 8); 
   UCSRB = ((1<<TXEN)|(1<<RXEN) | (1<<RXCIE));
}

void Led_Error(void)
{
  PORTD = 0xff;
}

void SPI_Init_Master()
{
  PORTB = 0xff;
  DDRB = ((1<<BRXEN)|(1<<BDATA)|(1<<BCLK)|(1<<BSRQ)); //spi pins on port b MOSI SCK,SS outputs
  clr_bit(PORTB, BRXEN);
  _delay_us(20);
  clr_bit(PORTB, BDATA);
  _delay_us(20);
  /*
  SPSR |= (1<<SPI2X);
  SPCR = ((1<<SPE)|(1<<MSTR)|(1<<DORD)|(1<<SPR0)|(1<<SPI2X)|(1<<SPR1)|(1<<CPOL)|(1<<CPHA));  // SPI enable, Master, f/16
  */
  set_bit(SPCR, MSTR); // master 
  set_bit(SPCR, SPR0);  // ** 
  clr_bit(SPSR, SPI2X); // ** f/128
  set_bit(SPCR, SPR1);  // **
  set_bit(SPCR, DORD); // LS first
  set_bit(SPCR, CPOL); // clock idle high
  set_bit(SPCR, CPHA); // trailing samle
  set_bit(SPCR, SPE); // enable SPI

}

void SPI_Disable()
{
  clr_bit(SPCR, SPE);
}


uint8_t SPI_Send_Byte(uint8_t byte)
{
  wait_high (PINB, BRXEN);

  SPI_Init_Master();
  
  SPDR = byte;
  while(!(SPSR & (1<<SPIF)))
    ;
  uint8_t ret = SPDR;
  SPI_Disable();
  _delay_us(6);
  clr_bit(DDRB, BRXEN);
  DDRB = 1<<BSRQ;
  PORTB = 0xff;
  _delay_us(440);
  return SPDR;
}

void USART_Send_Byte(uint8_t u8Data){
  // Wait until last byte has been transmitted
  while((UCSRA &(1<<UDRE)) == 0);
  // Transmit data
  UDR = u8Data;
}

void Led_Init(){
  set_bit(DDRD, DDD5);
  set_bit(DDRD, DDD4);
  clr_bit(PORTD, DDD5);
  clr_bit(PORTD, DDD4);
  //  set_bit(SFIOR, PUD);
}

void SPI_Slave_Init()
{
  PORTB = 0x00 | (1<<BSRQ);
  DDRB  = 0x00 | (1<<BSRQ);
}

uint8_t SPI_Read_Byte(void)
{
  // TODO Replace with HW implementation
  // wait for BRXEN becomes low
  wait_low (PINB, BRXEN);



  uint8_t byte = 0;
  for (int bit = 0; bit < 8; bit++)
    {
      if (bit_is_set (PINB, BRXEN))
        {
          Led_Error();
          USART_Send_Byte(0x01);
          return 0xBB;
        };
      // wait for falling clock
      wait_low (PINB, BCLK);
      // Master setup
      
      // wait for clock raising
      wait_high(PINB, BCLK);

      // changed from low to high
      // read bit value
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

void Send_Status_Not_Playing(){
  _delay_ms(interval);
  SPI_Send_Byte (0x60);
  SPI_Send_Byte (0x03);
  SPI_Send_Byte (0x18);
  if (in_recovery)
    {
      SPI_Send_Byte (0x10);
      in_recovery = 0;
    }
  else
    {
      SPI_Send_Byte(0x11);
    }
  SPI_Send_Byte (0x01);
};

void Send_Status_Playing(){
  _delay_ms(interval);
  SPI_Send_Byte(0x61);
  SPI_Send_Byte(0x0a);

  if ((! is_paused) && (secs > 60)){
    if (secs > 60){
      mins++;
      secs = 0;
    }
  }
                  
  uint8_t body[10] = {0x18,
                      0x04,
                      0xf1,
                      0x01,
                      mins,
                      secs,
                      0x00,
                      0x3f,
                      0x03,
                      0x00};
  // Original CD changer sends words groupped by 3
  // for example
  // W - 0.5ms - W - 0.5ms - W ---------- 6ms ------------- W - 0.5ms - ....
  // emulate it here
  for (int i = 0; i < 10; i++){
    SPI_Send_Byte(body[i]);
    switch (i)
      {
      case 1 | 4 | 7  :
        _delay_ms(5);
        break;
      default:
        break;
      }
  };
}

void Send_Status()
{
  if (is_playing)
    {
      Send_Status_Playing();
      return;
    };
  Send_Status_Not_Playing();
}

void Process_Message()
{
            set_bit(PORTB, BSRQ);
            // We won!
            uint8_t length = SPI_Read_Byte();
            if (length > 1)
            {
                Led_Error();
                USART_Send_Byte (0x02);
                return;
            }

            if (length)
            {
                uint8_t body = SPI_Read_Byte();

                switch (body)
                {
                    case 0xF0:
                        Send_Status_Not_Playing();
                        break;
                    case 0x06:
                        is_playing = 1;
                        Send_Status();
                        break;
                    case 0x16:
                        is_playing = 0;
                        Send_Status();
                        break;
                    case 0x26:
                        track++;
                        Send_Status();
                        break;
                    case 0x27:
                        track--;
                        Send_Status();
                        break;
                    case 0x28:
                        is_random = ~is_random;
                        Send_Status();
                        break;
                    case 0x3d:
                        break;
                    case 0xff:
                        _delay_ms(interval);
                        SPI_Send_Byte (0x60);
                        SPI_Send_Byte (0x01);
                        SPI_Send_Byte (0x18);
                        break;

                    case 0x00:
                    case 0xf6:
                        Send_Status();
                        break;
                    default:
                        Led_Error();
                        USART_Send_Byte (0x03);
                        USART_Send_Byte (body);
                        USART_Send_Byte (0x03);
                        Send_Status();
                        break;
                }
            }
            else
            {
                _delay_ms(interval);
                SPI_Send_Byte (0x60);
                SPI_Send_Byte (0x01);
                SPI_Send_Byte (0x18);
            }
}

void SPI_Listen()
{

    for (;;)
    {
      // read address
        uint8_t byte = SPI_Read_Byte();
        if (byte == 0x06)
        {
            Process_Message();
        }
        else
        {
            // Skip message
            uint8_t length = SPI_Read_Byte();
            USART_Send_Byte(0x67);
	    USART_Send_Byte(length);
	    while (length--)
	    {
		    uint8_t temp = SPI_Read_Byte();
		    USART_Send_Byte(temp);
	    }
	    USART_Send_Byte(0x76);

        }
    }
}

void Send_First_SRQ(void)
{
  // wait for high RST
  wait_high (PINB, BRST);
  _delay_ms(1.5);
  clr_bit (PORTB, BSRQ);
}

void Set_SRQ_Timer(){

  // CD-Changer should send SRQ request to head unit
  // to achieve this use simple "blink on overflow"
  TCCR1B = _BV(CS11) | _BV(CS10); // prescale 64 524.288 ms 
  TIMSK = _BV(TOIE1); // Enable overflow interrupts
  
};

void Set_RST_Watchdog()
{
  TCCR2 = (_BV(CS20));
  TIMSK |= _BV(TOIE2);
}

int main(void)
{
  USART_Init();
  Led_Init();
  sei();

  SPI_Slave_Init();
  Set_SRQ_Timer();
  Set_RST_Watchdog();
  Send_First_SRQ();
  
  SPI_Listen();

  for (;;)
    {
      Led_Error();
      USART_Send_Byte (0x04);
      _delay_ms(200);
    }
}
