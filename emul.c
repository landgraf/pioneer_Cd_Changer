#define F_CPU 8000000UL  // 8 MHz
#define TIMER_FREQ     44100

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//#define send_SPI(byte) wait_SPI; 
#define set_bit(reg, bit)  reg |= (1<<(bit))    // Установка бита.
#define clr_bit(reg, bit)   reg &= (~(1<<(bit))) // Сброс бита.
#define switch_bit(reg, bit)  reg ^= (1<<(bit)) // Переключение бита.

// looping when bit is 1
//#define wait_low(reg, bit) while(reg & (1<<bit)){}
#define wait_low(reg, bit) while(bit_is_set(reg,bit)){}
// looping when bit is 0
#define wait_high(reg, bit) while(! bit_is_set(reg,bit)){}

void SPI_Init_Master()
{
  PORTB = 0xff;
  DDRB = ((1<<DDB4)|(1<<DDB5)|(1<<DDB6)|(1<<DDB7)); //spi pins on port b MOSI SCK,SS outputs
  clr_bit(PORTB, 4);
  _delay_us(50);
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

void SPI_Switch_Slave()
{
  DDRB = ((1<<DDB6));
  SPCR = 0x00;
  set_bit(SPCR, DORD); // LS first
  set_bit(SPCR, CPOL); // clock idle high
  set_bit(SPCR, CPHA); // trailing samle
  set_bit(SPCR, SPE); // enable SPI
}

void SPI_Disable()
{
  clr_bit(SPCR, SPE);
}

uint8_t SPI_Send(uint8_t byte)
{
  SPI_Init_Master();
  
  SPDR = byte;
  while(!(SPSR & (1<<SPIF)))
    ;
  uint8_t ret = SPDR;
  SPI_Disable();
  _delay_us(200);  
  set_bit(PORTB, 4);
  SPI_Switch_Slave();
  return SPDR;
}

void Set_Init_SRQ()
{
  clr_bit(PORTB,3);
};

void main(void)
{

  for (;;)
  {
    _delay_ms(100);
    SPI_Send(0x18);
  }
  
}
