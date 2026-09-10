#include "config.h"
#include "protocol.h"
#include "mbus.h"
#include "changer.h"
#include "usart.h"

#include <avr/interrupt.h>

static void Mbus_Listen(void)
{
  for (;;)
    {
      uint8_t address = Mbus_ReadByte();
      if (address == MSG_TO_CD)
        {
          Changer_HandleCdMessage();
        }
      else
        {
          Mbus_SkipMessage();
        }
    }
}

int main(void)
{
  USART_Init();
  Changer_Init();
  Mbus_InitHardware();
  sei();

  Mbus_SetupTimers();
  Mbus_WaitForFirstReset();

  Mbus_Listen();

  return 0;
}
