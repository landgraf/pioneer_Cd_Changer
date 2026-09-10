// CD-changer state machine and message content. Knows the M-bus
// protocol semantics, delegates actual byte transport to mbus.h.

#include "changer.h"
#include "config.h"
#include "protocol.h"
#include "mbus.h"

#include <util/delay.h>

typedef enum
{
  CHANGER_PLAYING  = 1 << 0,
  CHANGER_PAUSED   = 1 << 1,
  CHANGER_RANDOM   = 1 << 2,
  CHANGER_RECOVERY = 1 << 3,
} changer_flag_t;

static volatile uint8_t state = 0;
static uint8_t disk = 1;
static uint8_t track = 1;
static uint8_t mins = 0;
static volatile uint8_t secs = 0;

static inline uint8_t is_playing(void) { return state & CHANGER_PLAYING; }
static inline uint8_t is_paused(void)  { return state & CHANGER_PAUSED; }

static inline uint8_t to_bcd(uint8_t value)
{
  return (uint8_t)(((value / 10) << 4) | (value % 10));
}

void Changer_Init(void)
{
  state = 0;
  disk = 1;
  track = 1;
  mins = 0;
  secs = 0;
}

void Changer_Tick(void)
{
  secs++;
}

void Changer_EnterRecovery(void)
{
  state |= CHANGER_RECOVERY;
}

uint8_t Changer_InRecovery(void)
{
  return state & CHANGER_RECOVERY;
}

static void Send_Ack(void)
{
  _delay_ms(REPLY_DELAY_MS);
  Mbus_SendByte(MSG_FROM_CD_STOPPED);
  Mbus_SendByte(0x01);
  Mbus_SendByte(CD_RESP_FLAGS_MAGAZINE);
}

static void Send_Status_Not_Playing(void)
{
  _delay_ms(REPLY_DELAY_MS);
  Mbus_SendByte(MSG_FROM_CD_STOPPED);
  Mbus_SendByte(0x03);
  Mbus_SendByte(CD_RESP_FLAGS_MAGAZINE);
  if (Changer_InRecovery())
    {
      Mbus_SendByte(CD_RESP_ACK_RECOVERED);
      state &= ~CHANGER_RECOVERY;
    }
  else
    {
      Mbus_SendByte(CD_RESP_ACK_NORMAL);
    }
  Mbus_SendByte(0x01);
}

static void Send_Status_Playing(void)
{
  _delay_ms(REPLY_DELAY_MS);
  Mbus_SendByte(MSG_FROM_CD_PLAYING);
  Mbus_SendByte(0x0a);

  if (!is_paused() && secs >= 60)
    {
      mins++;
      secs -= 60;
    }

  uint8_t body[10] = {
    CD_RESP_FLAGS_MAGAZINE,
    0x04,
    0xf0 | (disk & 0x0f),
    to_bcd(mins),
    to_bcd(secs),
    to_bcd(track),
    0x00,
    0x3f,
    0x3f,
    0x00,
  };

  // Original CD changer sends words grouped by 3:
  //   W - 0.5ms - W - 0.5ms - W ---------- 6ms ------------- W - 0.5ms - ...
  // that finer pacing isn't reproduced here; Mbus_SendByte's own delay
  // is close enough in practice.
  for (uint8_t i = 0; i < 10; i++)
    {
      Mbus_SendByte(body[i]);
    }
}

static void Send_Status(void)
{
  if (is_playing())
    {
      Send_Status_Playing();
    }
  else
    {
      Send_Status_Not_Playing();
    }
}

static void Handle_Command(uint8_t cmd)
{
  switch (cmd)
    {
    case CD_CMD_STARTUP_F0:
    case CD_CMD_STATUS_REQUEST:
    case CD_CMD_STARTUP_F6:
      break;
    case CD_CMD_PLAY:
      state |= CHANGER_PLAYING;
      break;
    case CD_CMD_STOP:
      state &= ~CHANGER_PLAYING;
      break;
    case CD_CMD_TRACK_NEXT:
      if (track >= 99)
        {
          track = 1;
        }
      else
        {
          track++;
        }
      break;
    case CD_CMD_TRACK_PREV:
      if (track <= 1)
        {
          track = 99;
        }
      else
        {
          track--;
        }
      break;
    case CD_CMD_RANDOM_TOGGLE:
      state ^= CHANGER_RANDOM;
      break;
    case CD_CMD_DISC_1:
    case CD_CMD_DISC_2:
    case CD_CMD_DISC_3:
    case CD_CMD_DISC_4:
    case CD_CMD_DISC_5:
    case CD_CMD_DISC_6:
      disk = cmd & 0x0f;
      break;
    default:
      Mbus_SignalError();
      break;
    }
}

void Changer_HandleCdMessage(void)
{
  Mbus_ReleaseServiceRequest(); // We won arbitration.

  uint8_t length = Mbus_ReadByte();
  if (length > 1)
    {
      Mbus_SignalError();
      return;
    }

  if (length == 0)
    {
      Send_Ack();
      return;
    }

  uint8_t cmd = Mbus_ReadByte();
  if (cmd == CD_CMD_RESET)
    {
      Send_Ack();
      return;
    }

  Handle_Command(cmd);
  Send_Status();
}
