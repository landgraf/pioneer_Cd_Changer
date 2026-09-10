#ifndef PROTOCOL_H
#define PROTOCOL_H

// Pioneer M-bus message types (first byte of every message).
// Full message layouts are documented in `protocoldecode`.
#define MSG_STATUS_OFF       0x00
#define MSG_HEAD_UNIT_STATUS 0x01
#define MSG_TO_TAPE          0x03
#define MSG_TO_CD            0x06
#define MSG_TO_DASHBOARD     0x21
#define MSG_FROM_CD_STOPPED  0x60
#define MSG_FROM_CD_PLAYING  0x61
#define MSG_FROM_TUNER       0x71

// Commands carried in the body of a MSG_TO_CD message.
#define CD_CMD_STATUS_REQUEST 0x00
#define CD_CMD_PLAY            0x06
#define CD_CMD_STOP             0x16
#define CD_CMD_TRACK_NEXT      0x26
#define CD_CMD_TRACK_PREV      0x27
#define CD_CMD_RANDOM_TOGGLE   0x28
#define CD_CMD_DISC_1           0x31
#define CD_CMD_DISC_2           0x32
#define CD_CMD_DISC_3           0x33
#define CD_CMD_DISC_4           0x34
#define CD_CMD_DISC_5           0x35
#define CD_CMD_DISC_6           0x36
// Sent once at startup only; meaning not fully reverse engineered.
#define CD_CMD_STARTUP_F0        0xf0
#define CD_CMD_STARTUP_F6        0xf6
#define CD_CMD_RESET              0xff

// Flag/ack bytes used in status replies. The exact bit layout isn't fully
// reverse engineered (see `protocoldecode`), so these are kept as opaque
// values, same as the original firmware.
#define CD_RESP_FLAGS_MAGAZINE 0x18
#define CD_RESP_ACK_RECOVERED  0x10
#define CD_RESP_ACK_NORMAL     0x11

#endif
