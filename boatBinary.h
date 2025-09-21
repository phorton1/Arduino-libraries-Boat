//---------------------------------------
// boatBinary.h
//---------------------------------------
// Implements binary output to teensyBoat.pm

#pragma once

#include <Arduino.h>


#define BINARY_FLAG			0x02

#define BINARY_TYPE_PROG	0x0001
#define BINARY_TYPE_BOAT	0x0002
#define BINARY_TYPE_ST		0x0004
#define BINARY_TYPE_0183	0x0008

#define NO_ECHO_TO_PERL		10000



#define BINARY_HEADER_LEN	3



extern uint16_t g_BINARY;

extern int binaryBool		(uint8_t *buf, int offset, bool b);
extern int binaryChar		(uint8_t *buf, int offset, char c);
extern int binaryUint8		(uint8_t *buf, int offset, uint8_t u);

extern int binaryInt16		(uint8_t *buf, int offset, int16_t i);
extern int binaryUint16		(uint8_t *buf, int offset, uint16_t u);

extern int binaryInt32		(uint8_t *buf, int offset, int32_t i);
extern int binaryUint32		(uint8_t *buf, int offset, uint32_t u);
extern int binaryFloat		(uint8_t *buf, int offset, float f);		// 32 bits on teensy

extern int binaryDouble		(uint8_t *buf, int offset, double d);		// 64 bits on teensy

extern int binaryVarStr		(uint8_t *buf, int offset, const char *s, int buf_size);
extern int binaryFixStr		(uint8_t *buf, int offset, const char *s, int fixed_len);

extern int startBinary		(uint8_t *buf, uint16_t type);
extern void endBinary		(uint8_t *buf, int offset);


