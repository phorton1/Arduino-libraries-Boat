//---------------------------------------
// boatBinary.cpp
//---------------------------------------
// Implements binary output to teensyBoat.pm

#include "boatBinary.h"
#include <myDebug.h>

uint16_t g_BINARY = 0;


int binaryBool(uint8_t *buf, int offset, bool b)
{
	buf[offset] = (uint8_t) b;
	return offset + 1;
}

int binaryChar(uint8_t *buf, int offset, char c)
{
	buf[offset] = (uint8_t) c;
	return offset + 1;
}


int binaryUint8(uint8_t *buf, int offset, uint8_t u)
{
	buf[offset] = u;
	return offset + 1;
}


int binaryInt16(uint8_t *buf, int offset, int16_t i)
{
	memcpy(&buf[offset],&i,sizeof(int16_t));
	return offset + sizeof(int16_t);
}


int binaryUint16(uint8_t *buf, int offset, uint16_t u)
{
	memcpy(&buf[offset],&u,sizeof(uint16_t));
	return offset + sizeof(uint16_t);
}



int binaryInt32(uint8_t *buf, int offset, int32_t i)
{
	memcpy(&buf[offset],&i,sizeof(int32_t));
	return offset + sizeof(int32_t);
}


int binaryUint32(uint8_t *buf, int offset, uint32_t u)
{
	memcpy(&buf[offset],&u,sizeof(uint32_t));
	return offset + sizeof(uint32_t);
}


int binaryFloat(uint8_t *buf, int offset, float f)
{
	memcpy(&buf[offset],&f,sizeof(float));
	return offset + sizeof(float);
}


int binaryDouble(uint8_t *buf, int offset, double d)
{
	memcpy(&buf[offset],&d,sizeof(double));
	return offset + sizeof(double);
}


int binaryVarStr(uint8_t *buf, int offset, const char *s, int buf_size)
{
	uint16_t len = strlen(s);
	if (len > buf_size-2)
		len = buf_size-2;
	memcpy(&buf[offset],&len,sizeof(uint16_t));
	offset += sizeof(uint16_t);
	memcpy(&buf[offset],s,len);
	offset += len;
	return offset;
}


int binaryFixStr(uint8_t *buf, int offset, const char *s, int fixed_len)
{
	int len = strlen(s);
	if (len > 255) len = 255;
	if (len > fixed_len) len = fixed_len;
	buf[offset] = (uint8_t) len;
	memset(&buf[offset+1],0,fixed_len);
	memcpy(&buf[offset+1],s,len);
	return offset + fixed_len + 1;
}




int startBinary(uint8_t *buf, uint16_t type)
{
	buf[0] = BINARY_FLAG;
	buf[1] = 0;				// two bytes for little endian uint16_t length
	buf[2] = 0;
	return binaryUint16(buf,3,type);
}


void endBinary(uint8_t *buf, int offset)
{
	// the length uint16_t does not include the flag or length itself
	binaryUint16(buf,1,offset-3);
}


