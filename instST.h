//--------------------------------------
// instST.h
//--------------------------------------
// contains defines and externs specific to Seatalk

#pragma once

#include <Arduino.h>


#define ST_DEPTH		0x100
#define ST_RPM			0x105
#define ST_WIND_ANGLE	0x110
#define ST_WIND_SPEED	0x111
#define ST_WATER_SPEED	0x120
#define ST_SOG			0x152
#define ST_COG			0x153
#define ST_TIME			0x154
#define ST_DATE			0x156
#define ST_LATLON		0x158
#define ST_HEADING		0x189

#define ST_TARGET_NAME	0x182
#define ST_NAV_TO_WP	0x185
#define ST_ARRIVAL		0x1A2


// in instSTIn

extern bool g_MON_ST;
extern uint32_t g_last_st_receive_time;
extern void showDatagram(const uint8_t *datagram);
	// in decode.cpp



// end of instST.h
