//--------------------------------------
// instST.h
//--------------------------------------
// contains defines and externs specific to Seatalk

#pragma once

#include <Arduino.h>

#define MAX_ST_BUF		20			// size of my maximum datagram buffers
#define MAX_ST_SEEN		9			// largest ST message I've seen


// Received by/from												0183_sim	E80_idle	notes

#define ST_DEPTH		0x100		// sent by instDepth		recd
#define ST_RPM			0x105		// sent by instEngine
#define ST_WIND_ANGLE	0x110		// sent	by instWind			recd					apparent
#define ST_WIND_SPEED	0x111		// sent	by instWind			recd					apparent
#define ST_WATER_SPEED	0x120		// sent by instLog			recd
#define ST_LOG_SPEED	0x126		//							recd
#define ST_LAT			0x150		//							recd
#define ST_LON			0x151		// 							recd
#define ST_SOG			0x152		// sent	by instLog			recd
#define ST_COG			0x153		// sent	by instCompass		recd
#define ST_TIME			0x154		// sent by instGPS			recd
#define ST_DATE			0x156		// sent by instGPS			recd
#define ST_SAT_INFO		0x157		// 							recd
#define ST_LATLON		0x158		// sent by instGPS			recd
#define ST_59			0x159		//							recd		idle
#define ST_E80_SIG		0x161		//							recd		idle
#define ST_TARGET_NAME	0x182		// sent by instAutopilot	recd
#define ST_NAV_TO_WP	0x185		// sent by instAutopilot	recd
#define ST_HEADING		0x189		// sent by instCompass		recd
#define ST_COMPASS_VAR	0x199		//							recd		idle
#define ST_ARRIVAL		0x1A2		// sent	by instAutopilot	recd

#define ST_WP_DEF		0x19E		// got it once, could be bogus bytes
#define ST_DEV_QUERY	0x1A4		// we don't respond (yet)
#define ST_SAT_DETAIL	0x1A5
#define ST_A7			0x1A7		// some kind of gps message
#define ST_AD			0x1AD		// unknown

// in instSTIn

extern bool g_MON_ST;

extern uint32_t g_last_st_receive_time;
extern void showDatagram(bool out, const uint8_t *datagram);
	// in decode.cpp



// end of instST.h
