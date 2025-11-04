//--------------------------------------
// instST.h
//--------------------------------------
// contains defines and externs specific to Seatalk

#pragma once

#include <Arduino.h>

#define MAX_ST_BUF		20			// size of my maximum datagram buffers
#define MAX_ST_SEEN		9			// largest ST message I've seen

//                                                      not
// Received by/from					// sent				parsed	0183_sim

#define ST_DEPTH		0x100		// instDepth(x)				recd
#define ST_RPM			0x105		// instEngine(x)
#define ST_WIND_ANGLE	0x110		// instWind(x)				recd
#define ST_WIND_SPEED	0x111		// instWind(x)				recd
#define ST_WATER_SPEED	0x120		// instLog(x)				recd
#define ST_TRIP			0x121		// instLog(0)
#define ST_LOG_TOTAL	0x122		// instLog(0)
#define ST_TRIP_TOTAL	0x125		// instLog(1)
#define ST_LOG_SPEED	0x126		// 							recd
#define ST_LAT			0x150		// 							recd
#define ST_LON			0x151		// 							recd
#define ST_SOG			0x152		// instGPS(1)				recd
#define ST_COG			0x153		// instGPS(1)				recd
#define ST_TIME			0x154		// instGPS(1)				recd
#define ST_DATE			0x156		// instGPS(1)				recd 						
#define ST_SAT_INFO		0x157		// instGPS(x)				recd
#define ST_LATLON		0x158		// instGPS(x)				recd
#define ST_59			0x159		// 					o		recd
#define ST_E80_SIG		0x161		// 					o		recd
#define ST_TARGET_NAME	0x182		// instAP(1)				recd
#define ST_NAV_TO_WP	0x185		// instAP(1)				recd
#define ST_HEADING		0x189		// instCompass(x)			recd
#define ST_COMPASS_VAR	0x199		// 							recd
#define ST_ARRIVAL		0x1A2		// instAP(1)				recd
#define ST_WP_DEF		0x19E		// 					o		got it once, could be bogus bytes
#define ST_DEV_QUERY	0x1A4		// 					o		we don't respond (yet)
#define ST_SAT_DETAIL	0x1A5		// instGPS(0)		o
#define ST_A7			0x1A7		// 					o		some kind of gps message
#define ST_AD			0x1AD		// 					o		unknown

// in instST_out

extern void sendDatagram();
extern void clearSTQueue();

// in instSTIn

extern bool g_MON_ST;

extern void showDatagram(bool out, const uint8_t *datagram);




// end of instST.h
