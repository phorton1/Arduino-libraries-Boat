//--------------------------------------
// instST.h
//--------------------------------------
// contains defines and externs specific to Seatalk

#pragma once

#include <Arduino.h>

#define MAX_ST_BUF		20			// size of my maximum datagram buffers
#define MAX_ST_SEEN		11			// space in the UI for long messages

#define ST_COMMAND_BIT	0x100		// the 0th byte of 9bit ST commands have this
#define ST_QUIET_BIT	0x200		// this is a flag I add to NOT echo the command while forwarding

//                                                      not
// Received by/from					// sent				parsed	0183_sim

#define ST_DEPTH			0x100		// depthInst(x)				recd
#define ST_RPM				0x105		// engInst(x)
#define ST_WIND_ANGLE		0x110		// windInst(x)				recd
#define ST_WIND_SPEED		0x111		// windInst(x)				recd
#define ST_WATER_SPEED		0x120		// logInst(x)				recd
#define ST_TRIP				0x121		// logInst(0)
#define ST_LOG_TOTAL		0x122		// logInst(0)
#define ST_WATER_TEMPR		0x123		//
#define ST_DISP_UNITS		0x124		//
#define ST_TRIP_TOTAL		0x125		// logInst(1)
#define ST_LOG_SPEED		0x126		// 							recd
#define ST_WATER_CELSIUS	0x127		//
#define ST_LAMP_INTENSITY	0x130		//
#define ST_LAT				0x150		// 							recd
#define ST_LON				0x151		// 							recd
#define ST_SOG				0x152		// gpsInst(1)				recd
#define ST_COG				0x153		// gpsInst(1)				recd
#define ST_TIME				0x154		// gpsInst(1)				recd
#define ST_DATE				0x156		// gpsInst(1)				recd
#define ST_SAT_INFO			0x157		// gpsInst(x)				recd
#define ST_LATLON			0x158		// gpsInst(x)				recd
#define ST_59				0x159		// 					o		recd
#define ST_E80_SIG			0x161		// 					o		recd
#define ST_TARGET_NAME		0x182		// apInst(1)				recd
#define ST_AUTOPILOT		0x184		// apInst(1)				recd from AP cpu
#define ST_NAV_TO_WP		0x185		// apInst(1)				recd
#define ST_AP_KEYSTROKE		0x186		//							recd from ST7000
#define ST_HEADING			0x189		// compassInst(x)			recd
#define ST_ST7000			0x197		//							recd from ST7000 error/query mode
#define ST_AP_CPU			0x198		// apInst(1)				recd from AP cpu in response to ST7000
#define ST_COMPASS_VAR		0x199		// 							recd
#define ST_RUDDER			0x19C		// apInst(1+1)				recd from AP cpu
#define ST_ARRIVAL			0x1A2		// apInst(1)				recd
#define ST_WP_DEF			0x19E		// 					o		got it once, could be bogus bytes
#define ST_DEV_QUERY		0x1A4		// 					o		we don't respond (yet)
#define ST_SAT_DETAIL		0x1A5		// gpsInst(0)		o
#define ST_DIF_DETAIL		0x1A7		// 					o
#define ST_AD				0x1AD		// 					o		unknown

// in instST_out

extern void queueDatagram(bool port2, const uint16_t *dg);
extern void queueDatagram8(bool port2, const uint8_t *dg, bool quiet);
extern bool sendDatagram(bool port2);
extern void clearSTQueues();
extern void setLampIntensity(int value);	// sent to all ports, 0==off, 1=low, 2=medium, 3=high

// support for neo6M_GPS ST test implementation

#define PRN_STATE_USED		2		// used in solution
#define PRN_STATE_TRACKED	1		// tracked
#define PRN_STATE_NONE		0		// "search"

#define ST_MAX_VIEW			11		// max number of reported sats
#define ST_MAX_TRACKED		9		// max number of tracked/used sats
#define ST_TRACK_CUTOFF  	4		// number kept in A5 0C 2nd series message

extern void initStSatMessages();
extern void sendStSatMessags(bool port2);
extern void addStSatMessage(uint8_t prn, uint8_t ele, uint16_t az, uint8_t snr, uint8_t prn_state);


// in instSTIn

extern void showDatagram(bool port2, bool out, const uint8_t *datagram);
extern void showDatagram16(bool port2, bool out, const uint16_t *dg);

extern volatile int ap_linked;
	// global variable for linking datagrams received in instST_in.cpp to
	// the apInst() in instST_out.cpp for nascent, optional, emulation
	// of ST7000<->ap-cpu communications, which *may* grow to include
	// calibration  mode, etc for further testing of the ST7000
extern volatile bool st_device_query_pending;
	// Set when the system receives the ST_DEV_QUERY
	//		0x1a4 06 00 00 00 00 00 00 00 device
	// device query from the E80, in which case, ST specific code
	// sends out 0x1a4 12 ID VV vv replies and clears the boolean



// end of instST.h
