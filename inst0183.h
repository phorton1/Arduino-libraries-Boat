//--------------------------------------
// inst0183.h
//--------------------------------------
// contains defines and externs specific to NMEA0183

#pragma once

#include <Arduino.h>

extern void handleNMEA0183Input(bool portB, const char *);

// Devices
// depthInst  		SD = Sounder device
// logInst			VW = Velocity Sensor, Speed Log, Water, Mechanical
// windInst			WI = Weather Instruments
// compassInst		HC = Heading Compass
// gpsInst			GP = GPS
// apInst			AP = Autopilot device
// aisInst			unused
// engInst			unused
// genInst			unused


// D=decoded, x=invariant, 0/1 = optional
//
// SDDPT - Water Depth					x		depthInst
// VWVHW - Speed through water			x		logInst
// VWVLW - Distance Traveled			1   	logInst
// WIMWV - Wind Speed and Angle			x+x 	windInst
// HCHDT - Heading True					1		compassInst
// HCMDM - Heading Magnetic				0		compassInst
// GPGGA - GP Fix Data					1		gpsInst
// GPGSA - GNSS DOP and Sats			1		gpsInst
// GPGSV - Satellites in view			1		gpsInst
// GPRMC - Minimum NavInfo C			1		gpsInst
// GPGLL - Geographic Lat/Lon			1		gpsInst
// APVTG - Velocity/Track over Ground	x		apInst
// APRMC - Minimum NavInfo C			x		apInst
// APRMB - Minimum NavInfo B			x		apInst(routing)
// APBWC - Bearing/Distance to WP		x		apInst(routing)
// xxVDM - VHF Data Link Message		D




// end of inst0183.h
