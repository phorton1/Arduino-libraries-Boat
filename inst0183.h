//--------------------------------------
// inst0183.h
//--------------------------------------
// contains defines and externs specific to NMEA0183
//
// NOTE NOTE NOTE I gotta put this someplace:  NOTE NOTE NOTE
//
// The default configuration of the E80 and the GX2410 are such that
// when connected directly together (or through my system with forwarding)
// the E80 will "kill" the GX2410's internal GPS as soon as it sends
// (an echo really) GGA, GLL, or RMC, which is doubly annoying because
// the GX2410 will send out a GGA/GLL about once a minute as it tries
// to hook up and is immediately killed by the E80, and, since the E80
// times out in about 30 seconds, it leads to an endless loop of BEEPING.
//
// IF YOU WANT THE GX2410 TO REALLY BE THE GPS MASTER, not only do you
// need to turn off Seatalk GPS, but you also need to TURN OFF THE E80
// transmission of GGA, GLL, and RMS, which, sigh can only be done by
// going to the SystemSetup-SystemIntegration dialog and FIRST changing
// the baud rate to 4800, then changing the NMEA Output Settings, and
// then changing the baud rate back to 38400.
//
// Hence the method GGA() that allows adding a
// checkbox for this filter to keep from going crazy, but when they are
// wired together, forget it, you'd need to change the E80 config.


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

extern void setE80Filter(bool value);
extern bool getE80Filter();
	// stop GX2410 killer GGA,GLL, and RMC messages
	// from being forwarded from 83A->83B


// end of inst0183.h
