//--------------------------------------
// inst0183.h
//--------------------------------------
// contains defines and externs specific to NMEA0183

#pragma once

// in inst0183_in.cpp

extern bool g_MON_0183;
extern bool g_MON_AIS;

extern void handleNMEA0183Input(const char *);


// end of inst0183.h
