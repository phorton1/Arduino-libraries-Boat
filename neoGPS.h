//----------------------------------------------
// neoGPS.h
//----------------------------------------------
// replace much of complexity of Boat library
// by not including instSimulator.h or boatSimulator.h

#pragma once

extern void initNeo6M_GPS();
extern void doNeo6M_GPS();

extern void enableNeoSeatalk(bool enable);
extern void enableNeoNMEA200(bool enable);
extern bool NeoSeatalkEnabled();
extern bool NeoNMEA2000Enabled();



