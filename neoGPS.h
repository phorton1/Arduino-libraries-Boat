//----------------------------------------------
// neoGPS.h
//----------------------------------------------
// replace much of complexity of Boat library
// by not including instSimulator.h or boatSimulator.h

#pragma once

extern void initNeo6M_GPS(HardwareSerial *neo_serial, uint8_t version, uint8_t subversion);
extern void doNeo6M_GPS();

extern void enableNeoSeatalk(bool enable);
extern void enableNeoNMEA200(bool enable);
extern bool NeoSeatalkEnabled();
extern bool NeoNMEA2000Enabled();



extern volatile bool st_neo_device_query_pending;
	// set by client code (instST_in.cpp or teensyGPS.ino) when a device query is
	// received, in which case, the neo will reply with a device id message

extern void replyToRestartGPSButton();
	// called by client code when a specific SAT_DETAIL message is
	// received that is transmitted by the E80 "Restart GPS Button"
