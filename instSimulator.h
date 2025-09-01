//---------------------------------------------
// instSimulator.h
//---------------------------------------------

#pragma once
#include "boatSimulator.h"
#include <NMEA2000.h>
#include <myDebug.h>

#define SERIAL_VHF_0183 Serial2
#define SERIAL_SEATALK	Serial3
#define SERIAL_E80_0183	Serial4

// Teensy Pins Used
//
// 23 - CRX from CANBUS module
// 22 - CTX to CANBUS module
// 7  - RX2
// 8  - TX2
// 15 - RX3 from seatalk opto isolator circuit
// 14 - TX3 to seatalk opto isolator circuit
// 16 - RX4
// 17 - TX4

#define INST_DEPTH			0
#define INST_LOG			1
#define INST_WIND			2
#define INST_COMPASS		3
#define INST_GPS			4
#define INST_AUTOPILOT		5
#define INST_ENGINE 		6
#define INST_GENSET			7

#define NUM_INSTRUMENTS 	8

#define PROTOCOL_NONE 		0x00
#define PROTOCOL_SEATALK	0x01
#define PROTOCOL_0183		0x02
#define PROTOCOL_2000		0x04
#define PROTOCOL_ALL		0x07


//-------------------------------
// instBase
//-------------------------------

class instBase
{
public:

	instBase(uint8_t supported) :
		m_supported(supported)
	{
		m_protocols = supported;	// 0;
	}

	virtual const char *getName() = 0;

	bool supported(uint8_t protocol) {
		return m_supported & protocol; }
	void setProtocol(uint8_t protocol, bool on) {
		m_protocols &= ~protocol;
		if (on) m_protocols |= protocol; }
	bool doProtocol(uint8_t protocol) {
		return (m_supported & protocol) &&
			   (m_protocols & protocol); }

	virtual void sendSeatalk() {};
	virtual void send0183() {};
	virtual void send2000(tNMEA2000 *nmea2000) {};

protected:

	uint8_t m_supported;
	uint8_t m_protocols;

};	// class instBase


//--------------------------------
// instruments
//--------------------------------

#define DEFINE_INST_CLASS(CLASSNAME, NAMESTR, SUPPORTED) \
	class CLASSNAME : public instBase { \
	public: \
		CLASSNAME() : instBase(SUPPORTED) {} \
	private: \
		virtual const char* getName() override { return NAMESTR; } \
		virtual void sendSeatalk() override; \
		virtual void send0183() override; \
		virtual void send2000(tNMEA2000 *nmea2000) override; \
	};

DEFINE_INST_CLASS(depthInst,      "DEPTH",		PROTOCOL_0183)
DEFINE_INST_CLASS(logInst,        "LOG",		PROTOCOL_ALL)
DEFINE_INST_CLASS(windInst,       "WIND",		PROTOCOL_ALL)
DEFINE_INST_CLASS(compassInst,    "COMPASS",	PROTOCOL_ALL)
DEFINE_INST_CLASS(gpsInst,        "GPS",		PROTOCOL_ALL)
DEFINE_INST_CLASS(autopilotInst,  "AUTOPILOT",	PROTOCOL_ALL)
DEFINE_INST_CLASS(engineInst,     "ENGINE",		PROTOCOL_0183 | PROTOCOL_2000)
DEFINE_INST_CLASS(gensetInst,     "GENSET",		PROTOCOL_0183 | PROTOCOL_2000)



//-------------------------------
// instSimulator
//-------------------------------

class instSimulator
{
public:
	
	void init(tNMEA2000 *nmea2000);
	void run();

	void setProtocol(int inst_num, uint8_t protocol, bool on)
	{
		instBase *inst = m_inst[inst_num];
		if (on && !inst->supported(protocol))
			my_error("request for unsupported protocol(%d) for %s instrument",
				protocol,inst->getName());
		else
			m_inst[inst_num]->setProtocol(protocol,on);
	}

private:

	tNMEA2000 *m_nmea2000;

	instBase *m_inst[NUM_INSTRUMENTS];
	
};	// class instSimulator



extern instSimulator instruments;


// end of instSimulator.h
