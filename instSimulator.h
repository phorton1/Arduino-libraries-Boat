//---------------------------------------------
// instSimulator.h
//---------------------------------------------

#pragma once
#include <myDebug.h>

#define BREADBOARD	1

#if BREADBOARD
	#define SERIAL_0183 	Serial4
	#define SERIAL_SEATALK	Serial3
#else
	#define SERIAL_0183 	Serial3
	#define SERIAL_SEATALK	Serial4
#endif

// Teensy Pins Used
//
// 23 - CRX from CANBUS module
// 22 - CTX to CANBUS module
// 7  - RX2
// 8  - TX2
// 15 - RX3
// 14 - TX3
// 16 - RX4
// 17 - TX4

#define INST_DEPTH			0
#define INST_LOG			1
#define INST_WIND			2
#define INST_COMPASS		3
#define INST_GPS			4
#define INST_AIS			5
#define INST_AUTOPILOT		6
#define INST_ENGINE 		7
#define INST_GENSET			8
#define NUM_INSTRUMENTS 	9


#define NUM_BOAT_PORTS		3
#define PORT_SEATALK		0
#define PORT_0183			1
#define PORT_2000			2


#define PORT_MASK_NONE 		0x00
#define PORT_MASK_SEATALK	(1 << PORT_SEATALK)
#define PORT_MASK_0183		(1 << PORT_0183)
#define PORT_MASK_2000		(1 << PORT_2000)
#define PORT_MASK_ALL		0x07




// monitor input and output

#define MONITOR_NONE		0x00
#define MONITOR_OUTPUT  	0x01		// monitor all sent message for port/protocol
#define MONITOR_SENSORS 	0x02		// monitor sensor (instrument) received messages for port/protocol
#define MONITOR_BUS	    	0x04		// monitor protcol bus (NMEA2000 only)


#define FEET_TO_METERS		0.3048
#define NM_TO_METERS		1852.0
#define GALLON_TO_LITRE		3.785
#define PSI_TO_PASCAL		6895.0



//-------------------------------
// instBase
//-------------------------------

class instBase
{
public:

	instBase(uint8_t supported) :
		m_supported(supported)
	{
		m_ports = supported;	// 0;
	}

	virtual const char *getName() = 0;

	bool portSupported(int port_num) {
		return m_supported & (1 << port_num); }
	void activatePort(int port_num, bool on) {
		uint8_t port_mask = (1 << port_num);
		m_ports &= ~port_mask;
		if (on) m_ports |= port_mask; }
	bool portActive(int port_num) {
		uint8_t port_mask = (1 << port_num);
		return (m_supported & port_mask) &&
			   (m_ports & port_mask); }

	virtual void sendSeatalk() {};
	virtual void send0183() {};
	virtual void send2000() {};

protected:

	uint8_t m_supported;
	uint8_t m_ports;

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
		virtual void send2000() override; \
	};

DEFINE_INST_CLASS(depthInst,      "DEPTH",		PORT_MASK_ALL)
DEFINE_INST_CLASS(logInst,        "LOG",		PORT_MASK_ALL)
DEFINE_INST_CLASS(windInst,       "WIND",		PORT_MASK_ALL)
DEFINE_INST_CLASS(compassInst,    "COMPASS",	PORT_MASK_ALL)
DEFINE_INST_CLASS(gpsInst,        "GPS",		PORT_MASK_ALL)
DEFINE_INST_CLASS(aisInst,        "AIS",		PORT_MASK_0183 | PORT_MASK_2000)
DEFINE_INST_CLASS(autopilotInst,  "AUTOPILOT",	PORT_MASK_ALL)
DEFINE_INST_CLASS(engineInst,     "ENGINE",		PORT_MASK_0183 | PORT_MASK_2000)
DEFINE_INST_CLASS(gensetInst,     "GENSET",		PORT_MASK_0183 | PORT_MASK_2000)



//-------------------------------
// instSimulator
//-------------------------------

class instSimulator
{
public:
	
	void init();
	void run();

	void activatePort(int inst_num, int port_num, bool on)
	{
		instBase *inst = m_inst[inst_num];
		if (on && !inst->portSupported(port_num))
			my_error("request for unsupported port_num(%d) for %s instrument",
				port_num,inst->getName());
		else
			m_inst[inst_num]->activatePort(port_num,on);
	}


	void activateMonitor(int port_num, uint8_t what, bool on)
	{
		uint8_t port_mask = (1 << port_num);
		m_monitor[port_num] &= ~port_mask;
		if (on) m_monitor[port_num] |= port_mask;
	}

	bool monitorActive(int port_num, uint8_t what)
	{
		return m_monitor[port_num] & what;
	}


private:

	instBase *m_inst[NUM_INSTRUMENTS];
	uint8_t m_monitor[NUM_BOAT_PORTS];
	
	
};	// class instSimulator



extern instSimulator instruments;


// end of instSimulator.h
