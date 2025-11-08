//---------------------------------------------
// instSimulator.h
//---------------------------------------------

#pragma once
#include <myDebug.h>

#define BREADBOARD	1

#if BREADBOARD
	#define SERIAL_83A 	Serial3		// NMEA0183-1
	#define SERIAL_83B 	Serial4		// NMEA0183-2
	#define SERIAL_ST	Serial2
#else
	#define SERIAL_83A 	Serial3		// NMEA0183-1
	#define SERIAL_83B 	Serial2		// NMEA0183-2
	#define SERIAL_ST	Serial4
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

#define MAX_INST_NAME		10

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


#define PORT_ST				0
#define PORT_83A			1
#define PORT_83B			2
#define PORT_2000			3
#define NUM_PORTS			4

#define FWD_A_TO_B			1
#define FWD_B_TO_A			2


#define FEET_TO_METERS		0.3048
#define NM_TO_METERS		1852.0
#define GALLON_TO_LITRE		3.785
#define PSI_TO_PASCAL		6895.0
#define SECONDS_PER_DAY		86400


// g_MON[PORT_83x) is bitwise

#define MON83_ALL		0x01 		// all in/out
#define MON83_AIS_IN	0x02 		// ais in only


// g_MON[PORT_2000] is bitwise

#define MON2000_SENSORS			0x01		// sensors out, known messages in
#define MON2000_AIS_GPS			0x02      // GPS/AIS specifically
#define MON2000_PROPRIETARY		0x04      // known proprietary in
#define MON2000_UNKNOWN			0x08      // unknown (not busi.e. proprietary) in
#define MON2000_BUS_IN			0x10      // BUS in
#define MON2000_BUS_OUT			0x20      // BUS out



//-------------------------------
// instBase
//-------------------------------

class instBase
{
public:

	instBase() 	{ m_ports = 0; }

	virtual const char *getName() = 0;

	uint8_t getPorts()					{ return m_ports; }
	void setPorts(uint16_t port_mask) 	{ m_ports = port_mask;}

	bool portActive(int port_num) 		{
		uint8_t port_mask = (1 << port_num);
		return m_ports & port_mask; }

	virtual void sendSeatalk() {};
	virtual void send0183(bool portB) {};
	virtual void send2000() {};

protected:

	uint8_t m_ports;

};	// class instBase



//--------------------------------
// instruments
//--------------------------------

#define DEFINE_INST_CLASS(CLASSNAME, NAMESTR) \
	class CLASSNAME : public instBase { \
	public: \
		CLASSNAME() : instBase() {} \
	private: \
		virtual const char* getName() override { return NAMESTR; } \
		virtual void sendSeatalk() override; \
		virtual void send0183(bool portB) override; \
		virtual void send2000() override; \
	};

DEFINE_INST_CLASS(depthInst,    "DEPTH"		)
DEFINE_INST_CLASS(logInst,      "LOG"		)
DEFINE_INST_CLASS(windInst,     "WIND"		)
DEFINE_INST_CLASS(compassInst,  "COMPASS"	)
DEFINE_INST_CLASS(gpsInst,      "GPS"		)
DEFINE_INST_CLASS(aisInst,      "AIS"		)
DEFINE_INST_CLASS(apInst,  		"AUTOPILOT" )
DEFINE_INST_CLASS(engInst,		"ENGINE"	)
DEFINE_INST_CLASS(genInst,		"GENSET"	)



//-------------------------------
// instSimulator
//-------------------------------

class instSimulator
{
public:
	
	void init();
	void run();

	void setPorts(int inst_num, uint8_t port_mask, bool no_echo);
	void setAll(int port_num, bool on, bool no_echo);
	void saveToEEPROM();
	void loadFromEEPROM();
	void sendBinaryState();

	static uint8_t g_FWD;
	static uint8_t g_MON[NUM_PORTS];

	instBase *getInst(int i)  { return i<NUM_INSTRUMENTS ? m_inst[i] : 0; }

private:

	instBase *m_inst[NUM_INSTRUMENTS];
	
};	// class instSimulator



extern instSimulator instruments;


// end of instSimulator.h
