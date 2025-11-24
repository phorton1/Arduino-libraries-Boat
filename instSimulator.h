//---------------------------------------------
// instSimulator.h
//---------------------------------------------

#pragma once
#include <myDebug.h>

#define SERIAL_ST1	Serial1
#define SERIAL_ST2	Serial2
#define SERIAL_83A 	Serial3
#define SERIAL_83B 	Serial4

#define PIN_UDP_ENABLE	4
#define SERIAL_ESP32	Serial5
	// if defined, enables whole UDP scheme
	// otherwise, no code/penalty for scheme's existance
extern bool udp_enabled;
	// in instSimulator.cpp


// Teensy Pins Used
//
// 23 - CRX from CANBUS module
// 22 - CTX to CANBUS module
// 0  - RX1 Seatalk1
// 1  = TX1 Seatalk1
// 7  - RX2 Seatalk2
// 8  - TX2 Seatalk2
// 15 - RX3 NMEA0183A
// 14 - TX3 NMEA0183A
// 16 - RX4 NMEA0183B
// 17 - TX4 NMEA0183B
// 20 - TX5 tbESP32
// 21 - RX5 tbESP32
// 9  - UDP_ENABLE


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


#define PORT_ST1			0
#define PORT_ST2			1
#define PORT_83A			2
#define PORT_83B			3
#define PORT_2000			4
#define NUM_PORTS			5

#define FWD_NONE			0x00
#define FWD_ST1_TO_2		0x01
#define FWD_ST2_TO_1		0x02
#define FWD_83A_TO_B		0x04
#define FWD_83B_TO_A		0x08
#define FWD_MAX				0x0f


#define FEET_TO_METERS		0.3048
#define NM_TO_METERS		1852.0
#define GALLON_TO_LITRE		3.785
#define PSI_TO_PASCAL		6895.0
#define SECONDS_PER_DAY		86400


// g_MON(PORT_STx) is simply on/off

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

	virtual void sendSeatalk(bool port2) {};
	virtual void send0183(bool portB) 	 {};
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
		virtual void sendSeatalk(bool port2) override; \
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

	void setPorts(int inst_num, uint8_t port_mask);
	void setAll(int port_num, bool on);
	void setFWD(int fwd);

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
