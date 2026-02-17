//---------------------------------------------
// instSimulator.h
//---------------------------------------------
// 2026-01-05 Adding boatActual class
//
// Inasmuch as the instrument simulator and it's instrument
// sub-implementations currently have the monitoring capabilities
// for both in and out, the initial implementation decision for
// the boatActual is that it will live under the instrument simulator
// much as the boat simulator does.
//
// I *barely* have a handle on monitoring
//
//		ST monitoring is mostly complete, minus an understanding
//			of the ST7000 that I could not spoof on the desk
//		NMEA2000 monitoring is moderately robust with most of
//			instrument types implicitly supported
//		NMEA0183 monitoring is currently "text" only, echoing
//			the text, but not parsing, the NMEA0183 messages

#pragma once
#include <myDebug.h>

// Highest level Serial Port Constant defines

#define SERIAL_ST1		Serial1
#define SERIAL_ST2		Serial2
#define SERIAL_83A 		Serial3
#define SERIAL_83B 		Serial4
#define SERIAL_ESP32	Serial5

#define GP8_FUNCTION_OFF 		0
#define GP8_FUNCTION_SPEED 		1
#define GP8_FUNCTION_WIND 		2
#define GP8_FUNCTION_ESP32 		3

#define PIN_SPEED_PULSE	 		2	// GREEN on ST50 Speed, YELLOW on ST50 Wind

#define PIN_WIND_PWMA			11		// GREEN
#define PIN_WIND_PWMB			12		// BLUE
#define PIN_UDP_ENABLE			13

#define ST50_WIND_CALIB			1
#define ST50_WIND_CIRCLE		2
	// non-zero values for m_st50_wind_circle "modes"


//----------------------------------
// constant defines
//----------------------------------

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

#define MON2000_SENSORS			0x0001		// sensors out, known messages in
#define MON2000_AIS_GPS			0x0002		// GPS/AIS specifically
#define MON2000_PROPRIETARY		0x0004		// known proprietary in
#define MON2000_UNKNOWN			0x0008 		// unknown (not sensors, known proprietary, or known bus) in
#define MON2000_BUS_IN			0x0010		// known BUS in
#define MON2000_BUS_OUT			0x0020		// known BUS out

#define MON2000_SELF			0x1000		// self (sent) as well as received
#define MON2000_RAW				0x8000		// show raw "instrument" messages


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
// simulated instruments
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
	void setFWD(uint8_t fwd);
	void setMonitor(int port, uint32_t value);
	void clearState();
		// clears all instruments, forwarding, monitoring, debugging, GP8 function
		// as well as boat_sim.g_MON_SIM, calls sendBinaryState() but does not
		// call saveToEEProm
		
	void saveToEEPROM();
	void loadFromEEPROM();
	void sendBinaryState();

	void setGP8Function(uint8_t fxn);
	uint8_t getGP8Function()  	{ return g_GP8_FUNCTION; }
	
	void setRawST50TestMode(bool raw_mode);
	int getRawST50TestMode()			{ return m_ST50_raw_mode; }

	void setRawPulseHz(float hz);
	float getRawPulseHz()  	{ return m_raw_hz; }
	
	void setRawPWMDuty(bool pwm_b, uint8_t duty);	// 0..255
	uint8_t getRawPWMDuty(bool pwm_b) { return pwm_b ? m_raw_pwmB : m_raw_pwmA; }

	void doST50WindCircle(int how);	// 0=off, 1=ST50_WIND_CALIB, 2=ST50_WIND_CIRCLE
		// 1=calibration, 2=measurement

	bool doTbEsp32();

	static uint8_t g_FWD;
	static uint32_t g_MON[NUM_PORTS];

	instBase *getInst(int i)  { return i<NUM_INSTRUMENTS ? m_inst[i] : 0; }

private:

	instBase *m_inst[NUM_INSTRUMENTS];

	static uint8_t g_GP8_FUNCTION;

	bool  m_ST50_raw_mode;			// true => use sim for speed pulses & pwm

	float m_raw_hz;					// user provided hertz
	float m_pulse_hz;				// hertz actual setting
	bool m_pulse_timer_running;		// if using the pulse timer

	int m_raw_pwmA;					// user provided 0..255
	int m_raw_pwmB;
	int m_wind_pwmA;				// actual setting 0..255
	int m_wind_pwmB;
	int m_last_pwmA;				// previous setting for change detection: -1..255
	int m_last_pwmB;

	int m_st50_wind_circle;			// sub_mode for doing automatic wind circles in GP8 WIND mode
		// 0=off,
		// 1=do 2 degrees per 250ms (45s total) "calibration" circle
		// 2=do 15 degrees per 15 second (360s total) "measurement" circle
	uint32_t m_circle_delay;		// 250 or 15000 ms depending on circle type
	int	m_circle_increment;			// 2 or 15 degrees


	void initST50Testing();
	void initSpeedPulse();
	void doST50Testing();

};	// class instSimulator



extern instSimulator inst_sim;



// end of instSimulator.h
