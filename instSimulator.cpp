//---------------------------------------------
// instSimulator.cpp
//---------------------------------------------
// I *think* the NMEA2000 library handles retries, and can be sent all instruments once per second.
// NMEA0183 is a dedicated one-direction port, and so shouild be able to handle all instruments once per second.
// Seatalk, on the other hand, has to monitor the bus for an idle time, abort any garbled messages, and do
// readbacks of each character it sends for collision detection, and so does not lend itself to sending all
// the messages blindly once per second.
//
// Therefore some kind of a queue and/or retry mechism is needed with Seatalk.
// Furthermore, this means that Seatalk monitoring has to take place in this
// layer, and not in the application (teensyBoat) layer, which then, in turn
// makes me want to create a consistent scheme that works across all three protocols.
// In otherwords, once per second, we ENQUEUE all the mssags for all the instruments,
// but run() needs to be called more often, and we DEQUEUE and send them while also
// monitoring.
//
// In other words, ALL OF THE PROTOCOL implementations should be in the instSimulator
// layer. I hardly know where to begin.

#include "instSimulator.h"
#include "boatSimulator.h"
#include "inst2000.h"
#include "inst0183.h"
#include "instST.h"
#include "boatBinary.h"
#include <EEPROM.h>
#include <myDebug.h>


#define UPDATE_MILLIS	1000

#define BROADCAST_NMEA2000_INFO   1

#define ST_IDLE_BUS_MS				10		// ms bus must be idle to send next datagram
#define ST_SEND_INTERVAL			10


instSimulator inst_sim;
	// global instance

depthInst		i_depth;
logInst			i_log;
windInst		i_wind;
compassInst		i_compass;
gpsInst			i_gps;
aisInst			i_ais;
apInst			i_autopilot;
engInst			i_engine;
genInst			i_genset;

uint32_t instSimulator::g_MON[NUM_PORTS];
uint8_t  instSimulator::g_FWD;
uint8_t  instSimulator::g_GP8_FUNCTION;


//-------------------------------------------------
// General Purpose Connector related
//-------------------------------------------------

static bool udp_enabled;
	// turned on only if PIN_UDP_ENABLE pulled high by ESP32


void instSimulator::setGP8Function(uint8_t fxn)
{
	// if (g_GP8_FUNCTION != fxn)
	{
		display(0,"setGPSFunction(%d)",fxn);

		// turn off ESP32 Serial Port if it is on

		if (g_GP8_FUNCTION == GP8_FUNCTION_ESP32)
			SERIAL_ESP32.end();

		// Set ALL GP8 io pins to known states
		// NOTE THIS WILL NOT WORK IF GP8_FUNCTION_SCREEN IS ADDED !!

		pinMode(PIN_SPEED_PULSE,OUTPUT);
		pinMode(PIN_WIND_PWMA,OUTPUT);
		pinMode(PIN_WIND_PWMB,OUTPUT);
		pinMode(PIN_UDP_ENABLE,INPUT_PULLDOWN);

		digitalWrite(PIN_SPEED_PULSE,0);
		analogWrite(PIN_WIND_PWMA,0);
		analogWrite(PIN_WIND_PWMB,0);

		initST50Testing();

		if (fxn == GP8_FUNCTION_ESP32)
		{
			SERIAL_ESP32.begin(921600);
			delay(500);
			display(0,"SERIAL_ESP32 started",0);
		}

		if (!fxn)
			warning(0,"GP8 FUNCTION turned off",0);

		g_GP8_FUNCTION = fxn;
	}
}




bool instSimulator::doTbEsp32()
{
	return (g_GP8_FUNCTION==GP8_FUNCTION_ESP32) && udp_enabled;
}



void instSimulator::initST50Testing()
{
	initSpeedPulse();
	analogWrite(PIN_WIND_PWMA,0);
	analogWrite(PIN_WIND_PWMB,0);
	m_wind_pwmA = 0;
	m_wind_pwmB = 0;
	m_last_pwmA = -1;
	m_last_pwmB = -1;
}


void instSimulator::initSpeedPulse()
	// called initially and whenever user changes mode or user_pulse_hz
	// causes the pulse output to re-initialize
{
	digitalWrite(PIN_SPEED_PULSE,0);
	m_pulse_hz = -1;
	m_pulse_state = false;
	m_last_pulse_toggle = 0;
	m_pulse_interval_ms = 0;
}


void instSimulator::setTestMode(bool sim_mode)
{
	display(0,"TEST_SIM_MODE(%d)",sim_mode);
	m_test_sim_mode = sim_mode;
	initST50Testing();
}


void instSimulator::setUserPulseHz(float hz)
{
	if (hz<0)
	{
		my_error("Illegal PULSE_HZ(%0.1f)",hz);
	}
	else
	{
		display(0,"PULSE_HZ(%0.1f)",hz);
		m_user_hz = hz;
		initSpeedPulse();
	}
}


void instSimulator::setWindPWM(bool pwm_b, uint8_t duty)
{
	display(0,"setWindPWM(%d=%s)=%d",pwm_b,pwm_b?"B":"A",duty);
	
	// if (duty > WIND_PWM_8V) duty = WIND_PWM_8V;
		// note: here I limit the duty cycle to WIND_PWM_8V ~= 194
		// 		to limit the output voltage to 8V FOR SAFETY in case the
		// 		tester is actually hooked up to an ST50 Wind Instrument.
		// To calibrate the values of WIND_PWM_8V and WIND_PWM_2V
		//		DISCONNECT ANY REAL WIND INSTRUMENTS
		// 		comment the above line out
		//		and use the "raw" m_test_sim_mode=0 and
		//		attache a multimeter to the tester SIGNALA and SIGNALB pins
		// to determine the constant values.

	if (pwm_b)
	{
		m_last_pwmB = -1;
		m_user_pwmB = duty;
	}
	else
	{
		m_last_pwmA = -1;
		m_user_pwmA = duty;
	}
}


void instSimulator::doST50Testing()
{
	float hz = 0;
	if (!m_test_sim_mode)
	{
		hz = m_user_hz;
		if (g_GP8_FUNCTION == GP8_FUNCTION_WIND)
		{
			m_wind_pwmA = m_user_pwmA;
			m_wind_pwmB = m_user_pwmB;
		}
	}
	else	// TEST_MODE_SIM
	{
		#define HZ_PER_KNOT  		5.6
		#define FUDGE_FACTOR 		1.0

		// when using explicit toggling, at less than 18 hz,
		// apparently the formula falls off for the ST_LOG instrument
		// and we need to increase the hz by a fudge factor.

		float speed = boat_sim.getWaterSpeed();
		hz = speed * HZ_PER_KNOT;
		if (hz < 18)
			hz = speed * HZ_PER_KNOT * FUDGE_FACTOR;

		if (g_GP8_FUNCTION == GP8_FUNCTION_WIND)
		{
			static uint32_t last_heading_change;
			uint32_t now = millis();

			if (now > last_heading_change + 1000)
			{
				last_heading_change = now;

				static float wa = 0;

				boat_sim.setHeading(0);
				wa += 5;
				if (wa>360) wa=0;
				boat_sim.setWindAngle(wa);
			}

			#define DEFAULT_NORTH	145.0
				// raw angle calculated when the vane pointing forward at my desk
				// from the teensyWind.ino program

			float apparent_angle = boat_sim.apparentWindAngle();		// degrees relative to the bow of the boat
			float theta_deg = apparent_angle + DEFAULT_NORTH;
			while (theta_deg >= 360.0f) theta_deg -= 360.0f;
			while (theta_deg <   0.0f) theta_deg += 360.0f;

			float theta = theta_deg * 3.14159265358979f / 180.0f;
			float c = cosf(theta);
			float s = sinf(theta);
			float WIND_MID_RANGE = (WIND_PWM_8V - WIND_PWM_2V) / 2;

			int pwm_blue  = (int)roundf(WIND_PWM_2V + WIND_MID_RANGE * (1.0f + c));
			int pwm_green = (int)roundf(WIND_PWM_2V + WIND_MID_RANGE * (1.0f + s));

			// optional clamp to [50,194]
			if (pwm_blue  < WIND_PWM_2V) pwm_blue  = WIND_PWM_2V;
			if (pwm_blue  > WIND_PWM_8V) pwm_blue = WIND_PWM_8V;
			if (pwm_green < WIND_PWM_2V) pwm_green = WIND_PWM_2V;
			if (pwm_green > WIND_PWM_8V) pwm_green = WIND_PWM_8V;

			// pwm_blue = WIND_PWM_2V + WIND_PWM_8V - pwm_blue;
			// pwm_green = WIND_PWM_2V + WIND_PWM_8V - pwm_green;

			m_wind_pwmA = pwm_green;
			m_wind_pwmB = pwm_blue;

			static float last_apparent_angle = 0;
			if (last_apparent_angle != apparent_angle)
			{
				last_apparent_angle = apparent_angle;
				warning(0,"apparent_angle(%0.1F)  theta_deg(%0.1f)  theta(%0.4f) c(%0.4f) s(%0.4f) pwm_blue=%d pwm_green=%d",
					apparent_angle,theta_deg,theta,c,s,pwm_blue,pwm_green);
			}
		}
	}

	// if the pulse frequency has changed, setup PWM or restart explicit toggle

	if (m_pulse_hz != hz)
	{
		m_pulse_hz = hz;
		m_pulse_state = false;
		m_last_pulse_toggle = 0;
		m_pulse_interval_ms = 0;
		pinMode(PIN_SPEED_PULSE, OUTPUT);
		digitalWrite(PIN_SPEED_PULSE,0);
		if (m_pulse_hz == 0.0)
		{
			display(0,"pulse_hz==0; pin forced low",0);
			return;;
		}

		if (m_pulse_hz >= 18)
		{
			// Use PWM for higher frequencies
			int pulse_int = roundf(m_pulse_hz);
			display(0,"using PWM Hz(%0.1f)=%d", m_pulse_hz,pulse_int);
			pinMode(PIN_SPEED_PULSE, OUTPUT);
			analogWriteFrequency(PIN_SPEED_PULSE, pulse_int);
			analogWrite(PIN_SPEED_PULSE, 128); // 50% duty
		}
		else
		{
			// Use manual toggling for lower frequencies
			float f_interval = 500.0 / m_pulse_hz;
			m_pulse_interval_ms = round(f_interval);
			m_last_pulse_toggle = millis();  // reset timer
			display(0,"using MS timer Hz(%0.1f) MS(%d)",m_pulse_hz,m_pulse_interval_ms);
		}
	}
	else if (m_pulse_hz < 18)	// implement manual toggling
	{
		uint32_t pulse_now = millis();
		if (pulse_now - m_last_pulse_toggle >= m_pulse_interval_ms)
		{
			m_last_pulse_toggle = pulse_now;
			m_pulse_state = !m_pulse_state;
			display(1,"MS pulse(%d)",m_pulse_state);
			digitalWrite(PIN_SPEED_PULSE, m_pulse_state ? HIGH : LOW);
		}
	}

	if (g_GP8_FUNCTION == GP8_FUNCTION_WIND)
	{
		if (m_last_pwmA != m_wind_pwmA)
		{
			m_last_pwmA = m_wind_pwmA;
			display(0,"writing WIND_PMWA(%d)",m_wind_pwmA);
			analogWrite(PIN_WIND_PWMA,m_wind_pwmA);
		}
		if (m_last_pwmB != m_wind_pwmB)
		{
			m_last_pwmB = m_wind_pwmB;
			display(0,"writing WIND_PMWB(%d)",m_wind_pwmB);
			analogWrite(PIN_WIND_PWMB,m_wind_pwmB);
		}
	}
}	// doST50Testing()



//----------------------------------------------------
// EEPROM
//----------------------------------------------------

#define dbg_eeprom  0
#define send_state  0

#define INST_EEPROM_BASE 	512


void instSimulator::saveToEEPROM()
{
	int offset = INST_EEPROM_BASE;
	for (int i=0; i<NUM_INSTRUMENTS; i++)
	{
		uint8_t mask = m_inst[i]->getPorts();
		EEPROM.write(offset++,mask);
		display(dbg_eeprom,"wrote mask(%d) for instrment(%d) to EEPROM",mask,i);
	}
	for (int i=0; i<NUM_PORTS; i++)
	{
		EEPROM.put(offset, g_MON[i]);	// put "knows" these are uint32_t's and stores four bytes
		offset += sizeof(uint32_t);
		display(dbg_eeprom,"wrote MON(%d)=%08x to EEPROM",i,g_MON[i]);
	}
	EEPROM.write(offset++,g_FWD);
	EEPROM.write(offset++,getE80Filter());
	EEPROM.write(offset++,g_GP8_FUNCTION);
	display(dbg_eeprom,"wrote FWD=%02x FWD=%02x GP8_FUNCTION=%02x to EEPROM",
		g_FWD,
		getE80Filter(),
		g_GP8_FUNCTION);
}


void instSimulator::loadFromEEPROM()
{
	int offset = INST_EEPROM_BASE;
	for (int i=0; i<NUM_INSTRUMENTS; i++)
	{
		uint8_t mask = EEPROM.read(offset++);
		if (mask != 255)
		{
			display(dbg_eeprom,"got mask(%d) for instrment(%d) from EEPROM",mask,i);
			m_inst[i]->setPorts(mask);
		}
	}
	for (int i=0; i<NUM_PORTS; i++)
	{
		EEPROM.get(offset,g_MON[i]);	// get knows uint32_t takes 4 bytes
		offset += sizeof(uint32_t);
		display(dbg_eeprom,"got G_MON(%d)=%08x",i,g_MON[i]);
	}
	g_FWD = EEPROM.read(offset++);
	display(dbg_eeprom,"got FWD=%02x",g_FWD);
	int e80_filter = EEPROM.read(offset++);
	display(dbg_eeprom,"got E80_FILTER=%d",e80_filter);
	setE80Filter(e80_filter);
	int fxn = EEPROM.read(offset++);
	if (fxn == 255)	// illegal init value
		fxn = 0;
	display(dbg_eeprom,"got GP8_FUNCTION=%02x",fxn);
	setGP8Function(fxn);
	sendBinaryState();
}



//-----------------------------------------------------
// accessors
//-----------------------------------------------------

// FLAG_FROM_PERL

void instSimulator::setPorts(int inst_num, uint8_t port_mask)
{
	display(0,"setPorts(%d) mask(0x%02x)",inst_num,port_mask);
	m_inst[inst_num]->setPorts(port_mask);
	sendBinaryState();
}


void instSimulator::setAll(int port_num, bool on)
{
	uint8_t port_mask = 1 << port_num;
	display(0,"setAll(%d) on(%d)",port_num,on);

	for (int i=0; i<NUM_INSTRUMENTS; i++)
	{
		instBase *inst = m_inst[i];
		uint8_t cur = inst->getPorts();
		if (on)
			cur |= port_mask;
		else
			cur &= ~port_mask;
		inst->setPorts(cur);
	}
	sendBinaryState();
}


void instSimulator::setFWD(uint8_t fwd)
{
	bool ok = 1;
	display(0,"setFWD(0x%02x)",fwd);
	if (fwd > FWD_MAX)
		ok = 0;
	// else if (((fwd & FWD_ST1_TO_2) && (fwd & FWD_ST2_TO_1)) ||
	// 		 ((fwd & FWD_83A_TO_B) && (fwd & FWD_83B_TO_A)) )
	// 	ok = 0;
	if (ok)
	{
		g_FWD = fwd;
		sendBinaryState();
	}
	else
		my_error("illegal forward value: 0x%02x",fwd);
}


void instSimulator::setMonitor(int port, uint32_t value)
{
	display(0,"instSimulator::setMonitor(port=%d) value=%08x",port,value);
	if (port<0 || port>NUM_PORTS-1)
		my_error("illegal port number*%d) in instSimulator::stMonitor()",port);
	else
		g_MON[port] = value;
}


void instSimulator::clearState()
	// clears all instruments, forwarding, monitoring, debugging, and GP8
	// and calls sendBinaryState
{
	for (int i=0; i<NUM_INSTRUMENTS; i++)
	{
		instBase *inst = m_inst[i];
		inst->setPorts(0);
	}
	g_FWD = 0;
	for (int i=0; i<NUM_PORTS; i++)
	{
		g_MON[i] = 0;
	}
	setGP8Function(0);
	boat_sim.g_MON_SIM = 0;
	sendBinaryState();

}


void instSimulator::sendBinaryState()
{
	if (!(g_BINARY & BINARY_TYPE_PROG))
		return;
	
	display(send_state,"sendBinaryState()",0);
	proc_entry();
	uint8_t buf[BINARY_HEADER_LEN + NUM_INSTRUMENTS + NUM_PORTS*4 + 3];
	int offset = startBinary(buf,BINARY_TYPE_PROG);
	display(send_state+1,"offset after header=%d",offset);
	for (int i=0; i<NUM_INSTRUMENTS; i++)
	{
		uint8_t mask = m_inst[i]->getPorts();
		display(send_state+1,"inst(%d) offset(%d) <= mask(%d)",i,offset,mask);
		offset = binaryUint8(buf,offset,mask);
	}
	for (int i=0; i<NUM_PORTS; i++)
	{
		display(send_state+1,"g_MON[%i]=%02x",i,g_MON[i]);
		offset = binaryUint32(buf,offset,g_MON[i]);
	}
	offset = binaryUint8(buf,offset,g_FWD);
	offset = binaryUint8(buf,offset,getE80Filter());
	offset = binaryUint8(buf,offset,g_GP8_FUNCTION);

	endBinary(buf,offset);
	display_bytes(send_state+1,"sending",buf,offset);
	proc_leave();
	Serial.write(buf,offset);

	#if WITH_TB_ESP32
		if (doTbEsp32())
			SERIAL_ESP32.write(buf,offset);
	#endif
}


//----------------------------------------------------
// implementation
//----------------------------------------------------

void instSimulator::init()
{
	display(0,"instSimulator::init() started",0);
	proc_entry();

	//----------------------------------
	// Port intializations
	//----------------------------------

	SERIAL_ST1.begin(4800, SERIAL_9N1);
	SERIAL_ST2.begin(4800, SERIAL_9N1);
		// Requires #define SERIAL_9BIT_SUPPORT in HardwareSerial.h
		// Uses "normal" data when using the opto-isolater as wired!
		// Note that there is also SERIAL_9N1_RXINV_TXINV which *might*
		// work with inverted signal (different circuit).

	SERIAL_83A.begin(38400);
	SERIAL_83B.begin(38400);

	//---------------------------------
	// boatSimulator initialization
	//---------------------------------
	// and instrument simulator initialization

	boat_sim.init();

	m_inst[INST_DEPTH]		= &i_depth;
	m_inst[INST_LOG]		= &i_log;
	m_inst[INST_WIND]		= &i_wind;
	m_inst[INST_COMPASS]	= &i_compass;
	m_inst[INST_GPS]		= &i_gps;
	m_inst[INST_AIS]		= &i_ais;
	m_inst[INST_AUTOPILOT]	= &i_autopilot;
	m_inst[INST_ENGINE]		= &i_engine;
	m_inst[INST_GENSET]		= &i_genset;

	// one time clear
	// saveToEEPROM();

	loadFromEEPROM();

	// init nmea2000 after EEPROM loaded
	
	nmea2000.init(TEENSYBOAT_NMEA_ADDRESS);

	proc_leave();
	display(0,"instSimulator::init() finished",0);
}




void handleStPort(
	bool port2,
	uint32_t *last_st_in,
	uint32_t *last_st_out,
	int *outp,
	int *dlen,
	uint8_t *datagram,
	Stream &SERIAL_ST)
{
	while (SERIAL_ST.available())
	{
		int c = SERIAL_ST.read();
		*last_st_in = millis();

		// the 9th bit is set on the first 'byte' of a sequence
		// the low nibble of the 2nd byte + 3 is the total number
		// 		of bytes, so all messages are at least 3 bytes
		//		the high nibble may be data.
		//	data[n+3];, implying a maximum datagram size of 19
		//  this routine can never receive more than 19 bytes
		//	but MAX_ST_BUF is set to 20 for neatness

		#if 0
			display(0,"ST%d got 0x%02x '%c'",port2,c,(c>32 && c<128)?c:' ');
		#endif

		if (c > 0xff)
		{
			if (*outp)
			{
				my_error("Dropped datagram ",0);
				*outp = 0;
			}
			datagram[(*outp)++] = c;
		}
		else if (*outp == 1)
		{
			*dlen = (c & 0x0f) + 3;
			datagram[(*outp)++] = c;
		}
		else if (*outp < *dlen)
		{
			datagram[(*outp)++] = c;
			if (*outp == *dlen)
			{
				showDatagram(port2,false,datagram);
				*outp = 0;
				*dlen = 0;
			}
		}
		else
		{
			my_error("unexpected byte 0x%02x '%c'",c,(c>32 && c<128)?c:' ');
		}

	}	// receiving datagrams


	// send one datagram from the ST port's queue

	uint32_t now_st = millis();
	if (now_st - *last_st_in >= ST_IDLE_BUS_MS &&
		now_st - *last_st_out > ST_SEND_INTERVAL)
	{
		sendDatagram(port2);
		*last_st_out = millis();
	}
}



//---------------------------------------------------
// RUN
//---------------------------------------------------

void instSimulator::run()
{
	if (g_GP8_FUNCTION == GP8_FUNCTION_ESP32)
	{
		bool enabled = digitalRead(PIN_UDP_ENABLE);
		if (udp_enabled != enabled)
		{
			udp_enabled = enabled;
			if (enabled)
			{
				extraSerial = &SERIAL_ESP32;
				display(0,"attaching extraSerial to SERIAL_ESP32",0);
			}
			else
			{
				display(0,"detaching extraSerial from SERIAL_ESP32",0);
				extraSerial = 0;
			}
		}
	}


	//-----------------
	// simulator
	//-----------------

	uint32_t now = millis();
	static uint32_t last_update = 0;
	if (now - last_update >= UPDATE_MILLIS)
	{
		last_update = now;
		boat_sim.run();
		if (boat_sim.running())
		{
			// This should not be necessary unless we are queing more ST messages
			// than can, in fact, be sent in a single second. I commented it out
			// because it clears the queue separate from the asynchronous neoGPS
			// usage of the queue.
			//
			//		clearSTQueues();

			for (int i=0; i<NUM_INSTRUMENTS; i++)
			{
				delay(10);

				instBase *inst = m_inst[i];
				if (inst->portActive(PORT_ST1))
					inst->sendSeatalk(false);
				if (inst->portActive(PORT_ST2))
					inst->sendSeatalk(true);
				if (inst->portActive(PORT_83A))
					inst->send0183(false);
				if (inst->portActive(PORT_83B))
					inst->send0183(true);
				if (inst->portActive(PORT_2000))
					inst->send2000();
			}
		}

		st_device_query_pending = 0;
			// clear any pending st_device_query
	}


	//-----------------
	// NMEA2000
	//-----------------

	nmea2000.ParseMessages(); // Keep parsing messages

	#if BROADCAST_NMEA2000_INFO
		nmea2000.broadcastNMEA2000Info();
	#endif


	//-----------------
	// NMEA0183
	//-----------------

	#define MAX_0183_MSG 180

	while (SERIAL_83A.available())
	{
		int c = SERIAL_83A.read();
		static char buf[MAX_0183_MSG+1];
		static int buf_ptr = 0;
		// display(0,"got Serial2 0x%02x %c",c,c>32 && c<127 ? c : ' ');
		if (buf_ptr >= MAX_0183_MSG || c == 0x0a)
		{
			buf[buf_ptr] = 0;
			handleNMEA0183Input(false,buf);
			buf_ptr = 0;
		}
		else if (c != 0x0d)
		{
			buf[buf_ptr++] = c;
		}
	}

	while (SERIAL_83B.available())
	{
		int c = SERIAL_83B.read();
		static char buf[MAX_0183_MSG+1];
		static int buf_ptr = 0;
		// display(0,"got Serial2 0x%02x %c",c,c>32 && c<127 ? c : ' ');
		if (buf_ptr >= MAX_0183_MSG || c == 0x0a)
		{
			buf[buf_ptr] = 0;
			handleNMEA0183Input(true,buf);
			buf_ptr = 0;
		}
		else if (c != 0x0d)
		{
			buf[buf_ptr++] = c;
		}
	}

	//-----------------
	// Seatalk
	//-----------------
	// if (1) and brackets serve to create scope
	// for static per-port variables

	if (1)
	{
		static uint32_t last_st_in;
		static uint32_t last_st_out;
		static int outp = 0;
		static int dlen = 0;
		static uint8_t datagram[MAX_ST_BUF];
		handleStPort(false,&last_st_in,&last_st_out,&outp,&dlen,datagram,SERIAL_ST1);
	}
	if (1)
	{
		static uint32_t last_st_in;
		static uint32_t last_st_out;
		static int outp = 0;
		static int dlen = 0;
		static uint8_t datagram[MAX_ST_BUF];
		handleStPort(true,&last_st_in,&last_st_out,&outp,&dlen,datagram,SERIAL_ST2);
	}


	//----------------------------
	// ST$0 Testing
	//----------------------------

	if (g_GP8_FUNCTION == GP8_FUNCTION_SPEED ||
		g_GP8_FUNCTION == GP8_FUNCTION_WIND)
		doST50Testing();

}	// instSimulator::run()




// end of instSimulator.cpp
