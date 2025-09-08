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
#include <inst2000.h>
	// only port that has a complicated initialization method
#include <myDebug.h>


#define UPDATE_MILLIS	1000

instSimulator instruments;
	// global instance

depthInst		i_depth;
logInst			i_log;
windInst		i_wind;
compassInst		i_compass;
gpsInst			i_gps;
aisInst			i_ais;
autopilotInst	i_autopilot;
engineInst		i_engine;
gensetInst		i_genset;


void instSimulator::init()
{
	display(0,"instSimulator::init() started",0);
	proc_entry();

	//----------------------------------
	// Seatalk initialization
	//----------------------------------

	SERIAL_SEATALK.begin(4800, SERIAL_9N1);
		// Requires #define SERIAL_9BIT_SUPPORT in HardwareSerial.h
		// Uses "normal" data when using the opto-isolater as wired!
		// Note that there is also SERIAL_9N1_RXINV_TXINV which *might*
		// work with inverted signal (different circuit).

	//----------------------------------
	// NMEA0183 initialization
	//----------------------------------

	#if TEST_RS232
		pinMode(TEST_OUT1,OUTPUT);
		pinMode(TEST_IN1,INPUT_PULLUP);
		pinMode(TEST_OUT2,OUTPUT);
		pinMode(TEST_IN2,INPUT_PULLUP);
		digitalWrite(TEST_OUT1,1);
		digitalWrite(TEST_OUT2,1);
	#else
		SERIAL_0183.begin(38400);
	#endif




	boat.init();

	m_inst[INST_DEPTH]		= &i_depth;
	m_inst[INST_LOG]		= &i_log;
	m_inst[INST_WIND]		= &i_wind;
	m_inst[INST_COMPASS]	= &i_compass;
	m_inst[INST_GPS]		= &i_gps;
	m_inst[INST_AIS]		= &i_ais;
	m_inst[INST_AUTOPILOT]	= &i_autopilot;
	m_inst[INST_ENGINE]		= &i_engine;
	m_inst[INST_GENSET]		= &i_genset;

	proc_leave();
	display(0,"instSimulator::init() finished",0);
}


void instSimulator::run()
{
	#if 1
		uint32_t now = millis();
		static uint32_t last_update = 0;
		if (now - last_update >= UPDATE_MILLIS)
		{
			last_update = now;
			boat.run();
			if (boat.running())
			{
				for (int i=0; i<NUM_INSTRUMENTS; i++)
				{
					delay(10);
					instBase *inst = m_inst[i];
					if (inst->portActive(PORT_SEATALK))
						inst->sendSeatalk();
					if (inst->portActive(PORT_0183))
						inst->send0183();
					if (inst->portActive(PORT_2000))
						inst->send2000();
				}
			}
		}
	#endif


	#if 0	// listen for input data


		nmea2000.ParseMessages(); // Keep parsing messages

		while (SERIAL_0183.available())
		{
			#define MAX_MSG 180
			int c = SERIAL_0183.read();
			static char buf[MAX_MSG+1];
			static int buf_ptr = 0;

			// display(0,"got Serial2 0x%02x %c",c,c>32 && c<127 ? c : ' ');

			if (buf_ptr >= MAX_MSG || c == 0x0a)
			{
				extern void handleNMEAInput(const char *);

				buf[buf_ptr] = 0;
				handleNMEAInput(buf);
				buf_ptr = 0;
			}
			else if (c != 0x0d)
			{
				buf[buf_ptr++] = c;
			}
		}

	#endif	// 0 at this time
}


// end of instSimulator.cpp
