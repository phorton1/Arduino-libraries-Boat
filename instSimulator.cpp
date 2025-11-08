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


instSimulator instruments;
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

uint8_t instSimulator::g_MON[NUM_PORTS];
uint8_t instSimulator::g_FWD;


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
		EEPROM.write(offset++, g_MON[i]);
		display(dbg_eeprom,"wrote MON(%d)=%02x to EEPROM",i,g_MON[i]);
	}
	EEPROM.write(offset++,g_FWD);
	display(dbg_eeprom,"wrote FWD=%02x to EEPROM",g_FWD);
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
		g_MON[i] = EEPROM.read(offset++);
		display(dbg_eeprom,"got G_MON(%d)=%02x",i,g_MON[i]);
	}
	g_FWD = EEPROM.read(offset++);
	display(dbg_eeprom,"got FWD=%02x",g_FWD);

	sendBinaryState();
}


//-----------------------------------------------------
// accessors
//-----------------------------------------------------

// FLAG_FROM_PERL

void instSimulator::setPorts(int inst_num, uint8_t port_mask, bool no_echo)
{
	display(0,"setPorts(%d) mask(%d) no_echo(%d)",inst_num,port_mask,no_echo);
	m_inst[inst_num]->setPorts(port_mask);
	if (!no_echo) sendBinaryState();
}


void instSimulator::setAll(int port_num, bool on, bool no_echo)
{
	uint8_t port_mask = 1 << port_num;
	display(0,"setAll(%d) on(%d) no_echo(%d)",port_num,on,no_echo);

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
	if (!no_echo) sendBinaryState();
}


void instSimulator::sendBinaryState()
{
	display(send_state,"sendBinaryState()",0);
	proc_entry();
	uint8_t buf[BINARY_HEADER_LEN + NUM_INSTRUMENTS];
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
		offset = binaryUint8(buf,offset,g_MON[i]);
	}
	offset = binaryUint8(buf,offset,g_FWD);
	endBinary(buf,offset);
	display_bytes(send_state+1,"sending",buf,offset);
	proc_leave();
	Serial.write(buf,offset);
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

	SERIAL_ST.begin(4800, SERIAL_9N1);
		// Requires #define SERIAL_9BIT_SUPPORT in HardwareSerial.h
		// Uses "normal" data when using the opto-isolater as wired!
		// Note that there is also SERIAL_9N1_RXINV_TXINV which *might*
		// work with inverted signal (different circuit).
// Serial1.setTX(1);  // Explicitly set TX pin
// Serial1.setRX(0);  // Explicitly set RX pin



	SERIAL_83A.begin(38400);
	SERIAL_83B.begin(38400);

	nmea2000.init();

	//---------------------------------
	// boatSimulator initialization
	//---------------------------------
	// and instrument simulator initialization

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


	// one time clear
	// saveToEEPROM();

	loadFromEEPROM();
	
	proc_leave();
	display(0,"instSimulator::init() finished",0);
}



bool clear = 0;

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
				clearSTQueue();
				for (int i=0; i<NUM_INSTRUMENTS; i++)
				{
					delay(10);
					instBase *inst = m_inst[i];
					if (inst->portActive(PORT_ST))
						inst->sendSeatalk();
					if (inst->portActive(PORT_83A))
						inst->send0183(false);
					if (inst->portActive(PORT_83B))
						inst->send0183(true);
					if (inst->portActive(PORT_2000))
						inst->send2000();
				}
			}
		}
	#endif


	#if 1	// listen for NMEA2000 data
		nmea2000.ParseMessages(); // Keep parsing messages
	#endif

	#if BROADCAST_NMEA2000_INFO
		nmea2000.broadcastNMEA2000Info();
	#endif

	#if 1	// listen for NMEA0183A data
		while (SERIAL_83A.available())
		{
			#define MAX_MSG 180
			int c = SERIAL_83A.read();
			static char buf[MAX_MSG+1];
			static int buf_ptr = 0;

			// display(0,"got Serial2 0x%02x %c",c,c>32 && c<127 ? c : ' ');

			if (buf_ptr >= MAX_MSG || c == 0x0a)
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
	#endif

	#if 1	// listen for NMEA0183B data
		while (SERIAL_83B.available())
		{
			#define MAX_MSG 180
			int c = SERIAL_83B.read();
			static char buf[MAX_MSG+1];
			static int buf_ptr = 0;

			// display(0,"got Serial2 0x%02x %c",c,c>32 && c<127 ? c : ' ');

			if (buf_ptr >= MAX_MSG || c == 0x0a)
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

	#endif	// NMEA0183 data

	#if 1	// listen for Seatalk data

		static uint32_t last_st_in;
		while (SERIAL_ST.available())
		{
			int c = SERIAL_ST.read();
			last_st_in = millis();

			// the 9th bit is set on the first 'byte' of a sequence
			// the low nibble of the 2nd byte + 3 is the total number
			// 		of bytes, so all messages are at least 3 bytes
			//		the high nibble may be data.
			//	data[n+3];, implying a maximum datagram size of 19
			//  this routine can never receive more than 19 bytes
			//	but MAX_ST_BUF is set to 20 for neatness

			#if 0
				display(0,"got 0x%02x '%c'",c,(c>32 && c<128)?c:' ');
			#endif

			static uint8_t datagram[MAX_ST_BUF];
			static int outp = 0;
			static int dlen = 0;

			if (c > 0xff)
			{
				if (outp)
				{
					my_error("Dropped datagram ",0);
					outp = 0;
				}
				datagram[outp++] = c;
			}
			else if (outp == 1)
			{
				dlen = (c & 0x0f) + 3;
				datagram[outp++] = c;
			}
			else if (outp < dlen)
			{
				datagram[outp++] = c;
				if (outp == dlen)
				{
					showDatagram(false,datagram);
					outp = 0;
					dlen = 0;
				}
			}
			else
			{
				my_error("unexpected byte 0x%02x '%c'",c,(c>32 && c<128)?c:' ');
			}

		}	// receiving datagrams


		// send one datagram from queue

		#define IDLE_BUS_MS				10		// ms bus must be idle to send next datagram
		#define SEND_INTERVAL			10

		uint32_t now_st = millis();
		static uint32_t last_st_out = 0;
		if (now_st - last_st_in >= IDLE_BUS_MS &&
			now_st - last_st_out > SEND_INTERVAL)
		{
			sendDatagram();
			last_st_out = millis();
		}


	#endif

}




// end of instSimulator.cpp
