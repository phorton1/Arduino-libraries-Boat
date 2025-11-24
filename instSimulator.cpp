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
	EEPROM.write(offset++,getE80Filter());
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
	int e80_filter = EEPROM.read(offset++);
	display(dbg_eeprom,"got E80_FILTER=%d",e80_filter);
	setE80Filter(e80_filter);

	display(dbg_eeprom,"got FWD=%02x",g_FWD);

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


void instSimulator::setFWD(int fwd)
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



void instSimulator::sendBinaryState()
{
	display(send_state,"sendBinaryState()",0);
	proc_entry();
	uint8_t buf[BINARY_HEADER_LEN + NUM_INSTRUMENTS + NUM_PORTS + 2];
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
	offset = binaryUint8(buf,offset,getE80Filter());
	
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

	SERIAL_ST1.begin(4800, SERIAL_9N1);
	SERIAL_ST2.begin(4800, SERIAL_9N1);
		// Requires #define SERIAL_9BIT_SUPPORT in HardwareSerial.h
		// Uses "normal" data when using the opto-isolater as wired!
		// Note that there is also SERIAL_9N1_RXINV_TXINV which *might*
		// work with inverted signal (different circuit).

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



void instSimulator::run()
{
	uint32_t now = millis();
	static uint32_t last_update = 0;
	if (now - last_update >= UPDATE_MILLIS)
	{
		last_update = now;
		boat.run();
		if (boat.running())
		{
			clearSTQueues();
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

		// kludge attempt to emulate AP computer

		if (0)
		{
			sendSTCourseComputer();
		}
	}


	// listen for NMEA2000 data

	nmea2000.ParseMessages(); // Keep parsing messages
	#if BROADCAST_NMEA2000_INFO
		nmea2000.broadcastNMEA2000Info();
	#endif


	// listen for NMEA0183 data

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


	// listen for Seatalk data
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

}




// end of instSimulator.cpp
