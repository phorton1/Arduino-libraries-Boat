//---------------------------------------------
// instST_out.cpp
//---------------------------------------------
// Implementation of simulated Seatalk instruments.

#include "instST.h"
#include "instSimulator.h"
#include "boatSimulator.h"
#include "timeLib.h"
#include <myDebug.h>

volatile int ap_linked = 0;
volatile bool st_device_query_pending = 0;



#define dbg_data 		(inst_sim.g_MON[port2?PORT_ST2:PORT_ST1]>1 ? 0 : 1)

#define WRITE_TIMEOUT			50

#define NUM_ST_PORTS		    2
#define CIRC_BUF_SIZE  			20

static uint16_t dg[MAX_ST_BUF];
	// there is only one dg message being built at a time
	// perhaps this *should* be local to each instrument method

// but there are two separate queues

static uint16_t circ[NUM_ST_PORTS][CIRC_BUF_SIZE][MAX_ST_BUF];
static int head[NUM_ST_PORTS];
static int tail[NUM_ST_PORTS];

void clearSTQueues()
{
	head[0] = 0;
	tail[0] = 0;
	head[1] = 0;
	tail[1] = 0;
}


void queueDatagram8(bool port2, const uint8_t *dg, bool quiet)
{
	uint16_t dg16[MAX_ST_BUF];
	int len = (dg[1] & 0xf) + 3;
	for (int i=0; i<len; i++)
	{
		dg16[i] = dg[i];
	}
	dg16[0] |= ST_COMMAND_BIT;
	if (quiet)
		dg16[0] |= ST_QUIET_BIT;

	queueDatagram(port2,dg16);
}


void queueDatagram(bool port2, const uint16_t *dg)
{
	int len = (dg[1] & 0xf) + 3;
	uint16_t *ptr = circ[port2][head[port2]++];
	memcpy(ptr,dg,len * sizeof(uint16_t));
	if (head[port2] >= CIRC_BUF_SIZE)
		head[port2] = 0;
	if (head[port2] == tail[port2])
	{
		tail[port2]++;
		if (tail[port2] > CIRC_BUF_SIZE)
			tail[port2] = 0;
	}
	// showDatagram16(port2,true,dg);
}





bool sendDatagram(bool port2)
{
	if (tail[port2] == head[port2])
		return false;

	HardwareSerial &SERIAL_ST = port2 ?
		SERIAL_ST2 :
		SERIAL_ST1 ;

	uint16_t *dg = circ[port2][tail[port2]++];
	if (tail[port2] > CIRC_BUF_SIZE)
		tail[port2] = 0;

	// the 10th bit (ST_QUIET_BIT) will not be sent
	// but we need to remove it for readback check

	if (dg[0] & ST_QUIET_BIT)
	{
		dg[0] &= ~ST_QUIET_BIT;
	}
	else
	{
		showDatagram16(port2,true,dg);
	}

	int len = (dg[1] & 0xf) + 3;
	for (int i=0; i<len; i++)
	{
		SERIAL_ST.write9bit(dg[i]);
	}

	bool reported = 0;
	uint32_t send_time = millis();
	int got = 0;
	while (got < len)
	{
		if (SERIAL_ST.available())
		{
			int c = SERIAL_ST.read();
			if (c != dg[got++])
			{
				if (!reported)
				{
					warning(0,"collision(0x%03x) at %d",dg[0],got);
					reported = 1;
				}
 			}
		}
		else if (millis() - send_time >= WRITE_TIMEOUT)
		{
			my_error("READBACK TIMEOUT at chr(%d)",got);
			return true;
		}
	}

	return true;
}


//------------------------------------
// general
//------------------------------------

void setLampIntensity(int value)
{
	// sent to all ports, 0==off, 1=low, 2=medium, 3=high
	display(0,"Set Lamp Intensity(%d)",value);
	dg[0] = ST_LAMP_INTENSITY;
	dg[1] = 0x00;
	dg[2] =
		value == 3 ? 0x0c :
		value == 2 ? 0x08 :
		value == 1 ? 0x04 : 0x00;
	queueDatagram(false,dg);
	queueDatagram(true,dg);

}




static void sendDeviceId(bool port2, uint8_t id, uint8_t version, uint8_t subversion)
	// send device ident if st_device_query_pending
{
	if (st_device_query_pending)
	{
		dg[0] = ST_DEV_QUERY;	// 0x1a4
		dg[1] = 0x12;      		// 0x10 constant + length
		dg[2] = id;      		// Unit ID = Seatalk GPS
		dg[3] = version;      	// Main SW version
		dg[4] = subversion;     // Minor SW version
		queueDatagram(port2,dg);
	}
}



//-------------------------------------
// instruments
//-------------------------------------


void depthInst::sendSeatalk(bool port2)
{
	display(dbg_data,"st%d depth(%0.1f)",port2,boat_sim.getDepth());
	sendDeviceId(port2,0x01,1,1);		// Depth device

	uint16_t d10;
	if (boat_sim.getDepth() > 999)
		d10 = 9990;
	else
		d10 = (boat_sim.getDepth() + 0.05) * 10.0;

	dg[0] = ST_DEPTH;
	dg[1] = 0x02;
	dg[2] = 0;	// flags
	dg[3] = d10 & 0xff;
	dg[4] = (d10 >> 8) & 0xff;

	queueDatagram(port2,dg);
}



void logInst::sendSeatalk(bool port2)
{
	sendDeviceId(port2,0x02,1,2);		// Speed device

	double speed = boat_sim.getWaterSpeed();
	int ispeed = (speed+ 0.05) * 10;
	display(dbg_data,"st%d WaterSpeed(%0.1f)",port2,speed);

	dg[0] = ST_WATER_SPEED;
	dg[1] = 0x01;
	dg[2] = ispeed & 0xff;
	dg[3] = (ispeed >> 8) & 0xff;
	queueDatagram(port2,dg);

	// trip distance, total, or both

	if (0)
	{
		double trip_distance = boat_sim.getTripDistance();
		uint32_t trip_int = round(trip_distance * 100.0);

		dg[0] = ST_TRIP;
		dg[1] = 0x02;
		dg[2] = trip_int & 0xff;
		dg[3] = (trip_int >> 8) & 0xff;
		dg[4] = (trip_int >> 16) & 0x0f;
		queueDatagram(port2,dg);
	}
	if (0)
	{
		double total_distance = boat_sim.getLogTotal();
		uint32_t total_int = round(total_distance * 10.0);

		dg[0] = ST_LOG_TOTAL;
		dg[1] = 0x02;
		dg[2] = total_int & 0xff;
		dg[3] = (total_int >> 8) & 0xff;
		dg[4] = 0;
		queueDatagram(port2,dg);
	}
	if (1)
	{
		double trip_distance = boat_sim.getTripDistance();
		double total_distance = boat_sim.getLogTotal();
		uint32_t trip_int = round(trip_distance * 100.0);
		uint32_t total_int = round(total_distance * 10.0);

		dg[0] = ST_TRIP_TOTAL;
		dg[1] = 0x04;
		dg[2] = total_int & 0xff;
		dg[3] = (total_int >> 8) & 0xff;
		dg[1] |= ((total_int >> 16) & 0xff) << 4;

		dg[4] = trip_int & 0xff;
		dg[5] = (trip_int >> 8) & 0xff;
		dg[6] = (trip_int >> 16) & 0x0f;
		dg[6] |= 0xA0;
		queueDatagram(port2,dg);
	}
}



void windInst::sendSeatalk(bool port2)
{
	sendDeviceId(port2,0x06,1,3);		// Wind device

	double speed = boat_sim.apparentWindSpeed();	// getWindSpeed();
	double angle = boat_sim.apparentWindAngle();	// getWindAngle();

	int tenths = (speed + 0.05) * 10.0;
	int ispeed = tenths / 10;
	tenths = tenths % 10;

	display(dbg_data,"st%d WindSpeed(%0.1f) ispeed=%d tenths=%d",port2,speed,ispeed,tenths);

	dg[0] = ST_WIND_SPEED;
	dg[1] = 0x01;
	dg[2] = (ispeed & 0x7f);		// high order bit 0=knots; 1 would be fathoms
	dg[3] = tenths;
	queueDatagram(port2,dg);

	display(dbg_data,"st%d WindAngle(%0.1f)",port2,angle);

	int a2 = round(angle * 2.0);
	dg[0] = ST_WIND_ANGLE;
	dg[1] = 0x01;
	dg[2] = (a2 >> 8) & 0xff;
	dg[3] = a2 & 0xff;
	queueDatagram(port2,dg);
}


void compassInst::sendSeatalk(bool port2)
{
	sendDeviceId(port2,0x18,1,4);		// ST30 compass device

	// what a weird encoding
	// get true hading, convert to magnetic version via getMagneticVariance
	// round to one decimal place

	float heading = boat_sim.makeMagnetic(boat_sim.getHeading());
	double f_heading = roundf(heading * 10.0f) / 10.0f;

	// calculate weird ninetees, twos, and halfs encoding

	int h_halfs = f_heading * 2;
	int h_nineties = h_halfs / 180;
	h_halfs = h_halfs - (h_nineties * 180);
	int h_twos = h_halfs / 4;
	h_halfs = h_halfs - (h_twos * 4);

	display(dbg_data,"st%d Heading(%0.4f) = f_heading(%0.1f) nineties(%d) twos(%d) halfs(%d)",port2,heading,f_heading,h_nineties,h_twos,h_halfs);

	// put encoding into the two bytes
	// according to knauff
	//
	//   U    2         VW
	// .... ....     ........
	// HHNN 0010       TTTTTT
	// halfs  2         twos

	dg[0] = ST_HEADING;									// 0x189
	dg[1] = 2 | (h_nineties << 4) | (h_halfs<<6);		// U2
	dg[2] = h_twos;										// VW
	dg[3] = 0x00;										// XY - unuaed by me at this time
	dg[4] = 0x20;										// 2Z - unuaed by me at this time
	queueDatagram(port2,dg);
}



void gpsInst::sendSeatalk(bool port2)
{
	display(dbg_data,"gpsInst:sendSeatalk(%d)",port2);
	proc_entry();
	// sendDeviceId(port2,0x0d,1,5);	// Seatalk GPS device
	sendDeviceId(port2,0xc4,1,5);	// RS120 GPS device

	//------------------------------------------------
	// Satellite Info (simple summary)
	//------------------------------------------------

	// Satellite IDs

	uint8_t sat1_id = 0x07;
	uint8_t sat2_id = 0x08;
	uint8_t sat3_id = 0x0A;

	if (1)
	{
		display(dbg_data,"st%d SatInfo()",port2);
		dg[0] = ST_SAT_INFO;	// 0x157
		dg[1] = 0x30;      		// num_sats << 4
		dg[2] = 0x02;          	// HDOP = 2
		queueDatagram(port2,dg);
	}

	//------------------------------------------------
	// SAT_DETAIL (0x57)  — GPS quality + HDOP block
	//------------------------------------------------
	
	if (1)
	{
		display(dbg_data,"st%d SatDetail(1)",port2);

		dg[0] = ST_SAT_DETAIL;		// // 0x1A5
		dg[1] = 0x57;

		// QQ
		// 		quality = QQ&0xF 						= .... 0010 = 0x02 = 2
		//		quality_available = QQ&0x10 			= ...1 .... = 0x10 = 1
		// 		high 3 bits of numsats = QQ&0xE0/16  	= 001. .... = 0x20 = 2
		// Knauf uses QQ&0xE0/16 to mean (QQ&0xE0)>>4 in exprssion for numsats

		uint8_t QQ = 0x32;          // 0x02 | 0x10 | 0x20
		dg[2] = QQ;

		// HH:
		//		HDOP = HH&0x7C = HH & 01111100			= .000 11.. = 0x0C = 3
		//		HDOP availability = HH&0x80 		    = 1... .... = 0x80 = 1
		//		low bit of numsats = HH&0x01			= .... ...1 = 0x01 = 1
		//		numsats available = HH&0x02				= .... ..1. = 0x20 = 1
		// hdop = 3, hdop_available = 1, num_sats_available = 1, low bit of num_sats = 1, numsats available = 1

		uint8_t HH = 0x8F;			// 1000 1111 = 0x8f
 		dg[3] = HH;

		dg[4] = 0x00;               // ?? = unknown
		dg[5] = 0x33;               // AA = antenna height; apparenly 0x33 is a constant that is seen in real Raystar GPS devices
		dg[6] = 0x20;               // GG = geoidal separation (32 * 16 = 512 m)
		dg[7] = 0x00;               // ZZ = differential age high bits
		dg[8] = 0x00;               // YY = diff age low bits + flags + station ID high bits
		dg[9] = 0x00;               // DD = station ID low

		queueDatagram(port2,dg);
	}

	//------------------------------------------------
	// SAT_DETAIL (0x74) — Satellite ID list
	//------------------------------------------------

	if (0)
	{
		display(dbg_data,"st%d SatDetail(2)",port2);

		dg[0] = ST_SAT_DETAIL;		// 0x1A5
		dg[1] = 0x74;				// 0x79 constant plus length
		dg[2] = sat1_id;
		dg[3] = sat2_id;
		dg[4] = sat3_id;
		dg[5] = 0x00;               // id4
		dg[6] = 0x00;               // id5
		queueDatagram(port2,dg);
	}

	//-----------------------------------------------
	// Unknown Meaning
	//-----------------------------------------------


	if (1)
	{
		display(dbg_data,"st%d SatDetail(unknown)",port2);

		// A5  61  04  E2    , A5 8D ..., A5 98 ..., A5 B5 ..., A5 0C...  Unknown meaning

		dg[0] = ST_SAT_DETAIL;
		dg[1] = 0x61;
		dg[2] = 0x04;
		dg[3] = 0xe2;
		queueDatagram(port2,dg);
	}


	//------------------------------------------------
	// SAT_DETAIL (0x0D) — Satellite geometry + SNR
	//------------------------------------------------

	if (0)
	{
		// Modified Knauf #1 - repeat first four bytes 3 times

		display(dbg_data,"st%d SatDetail(test opt1 0x0B)",port2);

		// --- sat1 ---
		uint8_t s1_id  = 7;
		uint8_t s1_az  = 48;
		uint8_t s1_el  = 79;
		uint8_t s1_snr = 42;

		// --- sat2 ---
		uint8_t s2_id  = 8;
		uint8_t s2_az  = 182;
		uint8_t s2_el  = 62;
		uint8_t s2_snr = 45;

		// --- sat3 ---
		uint8_t s3_id  = 10;
		uint8_t s3_az  = 180;
		uint8_t s3_el  = 51;
		uint8_t s3_snr = 43;

		// pack using Knauf's sat1 rules for all sats
		uint8_t r1_id  = (s1_id << 1) & 0xFE;
		uint8_t r1_az  = s1_az / 2;
		uint8_t r1_el  = (s1_el << 1) | (s1_az & 0x01);
		uint8_t r1_snr = (s1_snr << 1) & 0xFE;

		uint8_t r2_id  = (s2_id << 1) & 0xFE;
		uint8_t r2_az  = s2_az / 2;
		uint8_t r2_el  = (s2_el << 1) | (s2_az & 0x01);
		uint8_t r2_snr = (s2_snr << 1) & 0xFE;

		uint8_t r3_id  = (s3_id << 1) & 0xFE;
		uint8_t r3_az  = s3_az / 2;
		uint8_t r3_el  = (s3_el << 1) | (s3_az & 0x01);
		uint8_t r3_snr = (s3_snr << 1) & 0xFE;

		// emit 0x0B block (12 bytes payload)
		dg[0]  = ST_SAT_DETAIL;   // 0x1A5
		dg[1]  = 0x0d;		// 0x0B;

		dg[2]  = r1_id;
		dg[3]  = r1_az;
		dg[4]  = r1_el;
		dg[5]  = r1_snr;

		dg[6]  = r2_id;
		dg[7]  = r2_az;
		dg[8]  = r2_el;
		dg[9]  = r2_snr;

		dg[10] = r3_id;
		dg[11] = r3_az;
		dg[12] = r3_el;
		dg[13] = r3_snr;

		dg[14] = 0;
		dg[15] = 0;

		queueDatagram(port2,dg);
	}


	if (1)
	{
		// Original Knauf to best of weird ability

		display(dbg_data,"st%d SatDetail(0x0d)",port2);

		// Satellite 1 (PRN 07)
		uint8_t sat1_az  = 48;
		uint8_t sat1_el  = 79;
		uint8_t sat1_snr = 42;

		// Satellite 2 (PRN 08)
		uint8_t sat2_az  = 182;
		uint8_t sat2_el  = 62;
		uint8_t sat2_snr = 45;

		// Satellite 3 (PRN 10)
		uint8_t sat3_az  = 180;
		uint8_t sat3_el  = 51;
		uint8_t sat3_snr = 43;

		// ---- Pack satellite 1 ----
		uint8_t NN = sat1_id & 0xFE;
		uint8_t AA = sat1_az / 2;
		uint8_t EE = (sat1_el << 1) | (sat1_az & 0x01);
		uint8_t SS = (sat1_snr << 1) & 0xFE;

		// ---- Pack satellite 2 ----
		uint8_t MM = (sat2_id << 1) & 0x70;
		uint8_t BB = (sat2_id & 0x07) | ((sat2_az / 2) & 0xF8);
		uint8_t FF = (sat2_az & 0x0F) | ((sat2_el << 1) & 0xF0);
		uint8_t GG = ((sat2_el & 0x07) << 5) | (sat2_snr & 0x3F);
		uint8_t OO = sat2_snr & 0x3F;

		// ---- Pack satellite 3 ----
		uint8_t CC = (sat3_id & 0x3F) | ((sat3_az >> 1) & 0xC0);
		uint8_t DD = sat3_az & 0x7F;
		uint8_t XX = sat3_el & 0x7F;
		uint8_t YY = (sat3_snr << 2) & 0xFC;
		uint8_t ZZ = sat3_snr & 0x01;

		// ---- Emit 0x0D block ----
		dg[0]  = ST_SAT_DETAIL;	// 0x1A5
		dg[1]  = 0x0D;		// 0x8D;
		dg[2]  = NN;
		dg[3]  = AA;
		dg[4]  = EE;
		dg[5]  = SS;
		dg[6]  = MM;
		dg[7]  = BB;
		dg[8]  = FF;
		dg[9]  = GG;
		dg[10] = OO;
		dg[11] = CC;
		dg[12] = DD;
		dg[13] = XX;
		dg[14] = YY;
		dg[15] = ZZ;

		queueDatagram(port2,dg);
	}


	//------------------------------------------
	// LatLon
	//------------------------------------------
	// Some folks think it better to send separate LAT and LON messags

	if (1)
	{
		double lat = boat_sim.getLat();
		double lon = boat_sim.getLon();
		display(dbg_data,"st%d LatLon(%0.6f,%0.6f)",port2,lat,lon);

		uint8_t Z1 = 0;
		uint8_t Z2 = 0x20;
		if (lat < 0)
		{
			lat = abs(lat);
			Z1 = 0x10;
		}
		if (lon < 0)
		{
			lon = abs(lon);
			Z2 = 0x0;
		}

		// integer portions
		uint16_t i_lat = lat;
		uint16_t i_lon = lon;

		// right of decimal point
		float frac_lat = lat - i_lat;
		float frac_lon = lon - i_lon;

		// converted to minutes
		float min_lat = frac_lat * 60.0;
		float min_lon = frac_lon * 60.0;

		// times 1000 into integers
		int imin_lat = round(min_lat * 1000.0);
		int imin_lon = round(min_lon * 1000.0);

		proc_entry();
		display(dbg_data+1,"i_lat(%d) frac_lat(%0.6f) min_lat(%0.6f) imin_lat(%d)",i_lat,frac_lat,min_lat,imin_lat);
		display(dbg_data+1,"i_lon(%d) frac_lon(%0.6f) min_lon(%0.6f) imin_lon(%d)",i_lon,frac_lon,min_lon,imin_lon);
		proc_leave();

		dg[0] = ST_LATLON;					// 0x158
		dg[1] = 0x5 | Z1 | Z2;
		dg[2] = i_lat;
		dg[3] = (imin_lat >> 8) & 0xff;
		dg[4] = imin_lat & 0xff;
		dg[5] = i_lon;
		dg[6] = (imin_lon >> 8) & 0xff;
		dg[7] = imin_lon & 0xff;
		queueDatagram(port2,dg);
	}
	
	
	//------------------------------------------
	// COG/SOG
	//------------------------------------------

	if (1)
	{
		float degrees = boat_sim.makeMagnetic(boat_sim.getCOG());

		int halfs_total = roundf(degrees * 2.0);
		int nineties = halfs_total / 180;
		int rem = halfs_total % 180;
		int twos = rem / 4;
		int halfs = rem % 4;

		display(dbg_data,"st%d COG(%0.1f) = nineties(%d) twos(%d) halfs(%d)",port2,degrees,nineties,twos,halfs);

		dg[0] = ST_COG;		// 0x153
		dg[1] = 0 | (nineties << 4) | (halfs<<6);
		dg[2] = twos;
		queueDatagram(port2,dg);

		//-----------------------

		double speed = boat_sim.getSOG();
		int ispeed = (speed+ 0.05) * 10;
		display(dbg_data,"st%d SOG & stSOG(%0.1f)",port2,speed);

		dg[0] = ST_SOG;		// 0x152
		dg[1] = 0x01;
		dg[2] = ispeed & 0xff;
		dg[3] = (ispeed >> 8) & 0xff;
		queueDatagram(port2,dg);
	}


	//------------------------------------------
	// DATE and TIME
	//------------------------------------------

	if (1)
	{
		int y = year() % 100;
		int m = month();
		int d = day();

		display(dbg_data,"st%d Date(%02d/%02d/%02d)",port2,y,m,d);
		dg[0] = ST_DATE;			// 0x156
		dg[1] = 0x01 | (m << 4);
		dg[2] = d;
		dg[3] = y;
		queueDatagram(port2,dg);

		// RST is 12 bits (6 bits for minute, 6 bits for second)
		// T is four bits (low order four bits of second)
		// RS is eight bits (6 bits of minute followed by 2 bits of second)

		int s = second();
		int h = hour();
		int mm = minute();

		uint16_t RST = (mm << 6) | s;
		uint16_t T = RST & 0xf;
		uint16_t RS = RST >> 4;

		display(dbg_data,"st%d Date(%02d:%02d:%02d)",port2,h,mm,s);
		dg[0] = ST_TIME;				// 0x154
		dg[1] = 0x01 | (T << 4);
		dg[2] = RS;
		dg[3] = h;
		queueDatagram(port2,dg);
	}

	proc_leave();

}	// gpsInst()



void aisInst::sendSeatalk(bool port2)
{
}




void apInst::sendSeatalk(bool port2)
	// CARE MUST BE TAKEN on the real boat when sending ANY information
	// about, or to, the actual autopilot computer!!!!!
	//
	// Note that the Seatalk Autopilot "instrument" does thinga that
	// are not currently implemented in the NMEA0183/NMEA2000 versions
	// in order to jive/test the ST7000 Autopilot control head.
	//
	// 		- It sends datagrams when m_autopilot == 0 == AP_MODE_OFF == STANDBY
	//		- It knows about the nascent AP_MODE_VANE == VANE mode
	// 		- It can be compiled to "listen" for the ST_ST7000 0x197 message and
	//		  transmit the associated ST_AP_CPU 0x198 message, via the ap_linked
	//		  global communication variable set by instST_in.cpp
	//	    - It *may* grow to provide more emulation of the 400 ap_cpu to
	//		  allow for emulating modal behavior like calibration menus, etc
	//
	// Note that the ST7000 autopilot head and computer speak solely
	// in terms of MAGNETIC, not TRUE headings.
{
	sendDeviceId(port2,0x81,1,7);		// ST7000 device

	// optional code to respond to 0x197 ST_ST7000 from the head and
	// transmit 0x198 ST_AP_CPU response from the ap instrument.
	// I thought this was "necessary" at some point, but now I don't

	#if 0
		if (!ap_linked) return;
		if (ap_linked == 1)
		{
			dg[0] = ST_AP_CPU;	// 0x198;
			dg[1] = 0x00;
			dg[2] = 0x00;
			queueDatagram(port2,dg);
			sendDatagram(port2);
			ap_linked++;
			return;
		}
	#endif

	uint8_t ap_mode = boat_sim.getAutopilot();
	bool	routing = boat_sim.getRouting();

	//-----------------------------------------------------------------------
	// ST_AUTOPILOT and ST_RUDDER datagrams regardless if autopilot engaged
	//-----------------------------------------------------------------------
	// KNAUFS DESCRIPTION:
	//
	// 84  U6  VW  XY 0Z 0M RR SS TT  Compass heading  Autopilot course and
	//     Rudder position (see also command 9C)
	//     Compass heading in degrees:
	//       The two lower  bits of  U * 90 +
	//       the six lower  bits of VW *  2 +
	//       number of bits set in the two higher bits of U =
	//       (U & 0x3)* 90 + (VW & 0x3F)* 2 + (U & 0xC ? (U & 0xC == 0xC ? 2 : 1): 0)
	//     Turning direction:
	//       Most significant bit of U = 1: Increasing heading, Ship turns right
	//       Most significant bit of U = 0: Decreasing heading, Ship turns left
	//     Autopilot course in degrees:
	//       The two higher bits of  V * 90 + XY / 2
	//     Z & 0x2 = 0 : Autopilot in Standby-Mode
	//     Z & 0x2 = 2 : Autopilot in Auto-Mode
	//     Z & 0x4 = 4 : Autopilot in Vane Mode (WindTrim), requires regular "10" datagrams
	//     Z & 0x8 = 8 : Autopilot in Track Mode
	//     M: Alarms + audible beeps
	//       M & 0x04 = 4 : Off course
	//       M & 0x08 = 8 : Wind Shift
	//     Rudder position: RR degrees (positive values steer right,
	//       negative values steer left. Example: 0xFE = 2° left)
	//     SS & 0x01 : when set, turns off heading display on 600R control.
	//     SS & 0x02 : always on with 400G
	//     SS & 0x08 : displays “NO DATA” on 600R
	//     SS & 0x10 : displays “LARGE XTE” on 600R
	//     SS & 0x80 : Displays “Auto Rel” on 600R
	//     TT : Always 0x08 on 400G computer, always 0x05 on 150(G) computer
	//
	// MY NOTES:
	//
	//	Knauf's description was probably written before the ST7000 came out
	//
	//		(1)	THE HIGH NIBBLE OF THE Z BYTE MUST BE 4 (i.e. 4Z) for
	//			the ST7000!! Knauf uses "0Z" in his header line, implying
	//			that the high order nibble of the Z byte should be zero.
	//			THIS DOES NOT WORK WITH THE ST7000 and it took me a full
	//			day to figure it out. My 400G ap-cpu always sends '4Z'.
	//			If the high nibble of the Z BYTE is zero, the ST7000 DOES
	//			NOT DISPLAY THE RUDDER INDICATOR.
	//		(2) I believe there IS NO "RIGHT" bit. Knauf's description
	//			clearly overuses the high order bit of the U nibble when
	//			he also says "number of bits set in the two higher bits of U =
	//       	(U & 0x3)* 90 + (VW & 0x3F)* 2 + (U & 0xC ? (U & 0xC == 0xC ? 2 : 1): 0)".
	//			I now believe that the encoding is simpler, and the same
	//			as the ST_HEADING encoding above. That:
	//
	//			THE TWO HIGH BITS OF U ENCODE THE NUMBER OF HALF DEGREES
	//			IN THE HEADING after the six low bits of VW encoded the
	//			number of "twos" in the heading.
	//
	//			This *may* be a ST7000/E80 genrerational issue, or perhaps
	//			more likely, Knauf just got it completely wrong.  In any case,
	//			treating those bits as half degrees works with the E80 AND the
	//			ST7000 when the E80 was NOT working correctly with the vestiges
	//			of Knauf's description.

	// set the mode_byte

	uint8_t mode_byte =						// 0x4Z mode byte Z nibble
		routing ? 0xA :					    // routing == AUTO | TRACK mode
		ap_mode == AP_MODE_VANE ? 0x6 :		// vane = AUTO | VANE mode
		ap_mode == AP_MODE_AUTO ? 0x2 :		// autopilot engaged == AUTO mode
		0;									// STANDBY mode
	mode_byte |= 0x40;						// THIS IS REQUIRED FOR THE ST7000 to show the Rudder Bar

	// get the Rudder position. It is up to the boatSimulator
	// to modify it realistically if the autopilot is engaged

	int rudder = boat_sim.getRudder();
	int8_t rr8 = (int8_t)rudder;     // signed 8 bit

	// copied from ST_HEADING
	// get true hading, convert to magnetic version via getMagneticVariance
	// round to one decimal place

	float heading = boat_sim.makeMagnetic(boat_sim.getHeading());
	double f_heading = roundf(heading * 10.0f) / 10.0f;

	// calculate weird ninetees, twos, and halfs encoding

	int h_halfs = f_heading * 2;
	int h_nineties = h_halfs / 180;
	h_halfs = h_halfs - (h_nineties * 180);
	int h_twos = h_halfs / 4;
	h_halfs = h_halfs - (h_twos * 4);

	// calculate ap_course in ninetees and halfs if ap engaged

	int a_ninetees = 0;
	int a_halfs    = 0;
	if (ap_mode)
	{
		double ap_course = boat_sim.getDesiredHeading();
		ap_course += boat_sim.getMagneticVariance();
		if (ap_course > 360.0) ap_course = ap_course - 360.0;
		a_halfs = ap_course * 2;
		a_ninetees = a_halfs / 180;
		a_halfs = a_halfs - (a_ninetees * 180);
	}

	// The 500G autopilot cpu sends the ST_RUDDER message first

	if (1)
	{
		dg[0] = ST_RUDDER;									// 0x19c
		dg[1] = 1 | (h_nineties << 4) | (h_halfs<<6);		// U1
		dg[2] = h_twos;										// VW
		dg[3] = (uint8_t)rr8;;								// RR
		queueDatagram(port2,dg);
	}

	// Then sends the ST_AUTOPILOT message

	if (1)
	{
		dg[0] = ST_AUTOPILOT;								// 0x184;
		dg[1] = 6 | (h_nineties << 4) | (h_halfs<<6);		// U6
		dg[2] = h_twos | (a_ninetees << 6);					// VW with the ap_course ninetiees
		dg[3] = a_halfs;									// XY = halfs
		dg[4] = mode_byte;									// 4Z = mode_byte.
		dg[5] = 0x00;										// 0M (no alarms)
		dg[6] = (uint8_t)rr8;								// RR
		dg[7] = 0x00;       								// SS always zero in my case
		dg[8] = 0x08;										// TT always zero in my case
		queueDatagram(port2,dg);
	}


	//------------------------------------------------------------
	// ST_NAV_TO_WP, ST_TARGET_NAME, and ST_ARRIVAL only if routing
	//------------------------------------------------------------
	// routing == TRACK mode
	// getDesiredHeading() returns headingToWaypoint()

	if (routing)
	{
		if (1)	
		{
			// ST_NAV_TO_WP	0x185 "should be sent before ST_TARGET_NAME	0x182"
			//	85  X6  XX  VU ZW ZZ YF 00 yf   Navigation to waypoint information
			//		Cross Track Error: XXX/100 nautical miles
			//			Example: X-track error 2.61nm => 261 dec => 0x105 => X6XX=5_10
			//		Bearing to destination: (U & 0x3) * 90° + WV / 2°
			//			Example: GPS course 230°=180+50=2*90 + 0x64/2 => VUZW=42_6
			//		U&8: U&8 = 8 -> Bearing is true, U&8 = 0 -> Bearing is magnetic
			//		Distance to destination:
			//			Distance 0-9.99nm: ZZZ/100nm, Y & 1 = 1
			//			Distance >=10.0nm: ZZZ/10 nm, Y & 1 = 0
			//		Direction to steer:
			//			if Y & 4 = 4 Steer right to correct error
			//			if Y & 4 = 0 Steer left  to correct error
			//		Example:
			//			Distance = 5.13nm, steer left: 5.13*100 = 513 = 0x201 => ZW ZZ YF=1_ 20 1_
			//			Distance = 51.3nm, steer left: 51.3*10  = 513 = 0x201 => ZW ZZ YF=1_ 20 0_
			//		F contains four flags which indicate the available data fields:
			//				Bit 0 (F & 1): XTE present
			//				Bit 1 (F & 2): Bearing to destination present
			//				Bit 2 (F & 4): Range to destination present
			//				Bit 3 (F & 8): XTE >= 0.3nm
			//			These bits are used to allow a correct translation from for instance an RMB sentence which
			//			contains only an XTE value, all other fields are empty. Since SeaTalk has no special value
			//			for a data field to indicate a "not present" state, these flags are used to indicate the
			//			presence of a value.
			//		In case of a waypoint change, sentence 85, indicating the new bearing and distance,
			//		should be transmitted prior to sentence 82 (which indicates the waypoint change).
			//		Corresponding NMEA sentences: RMB, APB, BWR, BWC, XTE
			//
			// PRH NOTES
			//
			//	- we dont set 0x8 int0 U nibble as we are using a magnetic bearing
			//  - we do not currently ever set the 'right' 0x4 bit into Y

			uint16_t XXX = round(boat_sim.getCrossTrackError() * 100);		// xte_error in hundredths

			int b_halfs = f_heading * 2;									// bearing halfs based on d_heading == desired heading set by ap simulator
			int b_nineties = b_halfs / 180;									// bearing ninetees
			b_halfs = b_halfs - (b_nineties * 180);							// adjust bearing halfs minus nineties

			uint8_t Y = 0;													// distance is in 10'ths of NM
			double dist = boat_sim.distanceToWaypoint();
			uint8_t ZZZ = round(dist * 10.0);								// 10ths of NMs
			if (dist < 10.0)
			{
				Y |= 1;														// distance < 10.0 is in 100ths of an NM
				ZZZ = round(dist * 100.0);									// 100's of NMs
			}


			uint8_t F = 0x01 | 0x02 | 0x04;									// XTE, bearing, distance present
			if (XXX > 300) F |= 0x08;										// 0x08 set if XTE > 0.3nm

			uint8_t VU = ((b_halfs & 0x0f) << 4) | b_nineties;				// VU = V=bottom four bits of b_halfs; top two bits of U=0 as its magnetic, and lower two bits are the b_ninetees
			uint8_t ZW = ((b_halfs & 0xf0) >> 4) | ((ZZZ & 0x0f) << 4);  	// ZW = W=high four bits of WV=b_halfs and low 4 bits of ZZZ
			uint8_t ZZ = ZZZ >> 4;											// ZZ = high 8 bits of ZZZ

			dg[0] = ST_NAV_TO_WP;
			dg[1] = 6 | ((XXX & 0xf)<<4);					// X6  X = low 4 bits of xte_hundreths
			dg[2] = XXX >> 4;								// XX  XX = high 8 bits of xte_hundreths
			dg[3] = VU;										// VU = V=bottom four bits of WV=b_halfs and bottom (all) four bits of U=>0x0_ mag flag, and b_ninetees
			dg[4] = ZW;										// ZW = W=high four bits of WV and low 4 bits of ZZZZ
			dg[5] = ZZ;										// ZZ = high 8 bits of ZZZ
			dg[6] = (Y<<4) | F;								// YF = 4 bits of Y and 4 bits of F
			dg[7] = 0xff - dg[6];							// yf = undocumented presumed checksum
			queueDatagram(port2,dg);
		}
		

		// get the waypoint structure and massage the name
		// for use in ST_TARGET NAME and ST_ARRIVAL blocks

		int wp_num = boat_sim.getTargetWPNum();
		const waypoint_t *wp = boat_sim.getWaypoint(wp_num);
		String name(wp->name);
		int len = name.length();
		while (len < 4)
		{
			name += '0';
			len++;
		}
		int at = len-4;
		String name4 = name.substring(at,at+4);
		name4 = name4.toUpperCase();

		display(dbg_data,"st%d TargetName(%s) name4(%s)",port2,name.c_str(),name4.c_str());

		if (1)
		{
			// #define ST_TARGET_NAME	0x182
			//	82  05  XX  xx YY yy ZZ zz   Target waypoint name
			//		XX+xx = YY+yy = ZZ+zz = FF (allows error detection)
			//		Takes the last 4 chars of name, assumes upper case only
			//		Char= ASCII-Char - 0x30
			//		XX&0x3F: char1
			//		(YY&0xF)*4+(XX&0xC0)/64: char2
			//		(ZZ&0x3)*16+(YY&0xF0)/16: char3
			//		(ZZ&0xFC)/4: char4
			//		Corresponding NMEA sentences: RMB, APB, BWR, BWC

			// another weird 6 bit character encoding
			// with additionally weird checksumming

			uint8_t chars[4];	// 4 chars of 6 bits each of (c - 0x30)
			for (int i=0; i<4; i++)
			{
				uint8_t c = name4.charAt(i);
				chars[i] = (c - 0x30) & 0x3f;
			}

			uint8_t XX = (chars[1] % 4) << 6 | (chars[0] & 0x3f);
			uint8_t YY = ((chars[2] % 16) << 4) | (chars[1] / 4);
			uint8_t ZZ = (chars[3] << 2) | (chars[2] / 16);

			uint8_t xx = 0xff - XX;
			uint8_t yy = 0xff - YY;
			uint8_t zz = 0xff - ZZ;

			dg[0] = ST_TARGET_NAME;
			dg[1] = 0x05;
			dg[2] = XX;
			dg[3] = xx;
			dg[4] = YY;
			dg[5] = yy;
			dg[6] = ZZ;
			dg[7] = zz;
			queueDatagram(port2,dg);
		}

		if (1)
		{
			// #define ST_ARRIVAL		0x1A2
			//	A2  X4  00  WW XX YY ZZ Arrival Info
			//		X&0x2=Arrival perpendicular passed, X&0x4=Arrival circle entered
			//		WW,XX,YY,ZZ = Ascii char's of waypoint id.   (0..9,A..Z)
			//			Takes the last 4 chars of name, assumes upper case only
			//		Corresponding NMEA sentences: APB, AA

			// What I see is a bit different.
			// It looks like WW XX YY ZZ follow X4 immediately
			// and there is an extra flag byte at the end.

			dg[0] = ST_ARRIVAL;
			dg[1] = boat_sim.getArrived() ? 0x64 : 0x04;			// both arrival types
			dg[2] = 0;
			dg[3] = name4.charAt(0); // - 0x30;
			dg[4] = name4.charAt(1); // - 0x30;
			dg[5] = name4.charAt(2); // - 0x30;
			dg[6] = name4.charAt(3); // - 0x30;
			queueDatagram(port2,dg);
		}
	}	// routing = TRACK mode

	
}	// apInst::sendSeatalk()




void engInst::sendSeatalk(bool port2)
{
	// not really supported
	int rpm = boat_sim.getRPM();
	display(dbg_data,"st% RPM(%d)",port2,rpm);
	if (rpm > 4000)
		rpm = 4000;
	dg[0] = ST_RPM;
	dg[1] = 0x03;
	dg[2] = 0;	// engine number
	dg[3] = rpm / 256;
	dg[4] = rpm % 256;
	dg[5] = 0x08;	// default 8 degree pitch
	queueDatagram(port2,dg);
}


void genInst::sendSeatalk(bool port2)
{
	// not supported
}




// end of instST_out.cpp
