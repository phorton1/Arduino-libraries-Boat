//---------------------------------------------
// instST_out.cpp
//---------------------------------------------
// Implementation of simulated Seatalk instruments.

#include "instST.h"
#include "instSimulator.h"
#include "boatSimulator.h"
#include "timeLib.h"
#include <myDebug.h>

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





void sendDatagram(bool port2)
{
	if (tail[port2] == head[port2])
		return;

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
			return;
		}
	}
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


//-------------------------------------
// instruments
//-------------------------------------


void depthInst::sendSeatalk(bool port2)
{
	display(dbg_data,"st%d depth(%0.1f)",port2,boat_sim.getDepth());

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
		double trip_distance = boat_sim.getTripDistance() * 100;
		uint32_t trip_int = trip_distance;;

		dg[0] = ST_TRIP;
		dg[1] = 0x02;
		dg[2] = trip_int & 0xff;
		dg[3] = (trip_int >> 8) & 0xff;
		dg[4] = (trip_int >> 16) & 0x0f;
		queueDatagram(port2,dg);
	}
	if (0)
	{
		double total_distance = boat_sim.getLogTotal() * 10;
		uint32_t total_int = total_distance;;

		dg[0] = ST_LOG_TOTAL;
		dg[1] = 0x02;
		dg[2] = total_int & 0xff;
		dg[3] = (total_int >> 8) & 0xff;
		dg[4] = 0;
		queueDatagram(port2,dg);
	}
	if (1)
	{
		double total_distance = boat_sim.getLogTotal() * 10;
		double trip_distance = boat_sim.getTripDistance() * 100;
		uint32_t total_int = total_distance;;
		uint32_t trip_int = trip_distance;;

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
	}}



void windInst::sendSeatalk(bool port2)
{
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

	int a2 = angle * 2;
	dg[0] = ST_WIND_ANGLE;
	dg[1] = 0x01;
	dg[2] = (a2 >> 8) & 0xff;
	dg[3] = a2 & 0xff;
	queueDatagram(port2,dg);
}


void compassInst::sendSeatalk(bool port2)
{
	// what a weird encoding
	// .... ........
	// HH99   222222

	double degrees = boat_sim.getHeading();
	degrees += boat_sim.getMagneticVariance();
	if (degrees > 360.0) degrees = degrees - 360.0;
		// added to send 'proper' magnetic version via ST_HEADING

	int idegrees = degrees;
	int nineties = idegrees / 90;
	idegrees = idegrees - (nineties * 90);
	int twos = idegrees / 2;
	idegrees = idegrees - (twos * 2);
	int halfs = idegrees * 2;

	display(dbg_data,"st%d Heading(%0.1f) = nineties(%d) twos(%d) halfs(%d)",port2,degrees,nineties,twos,halfs);

	// The compass only sends out a Heading (magnetic)
	// and does not know the COG
	
	dg[0] = ST_HEADING;
	dg[1] = 2 | (nineties << 4) | (halfs<<6);
	dg[2] = twos;
	dg[3] = 0x00;	// unuaed by me at this time
	dg[4] = 0x20;	// unuaed by me at this time
	queueDatagram(port2,dg);
}



void gpsInst::sendSeatalk(bool port2)
{

	double lat = boat_sim.getLat();
	double lon = boat_sim.getLon();
	display(dbg_data,"st%d LatLon(%0.6f,%0.6f)",port2,lat,lon);
	proc_entry();

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
	int imin_lat = min_lat * 1000;
	int imin_lon = min_lon * 1000;

	proc_entry();
	display(dbg_data+1,"i_lat(%d) frac_lat(%0.6f) min_lat(%0.6f) imin_lat(%d)",i_lat,frac_lat,min_lat,imin_lat);
	display(dbg_data+1,"i_lon(%d) frac_lon(%0.6f) min_lon(%0.6f) imin_lon(%d)",i_lon,frac_lon,min_lon,imin_lon);
	proc_leave();

	dg[0] = ST_LATLON;
	dg[1] = 0x5 | Z1 | Z2;

	dg[2] = i_lat;
	dg[3] = (imin_lat >> 8) & 0xff;
	dg[4] = imin_lat & 0xff;

	dg[5] = i_lon;
	dg[6] = (imin_lon >> 8) & 0xff;
	dg[7] = imin_lon & 0xff;
	queueDatagram(port2,dg);

	//------------------------------------------------

	display(dbg_data,"st%d SatInfo()",port2);

	dg[0] = ST_SAT_INFO;
	dg[1] = 0x30;	// num_sats<<4
	dg[2] = 0x02;	// hdop
	queueDatagram(port2,dg);

	uint8_t sat1_id = 0x07;
	uint8_t sat2_id = 0x08;
	uint8_t sat3_id = 0x0a;

	if (0)
	{
		// SAT_DETAIL (57)
		// if the HDOP available flag is set it counts as the low order bit of num_sats

		display(dbg_data,"st%d SatDetail(1)",port2);

		uint8_t num_sats = 3;
		dg[0] = ST_SAT_DETAIL;
		dg[1] = 0x57;			// fixed
		dg[2] = 0x17 | (num_sats & 0x7)<<4 ; // QQ=0x10=Quality available 0x7=quality; high 3 bits of num_sats in top nibble
		dg[3] = 0x83;						 // HH=top bit is HDOP available, rest of bits are HDOP
		dg[4] = 0x00;						 // ??=unknown
		dg[5] = 0x33;						 // AA=antenna height
		dg[6] = 0x20;  						 // GG=GeoSeparation = +512 meters
		dg[7] = 0x00;						 // ZZ=differential age
		dg[8] = 0x00;						 // YY=differential station ID
		dg[9] = 0x00;						 // DD=differential station id
		queueDatagram(port2,dg);

		display(dbg_data,"st%d SatDetail(2)",port2);

		// SAT_DETAIL(0x74)  (satellite IDs)
		dg[0] = ST_SAT_DETAIL;
		dg[1] = 0x74;
		dg[2] = sat1_id;	// id1
		dg[3] = sat2_id;	// id2
		dg[4] = sat3_id;	// id3
		dg[5] = 0x00;		// id4
		dg[6] = 0x00;		// id5
		queueDatagram(port2,dg);
	}

	if (0)
	{
		display(dbg_data,"st%d SatDetail(0xXd)",port2);

		// SAT_DETAIL(0xXD)  (satellite IDs)
		// Satellite 1 (PRN 07)
		uint8_t sat1_az  = 48;   // Azimuth: 48° (low eastern sky)
		uint8_t sat1_el  = 79;   // Elevation: 79° (almost overhead)
		uint8_t sat1_snr = 42;   // SNR: strong signal

		// Satellite 2 (PRN 08)
		uint8_t sat2_az  = 182;  // Azimuth: 182° (northwest)
		uint8_t sat2_el  = 62;   // Elevation: 62° (high in sky)
		uint8_t sat2_snr = 45;   // SNR: excellent signal

		// Satellite 3 (PRN 10)
		uint8_t sat3_az  = 180;  // Azimuth: 180° (due south)
		uint8_t sat3_el  = 51;   // Elevation: moderate
		uint8_t sat3_snr = 43;   // SNR: solid signal

		uint8_t NN = sat1_id & 0xFE;
		uint8_t AA = sat1_az / 2;
		uint8_t EE = (sat1_el << 1) | (sat1_az & 0x01);
		uint8_t SS = (sat1_snr << 1) & 0xFE;

		uint8_t MM = (sat2_id << 1) & 0x70;
		uint8_t BB = (sat2_id & 0x07) | ((sat2_az / 2) & 0xF8);
		uint8_t FF = ((sat2_az & 0x0F)) | ((sat2_el << 1) & 0xF0);
		uint8_t GG = ((sat2_el & 0x07) << 5) | (sat2_snr & 0x3F);
		uint8_t OO = sat2_snr & 0x3F;

		uint8_t CC = (sat3_id & 0x3F) | ((sat3_az >> 1) & 0xC0);
		uint8_t DD = sat3_az & 0x7F;
		uint8_t XX = sat3_el & 0x7F;
		uint8_t YY = (sat3_snr << 2) & 0xFC;
		uint8_t ZZ = sat3_snr & 0x01;

		dg[0] = ST_SAT_DETAIL;
		dg[1] = 0x0d;	// msg number and length
		dg[2] = NN;
		dg[4] = AA;
		dg[5] = EE;
		dg[6] = SS;
		dg[7] = MM;
		dg[8] = BB;
		dg[9] = FF;
		dg[10] = GG;
		dg[11] = OO;
		dg[12] = CC;
		dg[13] = DD;
		dg[14] = XX;
		dg[15] = YY;
		dg[16] = ZZ;
		queueDatagram(port2,dg);
	}


	//------------------------------------------

	if (1)	// GPS Instrument sends out SOG/COG
	{
		double degrees = boat_sim.getCOG();

		degrees += boat_sim.getMagneticVariance();
		if (degrees > 360.0) degrees = degrees - 360.0;
			// added to send 'proper' magnetic version via ST_COG

		int idegrees = degrees;
		int nineties = idegrees / 90;
		idegrees = idegrees - (nineties * 90);
		int twos = idegrees / 2;
		idegrees = idegrees - (twos * 2);
		int halfs = idegrees * 2;

		display(dbg_data,"st%d COG(%0.1f) = nineties(%d) twos(%d) halfs(%d)",port2,degrees,nineties,twos,halfs);

		dg[0] = ST_COG;
		dg[1] = 0 | (nineties << 4) | (halfs<<6);
		dg[2] = twos;
		queueDatagram(port2,dg);

		//-----------------------

		double speed = boat_sim.getSOG();
		int ispeed = (speed+ 0.05) * 10;
		display(dbg_data,"st%d SOG & stSOG(%0.1f)",port2,speed);

		dg[0] = ST_SOG;
		dg[1] = 0x01;
		dg[2] = ispeed & 0xff;
		dg[3] = (ispeed >> 8) & 0xff;
		queueDatagram(port2,dg);
	}


	if (1)	// GPS instrument sends date and time
	{
		int y = year() % 100;
		int m = month();
		int d = day();

		display(dbg_data,"st%d Date(%02d/%02d/%02d)",port2,y,m,d);
		dg[0] = ST_DATE;
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
		dg[0] = ST_TIME;
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
{
	// as somewhat expected, as with NMEA2000, these messages
	// don't seem to affect the E80's notion of "following" or XTE values.
	// I'll have to check this out sometime with the real autopilot
	
	if (boat_sim.getAutopilot())
	{
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

		if (1)
		{
			// ST_NAV_TO_WP	0x185 "should be sent before ST_TARGET_NAME	0x182"
			//	85  X6  XX  VU ZW ZZ YF 00 yf   Navigation to waypoint information
			//					Cross Track Error: XXX/100 nautical miles
			//					Example: X-track error 2.61nm => 261 dec => 0x105 => X6XX=5_10
			//					Bearing to destination: (U & 0x3) * 90° + WV / 2°
			//					Example: GPS course 230°=180+50=2*90 + 0x64/2 => VUZW=42_6
			//					U&8: U&8 = 8 -> Bearing is true, U&8 = 0 -> Bearing is magnetic
			//					Distance to destination: Distance 0-9.99nm: ZZZ/100nm, Y & 1 = 1
			//											Distance >=10.0nm: ZZZ/10 nm, Y & 1 = 0
			//					Direction to steer: if Y & 4 = 4 Steer right to correct error
			//										if Y & 4 = 0 Steer left  to correct error
			//					Example: Distance = 5.13nm, steer left: 5.13*100 = 513 = 0x201 => ZW ZZ YF=1_ 20 1_
			//							Distance = 51.3nm, steer left: 51.3*10  = 513 = 0x201 => ZW ZZ YF=1_ 20 0_
			//					F contains four flags which indicate the available data fields:
			//							Bit 0 (F & 1): XTE present
			//							Bit 1 (F & 2): Bearing to destination present
			//							Bit 2 (F & 4): Range to destination present
			//							Bit 3 (F & 8): XTE >= 0.3nm
			//						These bits are used to allow a correct translation from for instance an RMB sentence which
			//						contains only an XTE value, all other fields are empty. Since SeaTalk has no special value
			//						for a data field to indicate a "not present" state, these flags are used to indicate the
			//						presence of a value.
			//					In case of a waypoint change, sentence 85, indicating the new bearing and distance,
			//					should be transmitted prior to sentence 82 (which indicates the waypoint change).
			//					Corresponding NMEA sentences: RMB, APB, BWR, BWC, XTE

			double head = boat_sim.headingToWaypoint();
			double dist = boat_sim.distanceToWaypoint();
			uint16_t xte_hundreths = boat_sim.getCrossTrackError() * 100;

			display(dbg_data,"st%d NavToWp head(%0.1f) dist(%0.3f) xte(%0.2f)",port2,head,dist,((float) xte_hundreths)/100.0);
			proc_entry();

			uint8_t X6 = (xte_hundreths & 0xf) << 4;
			X6 |= 6;
			xte_hundreths >>= 4;
			uint8_t XX = xte_hundreths;

			int h90s = (head / 90.0);								// number of 90's in the heading
			double remainder = head - ((float) h90s) * 90.0;		// remaining degrees 0..90
			int halfs = ((remainder + 0.25) * 2);					// number of half degress in remainder
			uint8_t U  = 0x08 | h90s;	// 0x08=true (0x00 would equal magnetic) + number of 90's
			uint8_t WV = halfs;			// number of halfs in the bearing
			uint8_t Y = 0;				// no direction to steer 0x0f

			display(dbg_data+1,"h90s(%d) halfs(%d) head rebuilt(%0.1f)",
				h90s,
				halfs,
				(float) (((float)h90s)*90.0) + (((float)halfs)/2.0) );

			uint8_t ZZZ = (dist * 10.0);	// 10ths of NMs
			if (dist < 10.0)
			{
				Y |= 1;						// distance < 10.0 is in 100ths of an NM
				ZZZ = (dist * 100.0);		// 100's of NMs
			}

			display(dbg_data,"ZZZ(%d) in %s dist rebuilt(%0.3f)",
				ZZZ,
				Y & 1 ? "100ths" : "10ths",
				(float) ((Y&1) ? (((float)ZZZ) / 100.0) : (((float)ZZZ) / 10.0)) );
			proc_leave();

			uint8_t F = 0x01 | 0x02 | 0x04;		// XTE, bearing, distance present; 0x08 not set because XTE < 0.3nm;

			uint8_t VU = (WV << 4) | U;						// VU = V=bottom four bits of WV and bottom (all) four bits of U
			uint8_t ZW = (WV >> 4) | ((ZZZ & 0x0f) << 4);  	// ZW = W=high four bits of WV and low 4 bits of ZZZZ
			uint8_t ZZ = ZZZ >> 4;							// ZZ = high 8 bits of ZZZ

			dg[0] = ST_NAV_TO_WP;
			dg[1] = X6;						// X6  X = low 4 bits of xte_hundreths
			dg[2] = XX;						// XX  XX = high 8 bits of xte_hundreths
			dg[3] = VU;						// VU = V=bottom four bits of WV and bottom (all) four bits of U
			dg[4] = ZW;						// ZW = W=high four bits of WV and low 4 bits of ZZZZ
			dg[5] = ZZ;						// ZZ = high 8 bits of ZZZ
			dg[6] = (Y<<4) | F;				// YF = 4 bits of Y and 4 bits of F
			dg[7] = 0xff - dg[6];			// yf undocumented presumed weird checksum

			queueDatagram(port2,dg);

		}


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


			display(dbg_data,"st%d TargetName(%s) name4(%s)",port2,name.c_str(),name4.c_str());

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
			//					X&0x2=Arrival perpendicular passed, X&0x4=Arrival circle entered
			//					WW,XX,YY,ZZ = Ascii char's of waypoint id.   (0..9,A..Z)
			//									Takes the last 4 chars of name, assumes upper case only
			//					Corresponding NMEA sentences: APB, AA


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


	}
}


void sendSTCourseComputer()
{
	display(0,"sendSTCourseComputer()",0);

	#define TRY 	1
	#define TRY2 	1

	// I should just hook it up and record the conversation
	// ST7000 sends 0x97 00 00 before reporting no link

	if (TRY2)
	{
		dg[0] = 0x197;
		dg[1] = 0x10;
		dg[2] = 0x00;
		queueDatagram(false,dg);
	}

	if (TRY)
	{
		// 9C  U1  VW  RR    Compass heading and Rudder position (see also command 84)
		//                     Compass heading in degrees:
		//                       The two lower  bits of  U * 90 +
		//                       the six lower  bits of VW *  2 +
		//                       number of bits set in the two higher bits of U =
		//                       (U & 0x3)* 90 + (VW & 0x3F)* 2 + (U & 0xC ? (U & 0xC == 0xC ? 2 : 1): 0)
		//                     Turning direction:
		//                       Most significant bit of U = 1: Increasing heading, Ship turns right
		//                       Most significant bit of U = 0: Decreasing heading, Ship turns left
		//                     Rudder position: RR degrees (positive values steer right,
		//                       negative values steer left. Example: 0xFE = 2° left)
		//                     The rudder angle bar on the ST600R uses this record
		dg[0] = 0x19c;
		dg[1] = 0x01;
		dg[2] = 0x00;
		dg[3] = 0x00;
		queueDatagram(false,dg);
	}
	

	if (1)
	{
		// 84  U6  VW  XY 0Z 0M RR SS TT  Compass heading  Autopilot course and
		//                  Rudder position (see also command 9C)
		//                  Compass heading in degrees:
		//                    The two lower  bits of  U * 90 +
		//                    the six lower  bits of VW *  2 +
		//                    number of bits set in the two higher bits of U =
		//                    (U & 0x3)* 90 + (VW & 0x3F)* 2 + (U & 0xC ? (U & 0xC == 0xC ? 2 : 1): 0)
		//                  Turning direction:
		//                    Most significant bit of U = 1: Increasing heading, Ship turns right
		//                    Most significant bit of U = 0: Decreasing heading, Ship turns left
		//                  Autopilot course in degrees:
		//                    The two higher bits of  V * 90 + XY / 2
		//                  Z & 0x2 = 0 : Autopilot in Standby-Mode
		//                  Z & 0x2 = 2 : Autopilot in Auto-Mode
		//                  Z & 0x4 = 4 : Autopilot in Vane Mode (WindTrim), requires regular "10" datagrams
		//                  Z & 0x8 = 8 : Autopilot in Track Mode
		//                  M: Alarms + audible beeps
		//                    M & 0x04 = 4 : Off course
		//                    M & 0x08 = 8 : Wind Shift
		//                  Rudder position: RR degrees (positive values steer right,
		//                    negative values steer left. Example: 0xFE = 2° left)
		//                  SS & 0x01 : when set, turns off heading display on 600R control.
		//                  SS & 0x02 : always on with 400G
		//                  SS & 0x08 : displays “NO DATA” on 600R
		//                  SS & 0x10 : displays “LARGE XTE” on 600R
		//                  SS & 0x80 : Displays “Auto Rel” on 600R
		//                  TT : Always 0x08 on 400G computer, always 0x05 on 150(G) computer

		dg[0] = 0x184;
		dg[1] = 0x06;
		dg[2] = 0x00;
		dg[3] = 0x00;
		dg[4] = 0x00;
		dg[5] = 0x00;
		dg[6] = 0x00;
		dg[7] = 0x00;
		dg[8] = 0x00;
		queueDatagram(false,dg);

	}


	// from https://forum.arduino.cc/t/arduino-autohelm-6000-autopilot/644379/12
	//
	//	140 0 255 255	= 0x8c 0x00 0xff 0xff
	//	128 96 255 255  = 0x80 0x60 0xff 0xff
	//	132 174 255 255 = 0x84 0xAE 0xff 0xff
	//	138 60 255 255  = 0x8A 0x3c 0xff 0xff
	//
	// These cannot be seatalk messages du to erroneous length bytes


	// 01  05  00 00 00 60 01 00  Course Computer 400G

	if (TRY2)
	{
		dg[0] = 0x101;
		dg[1] = 0x05;
		dg[2] = 0x00;
		dg[3] = 0x00;
		dg[4] = 0x00;
		dg[5] = 0x60;
		dg[6] = 0x01;
		dg[7] = 0x00;
		queueDatagram(false,dg);
	}

	if (TRY2)
	{
		dg[0] = 0x183;	// Sent by course computer.
		dg[1] = 0x07;
		dg[2] = 0x00;
			// XX = 0 after clearing a failure condition, also sent once after power-up.
			// XX = 1 failure, auto release error. Repeated once per second.
			// XX = 8 failure, drive stopped.
		dg[3] = 0x00;
		dg[4] = 0x00;
		dg[5] = 0x00;
		dg[6] = 0x00;
		dg[7] = 0x00;
		dg[8] = 0x80;
		dg[9] = 0x00;
		dg[10] = 0x00;
		queueDatagram(false,dg);
	}
}



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
