//---------------------------------------------
// instST_out.cpp
//---------------------------------------------
// Implementation of Seatalk instruments.

#include "instST.h"
#include "instSimulator.h"
#include "boatSimulator.h"
#include <myDebug.h>

#define dbg_data 		(instruments.g_MON_OUT ? 0 : 1)


#define SEND_OK			0
#define SEND_BUSY 		1
#define SEND_COLLISION	2
#define SEND_ERROR		3

#define IDLE_BUS_MS				10		// ms bus must be idle to send next datagram
#define SEND_DELAY				50		// ms between each datagram
#define NUM_RETRIES				6

uint32_t last_seatalk_receive_time;


static uint16_t dg[MAX_ST_SEEN];
static int retry_num = 0;


static int sendDatagram(const uint16_t *dg)
{
	if (millis() - last_seatalk_receive_time < IDLE_BUS_MS)
		return SEND_BUSY;	// bus busy

	#define WRITE_TIMEOUT	40

			uint8_t echo_dg[8];
			for (int i=0; i<8; i++)
			{
				echo_dg[i] = dg[i];
			}
			showDatagram(true,echo_dg);


	//	if (show_output)
	//	{
	//		Serial.print("--> [");
	//		Serial.print(dg_num);
	//		Serial.print(",");
	//		Serial.print(retry_num);
	//		Serial.print("]");
	//	}

	int len = (dg[1] & 0xf) + 3;
	for (int i=0; i<len; i++)
	{
		bool ok = false;
		int out_byte = dg[i];

		//	if (show_output)
		//	{
		//		Serial.print(" 0x");
		//		Serial.print(out_byte,HEX);
		//	}

		uint32_t sent_time = millis();
		SERIAL_SEATALK.write9bit(out_byte);
		while (!ok)
		{
			if (SERIAL_SEATALK.available())
			{
				int in_byte = SERIAL_SEATALK.read();
				if (in_byte == out_byte)
				{
					ok = true;
				}
				else
				{
					// if (show_output)
					// 	Serial.println();
					warning(0,"collision(%d)",retry_num);
					return SEND_COLLISION;
				}
			}
			else if (millis() - sent_time >= WRITE_TIMEOUT)
			{
				// if (show_output)
				// 	Serial.println();
				my_error("WRITE_TIMEOUT",0);
				return SEND_ERROR;
			}
		}
	}

	// if (show_output)
	// 	Serial.println();


	return SEND_OK;
}




//-------------------------------------
// instruments
//-------------------------------------


void depthInst::sendSeatalk()
{
	uint16_t d10;
	if (boat.getDepth() > 999)
		d10 = 9990;
	else
		d10 = (boat.getDepth() + 0.05) * 10.0;

	dg[0] = ST_DEPTH;
	dg[1] = 0x02;
	dg[2] = 0;	// flags
	dg[3] = d10 & 0xff;
	dg[4] = (d10 >> 8) & 0xff;

	sendDatagram(dg);
}

void logInst::sendSeatalk()
{
	double speed = boat.getSOG();
	int ispeed = (speed+ 0.05) * 10;
	display(dbg_data,"stWaterSpeed & stSOG(%0.1f)",speed);

	dg[0] = ST_WATER_SPEED;
	dg[1] = 0x01;
	dg[2] = ispeed & 0xff;
	dg[3] = (ispeed >> 8) & 0xff;
	sendDatagram(dg);

	dg[0] = ST_SOG;
	sendDatagram(dg);
}



void windInst::sendSeatalk()
{
	double speed = boat.apparentWindSpeed();	// getWindSpeed();
	double angle = boat.apparentWindAngle();	// getWindAngle();

	int tenths = (speed + 0.05) * 10.0;
	int ispeed = tenths / 10;
	tenths = tenths % 10;

	display(dbg_data,"stWindSpeed(%0.1f) ispeed=%d tenths=%d",speed,ispeed,tenths);

	dg[0] = ST_WIND_SPEED;
	dg[1] = 0x01;
	dg[2] = (ispeed & 0x7f);		// high order bit 0=knots; 1 would be fathoms
	dg[3] = tenths;
	sendDatagram(dg);

	display(dbg_data,"stWindAngle(%0.1f)",angle);

	int a2 = angle * 2;
	dg[0] = ST_WIND_ANGLE;
	dg[1] = 0x01;
	dg[2] = (a2 >> 8) & 0xff;
	dg[3] = a2 & 0xff;
	sendDatagram(dg);
}


void compassInst::sendSeatalk()
{
	// what a weird encoding
	// .... ........
	// HH99   222222

	double degrees = boat.getCOG();
	int idegrees = degrees;
	int nineties = idegrees / 90;
	idegrees = idegrees - (nineties * 90);
	int twos = idegrees / 2;
	idegrees = idegrees - (twos * 2);
	int halfs = idegrees * 2;

	display(dbg_data,"stCOG & stHeading(%0.1f) = nineties(%d) twos(%d) halfs(%d)",degrees,nineties,twos,halfs);

	dg[0] = ST_COG;
	dg[1] = 0 | (nineties << 4) | (halfs<<6);
	dg[2] = twos;
	sendDatagram(dg);

	dg[0] = ST_HEADING;
	dg[1] = 2 | (nineties << 4) | (halfs<<6);
	dg[2] = twos;
	dg[3] = 0x00;	// unuaed by me at this time
	dg[4] = 0x20;	// unuaed by me at this time
	sendDatagram(dg);

}



void gpsInst::sendSeatalk()
{
	double lat = boat.getLat();
	double lon = boat.getLon();
	display(dbg_data,"stLatLon(%0.6f,%0.6f)",lat,lon);

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
	display(dbg_data,"i_lat(%d) frac_lat(%0.6f) min_lat(%0.6f) imin_lat(%d)",i_lat,frac_lat,min_lat,imin_lat);
	display(dbg_data,"i_lon(%d) frac_lon(%0.6f) min_lon(%0.6f) imin_lon(%d)",i_lon,frac_lon,min_lon,imin_lon);
	proc_leave();

	dg[0] = ST_LATLON;
	dg[1] = 0x5 | Z1 | Z2;

	dg[2] = i_lat;
	dg[3] = (imin_lat >> 8) & 0xff;
	dg[4] = imin_lat & 0xff;

	dg[5] = i_lon;
	dg[6] = (imin_lon >> 8) & 0xff;
	dg[7] = imin_lon & 0xff;
	sendDatagram(dg);
}


void aisInst::sendSeatalk()
{
}

void autopilotInst::sendSeatalk()
{
	// as somewhat expected, as with NMEA2000, these messages
	// don't seem to affect the E80's notion of "following" or XTE values.
	// I'll have to check this out sometime with the real autopilot
	
	if (boat.getAutopilot())
	{
		int wp_num = boat.getTargetWPNum();
		const waypoint_t *wp = boat.getWaypoint(wp_num);

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

			double head = boat.headingToWaypoint();
			double dist = boat.distanceToWaypoint();
			uint16_t xte_hundreths = 123;

			display(0,"stNavToWp head(%0.1f) dist(%0.3f) xte(%0.2f)",head,dist,((float) xte_hundreths)/100.0);
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

			display(0,"h90s(%d) halfs(%d) head rebuilt(%0.1f)",
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

			sendDatagram(dg);

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


			display(0,"stTargetName(%s) name4(%s)",name.c_str(),name4.c_str());

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
			sendDatagram(dg);
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
			dg[1] = boat.getArrived() ? 0x64 : 0x04;			// both arrival types
			dg[2] = 0;
			dg[3] = name4.charAt(0); // - 0x30;
			dg[4] = name4.charAt(1); // - 0x30;
			dg[5] = name4.charAt(2); // - 0x30;
			dg[6] = name4.charAt(3); // - 0x30;
			sendDatagram(dg);
		}


	}
}

void engineInst::sendSeatalk()
{
	// not really supported
	int rpm = boat.getRPM();
	display(0,"stRPM(%d)",rpm);
	if (rpm > 4000)
		rpm = 4000;
	dg[0] = ST_RPM;
	dg[1] = 0x03;
	dg[2] = 0;	// engine number
	dg[3] = rpm / 256;
	dg[4] = rpm % 256;
	dg[5] = 0x08;	// default 8 degree pitch
	sendDatagram(dg);
}

void gensetInst::sendSeatalk()
{
	// not supported
}


// INST_CLOCK
//

//	void stTime(uint16_t *dg)	// GMT
//	{
//		uint32_t elapsed = millis() - time_init;
//		int hour_counter = (loop_counter / 10) % 24;
//			// every 10 seconds we increment the hour
//			// to help find all time bytes in raynet.pm
//
//		int secs = elapsed/1000  + hour_counter * 3600;
//		int hour = secs / 3600;
//		int minute = (secs - hour * 3600) / 60;
//		secs = secs % 60;
//
//		if (hour > 23)
//			hour = 0;
//
//		// RST is 12 bits (6 bits for minute, 6 bits for second)
//		// T is four bits (low order four bits of second)
//		// RS is eight bits (6 bits of minute followed by 2 bits of second)
//
//		uint16_t RST = (minute << 6) | secs;
//		uint16_t T = RST & 0xf;
//		uint16_t RS = RST >> 4;
//
//		dg[0] = ST_TIME;
//		dg[1] = 0x01 | (T << 4);
//		dg[2] = RS;
//		dg[3] = hour;
//	}
//
//
//	void stDate(uint16_t *dg)
//	{
//		int year = time_year;
//		int month = time_month;
//		int day = time_day;
//
//		display(dbg_data,"stDate(%02d/%02d/%02d)",month,day,year);
//		dg[0] = ST_DATE;
//		dg[1] = 0x01 | (month << 4);
//		dg[2] = day;
//		dg[3] = year;
//
//		// if (loop_counter && loop_counter % 10 == 0)
//		//	time_day = time_day == 1 ? 2 : 1;
//
//	}


// end of instST_out.cpp
