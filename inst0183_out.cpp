//---------------------------------------------
// inst0183_out.cpp
//---------------------------------------------
// Implementation of simulated NMEA0183 instruments.

#include "inst0183.h"
#include "instSimulator.h"
#include "boatSimulator.h"
#include "boatBinary.h"
#include <myDebug.h>

#define show_0183 (1-g_MON_0183)


#define MAX_NMEA_MSG	180
	// my buffer size is way bigger than official NMEA0183
	// maximum of 80 bytes
static char nmea_buf[MAX_NMEA_MSG+1];



#define BINARY_BUF_SIZE		200
#define SEND_NMEA0183_AS_BINARY		1
	// send these messages out as binary so teensyBoat.pm can
	// forward them to a VSPE COM29 for RNS

static void sendNMEA0183()
{
	if (SEND_NMEA0183_AS_BINARY)
	{
		uint8_t buf[BINARY_BUF_SIZE+1];
		int offset = startBinary(buf,BINARY_TYPE_0183);
		offset = binaryVarStr(buf,offset,nmea_buf,BINARY_BUF_SIZE);
		endBinary(buf,offset);
		// display(0,"sending type(%d) %d bytes to binary serial port",BINARY_TYPE_0183,offset);
		Serial.write(buf,offset);
	}
	// else
	{
		SERIAL_0183.println(nmea_buf);
	}
}





static void checksum()
{
	char *ptr = &nmea_buf[1]; 	// skip the leading $
	unsigned char byte = *ptr++;
	while (*ptr)
	{
		byte ^= *ptr++;
	}
	sprintf(ptr,"*%02x",byte);
}


static const char *standardDate()
	// returns constant fake date
{
	// return "100525";	// 2025-05-10
	int year = boat.getYear();
	year = year > 2000 ? year-2000 : 0;
	int month = boat.getMonth();
	int day = boat.getDay();
	static char time_buf[20];
	sprintf(time_buf,"%02d%02d%02d",day,month,year);
	return time_buf;
}


static const char *standardTime()
	// returns time of day as millis() since boot
{
	// int secs = 12*3600 + millis()/1000;
	// int hour = secs / 3600;
	// int minute = (secs - hour * 3600) / 60;
	// secs = secs % 60;
	// if (hour > 23)
	// 	hour = 0;

	int hour = boat.getHour();
	int minute = boat.getMinute();
	int secs = boat.getSecond();

	static char time_buf[20];
	sprintf(time_buf,"%02d%02d%02d.00",hour,minute,secs);
	return time_buf;
}


static const char *standardLat(double latitude)
	// returns NMEA0183 formated "latitude,N/S" with comma
{
	char ns = 'N';
	if (latitude < 0)
	{
		latitude = abs(latitude);
		ns = 'S';
	}
	int d_lat = latitude;
	double m_lat = (latitude - d_lat) * 60.0;

	static char lat_buf[20];
	sprintf(lat_buf,"%02d%07.4f,%c",d_lat,m_lat,ns);
		// note %0X.Yf print spec is for Y decimal
		// places padded with zeros to overal length X
	return lat_buf;
}


static const char *standardLon(double longitude)
	// returns NMEA0183 formated "longitude,N/S" with comma
{
	char ew = 'W';
	if (longitude < 0)
	{
		longitude = abs(longitude);
		ew = 'W';
	}
	int d_lon = longitude;
	double m_lon = (longitude - d_lon) * 60.0;

	static char lon_buf[20];
	sprintf(lon_buf,"%03d%07.4f,%c",d_lon,m_lon,ew);
	return lon_buf;
}



//-----------------------------
// instruments
//-----------------------------

void depthInst::send0183()
	// SD = Sounder device
	// DPT = Water Depth
{
	double d_meters = boat.getDepth() * FEET_TO_METERS;
	// 1) Depth, meters
	// 2) Offset from transducer;
	// 	  positive means distance from transducer to water line,
	// 	  negative means distance from transducer to keel
	//                       1     2
	sprintf(nmea_buf,"$SDDPT,%0.1f,0.0",d_meters);
	checksum();
	display(show_0183,"depthInst --> %s",nmea_buf);
	sendNMEA0183();
}


void logInst::send0183()
{
	// VW = Velocity Sensor, Speed Log, Water, Mechanical
	// VHW device
	// The sentence expects the RELATIVE degrees as True or Magnetic !?!
	// We only send the 'T' true degrees and speed in knots
	// Our simulator currently assumes no current, so
	// 		water speed = sog
	// 		water_heading = cog, which may be off by 180, not sure
	//
	// 1) Degress True
	// 2) T = True
	// 3) Degrees Magnetic
	// 4) M = Magnetic
	// 5) Knots (speed of vessel relative to the water)
	// 6) N = Knots
	// 7) Kilometers (speed of vessel relative to the water)
	// 8) K = Kilometres
	//
	// OLD:
	//	                         1     2 345     678
	//	sprintf(nmea_buf,"$VWVHW,%0.1f,T,,,%0.1f,N,,",

	//                       12345     678
	sprintf(nmea_buf,"$VWVHW,,,,,%0.1f,N,,",
		// boat.getCOG(),	// log instrument does not send a heading!
		boat.getWaterSpeed());
	checksum();
	display(show_0183,"logInst --> %s",nmea_buf);
	sendNMEA0183();
}


void windInst::send0183()
{
	double heading = boat.getHeading();
	double bow_angle_true = boat.getWindAngle() - heading;
	if (bow_angle_true < 0) bow_angle_true += 360;
	
	// WI = Weather Instruments
	// MWV = Wind Speed and Angle
	// the E80 does not not know VWR - Relative Wind Speed and Angle
	// sentence MWV device WI = Weather Instruments
	// 		can send (T)heoretical true or (R)elative
	// until cog, sog, and water speed are set
	//		the E80 does not calculate the other winds for us
	// note that even with T the degrees are passed relative to bow
	//
	// 1) Wind Angle, 0 to 360 degrees
	// 2) Reference, R = Relative, T = Theoretical true
	// 3) Wind Speed
	// 4) Wind Speed Units, K/M/N
	// 5) Status, A = Data Valid
	//                       1     2  3     4 5
	sprintf(nmea_buf,"$WIMWV,%0.1f,T,%0.1f,N,A",
		bow_angle_true,
		boat.getWindSpeed());
	checksum();
	display(show_0183,"windInst --> %s",nmea_buf);
	sendNMEA0183();

	// apparentWindAngle() is ALREADY measured relative to the bow!

	sprintf(nmea_buf,"$WIMWV,%0.1f,R,%0.1f,N,A",
		boat.apparentWindAngle(),
		boat.apparentWindSpeed());
	checksum();
	display(show_0183,"windInst --> %s",nmea_buf);
	sendNMEA0183();
}


void compassInst::send0183()
{
	// Device = HC (heading compass)

	if (1)
	{
		// HDT: $MCHDT,237.5,T*hh
		//		237.5 = true heading
		//		T = true
		sprintf(nmea_buf,"$HCHDT,%0.1f,T",
			boat.getHeading() );
		checksum();
		display(show_0183,"compassInst --> %s",nmea_buf);
		sendNMEA0183();
	}
	if (0)
	{
		// HDM: $HCHDM,238.0,M*hh
		// 		238.0 = magnetic heading
		// 		M = magnetic
		sprintf(nmea_buf,"$HCHDM,%0.1f,M",
			boat.getHeading() );
		checksum();
		display(show_0183,"compassInst --> %s",nmea_buf);
		sendNMEA0183();
	}
}


void gpsInst::send0183()
	//	Recommended NMEA Sentence Order
	//	For a full GPS data stream that devices like the Raymarine E80 expect, the typical order and priority is:
	//	- $GPGGA — Fix data (position, satellites used, HDOP, altitude)
	//	- $GPGSA — Satellites used in fix + DOP values
	//	- $GPGSV — Satellites in view (PRNs, elevation, azimuth, SNR)
	//	- $GPRMC — Recommended minimum data (position, speed, course, date)
	//	- $GPGLL — Latitude/Longitude (optional, often redundant with GGA/RMC)
{

	if (1)
	{
		// $GPGGA - Global Positioning System Fix Data
		// Format:
		// $GPGGA,<UTC>,<Latitude>,<N/S>,<Longitude>,<E/W>,<FixQuality>,<NumSat>,<HDOP>,<Altitude>,M,<GeoidSep>,M,,*<checksum>
		//
		// Field breakdown:
		//  1) UTC Time (hhmmss)
		//  2) Latitude (ddmm.mmmm)
		//  3) N/S Indicator
		//  4) Longitude (dddmm.mmmm)
		//  5) E/W Indicator
		//  6) Fix Quality: 0 = invalid, 1 = GPS fix, 2 = DGPS fix
		//  7) Number of satellites used (00–12 typical)
		//  8) HDOP (horizontal dilution of precision)
		//  9) Altitude above mean sea level (meters)
		// 10) 'M' = units for altitude
		// 11) Geoid separation (meters)
		// 12) 'M' = units for geoid separation
		// 13) Age of differential GPS data (blank if unused)
		// 14) DGPS reference station ID (blank if unused)
		//
		//                        1  23 45 6 7  8   9   10 11  12
		sprintf(nmea_buf, "$GPGGA,%s,%s,%s,1,03,2.2,15.3,M,0.0,M,,",
			standardTime(),
			standardLat(boat.getLat()),
			standardLon(boat.getLon())
		);
		checksum();  // Appends *hh
		sendNMEA0183();
	}

	if (1)
	{
		// $GPGSA - GNSS DOP and Active Satellites
		// Format:
		// $GPGSA,<1>,<2>,<3>...<14>,<15>,<16>,<17>*hh
		//
		// Field breakdown:
		//  1) Mode (M = Manual, A = Automatic)
		//  2) Fix type (1 = no fix, 2 = 2D fix, 3 = 3D fix)
		//  3-14) PRNs of satellites used in fix (up to 12, blank if unused)
		// 15) PDOP (Position Dilution of Precision)
		// 16) HDOP (Horizontal DOP)
		// 17) VDOP (Vertical DOP)
		// *hh) Checksum (XOR of all characters between $ and *)

		strcpy(nmea_buf, "$GPGSA,A,3,07,08,10,,,,,,,,,,1.8,2.2,2.1");
		checksum();  // Appends *hh checksum
		sendNMEA0183();
	}

	if (1)
	{
		// GSV Satellites in view
		//
		// 1) number of messages (4 sats per message)
		// 2) msg number 1 of n
		// 3) num sats
		//
		// for each sat:
		//   a = PRN number
		//   b = Elevation
		//   c = Azimuth degrees
		//   d = Signal to noise ratio
		//
		//                      1 2 3  a  b  c   d  a  b  c   d  a  b  c   d
		strcpy(nmea_buf,"$GPGSV,1,1,03,07,79,048,42,08,62,308,45,10,51,180,88");
		checksum();
		sendNMEA0183();
	}

	if (1)
	{
		// this overkill for a GPS instrument and
		// causes the E80 to send out Navigation (DBNAV) information rapidly

		// GP = GPS device
		// RMC = Recommended Minimum Navigation Information 'C'
		// "$GPRMC,092750.000,A,5321.6802,N,00630.3372,W,0.02,31.66,280511,,,A*43",
		//
		// 1) Time (UTC)
		// 2) Status, A=valid, V=Navigation receiver warning
		// 3) Latitude
		// 4) N or S
		// 5) Longitude
		// 6) E or W
		// 7) Speed over ground, knots
		// 8) Track made good, degrees true
		// 9) Date, ddmmyy
		// 10) Magnetic Variation, degrees
		// 11) E or W
		//                       1  2 34 56 7     8     9 10&11 missing
		sprintf(nmea_buf,"$GPRMC,%s,A,%s,%s,%0.1f,%0.1f,%s,,,",
			standardTime(),
			standardLat(boat.getLat()),
			standardLon(boat.getLon()),
			boat.getSOG(),
			boat.getCOG(),
			standardDate());
		checksum();
		display(show_0183,"gpsInst --> %s",nmea_buf);
		sendNMEA0183();
	}

	if (1)
	{
		// GP = Global Positioning System
		// GLL = Geographic Position - Latitude/Longitude
		// E80 takes about 10 seconds to lose this
		//
		//	1) Latitude
		//	2) N or S (North or South)
		//	3) Longitude
		//	4) E or W (East or West)
		//	5) Time (UTC)
		//	6) Status A - Data Valid, V - Data Invalid
		//                       12 34 5  6
		sprintf(nmea_buf,"$GPGLL,%s,%s,%s,A",
			standardLat(boat.getLat()),
			standardLon(boat.getLon()),
			standardTime());
		checksum();
		display(show_0183,"gpsInst --> %s",nmea_buf);
		sendNMEA0183();
	}
}


void aisInst::send0183()
{
}


void autopilotInst::send0183()
{
	// 1) Status, V = Navigation receiver warning
	// 2) Cross Track error - nautical miles
	// 3) Direction to Steer, Left or Right
	// 4) E80 unused FROM Waypoint ID
	// 5) E80 TO Waypoint ID (reversed from spec)
	// 6) Destination Waypoint Latitude
	// 7) N or S
	// 8) Destination Waypoint Longitude
	// 9) E or W
	// 10) Range to destination in nautical miles
	// 11) Bearing to destination in degrees True
	// 12) Destination closing velocity in knots
	// 13) Arrival Status, A = Arrival Circle Entered; V=reset arrival alarm
	// 14) Checksum

	double xte = 0.12;	// something I can see in 1/100's of an NM (from ST)
	char lr = 'R';
	const char *arrive_char = boat.getArrived() ? "A" : "V";
	const waypoint_t *start_wp = boat.getWaypoint(boat.getStartWPNum());
	const waypoint_t *target_wp = boat.getWaypoint(boat.getTargetWPNum());
	String start_name(start_wp->name);
	String target_name(target_wp->name);

	if (0)
	{
		// a vain attempt to get the E80 to echo a meaningful
		// waypoint name back to the ST_ARRIVAL message
		
		start_name.toUpperCase();
		target_name.toUpperCase();
		start_name = start_name.substring(2,6);
		target_name = target_name.substring(2,6);
	}

	//                       1 2     3  4  5  67 89 10    11    12    13
	sprintf(nmea_buf,"$APRMB,A,%0.3f,%c,%s,%s,%s,%s,%0.3f,%0.1f,%0.1f,%s,",
		xte,						// 2
		lr,							// 3
		start_name.c_str(),			// 4
		target_name.c_str(),		// 5
		standardLat(boat.getLat()),	// 67
		standardLon(boat.getLon()),	// 89
		boat.distanceToWaypoint(),	// 10
		boat.headingToWaypoint(),	// 11
		boat.getCOG(),				// 12
		arrive_char);				// 13

	// $APRMB,A,0.100,R,,Popa1,0920.0450,N,08214.5230,W,12.000,149.2,149.2,,*5a

	checksum();
	display(show_0183,"apInst --> %s",nmea_buf);
	sendNMEA0183();

	// E80 lights up a waypoint,
	// shows xte, name, bearing, and distance to waypoint,
	// and shows the time to waypoint if the boat is moving,
	// along with showing the WPT STOP_GOTO and RESTART_XTE buttons
	// and starts sending out these messages:
	//
	// $ECAPB,A,A,0.010,R,N,V,V,,,Popa1,000.0,T,,,A*67
	// $ECBWC,120044.49,0926.245,N,08214.523,W,000.0,T,000.0,M,6.20,N,Popa1,A*6A
	// $ECBWR,120044.49,0926.245,N,08214.523,W,000.0,T,000.0,M,6.20,N,Popa1,A*7B
	// $ECRMB,A,0.010,R,,Popa1,0926.245,N,08214.523,W,6.20,000.0,,V,A*7E
	//
	// When we start sending the "A" arrival letter at dist=0.064nm
	// the E80 starts beeping, shows the Waypoint Arrival Dialog and
	// the ACKNOWLEDGE button. When the user acknowledges, or we
	// switch to 'V' it stops.

	if (1)
	{
		// this overkill for a GPS instrument and
		// causes the E80 to send out Navigation (DBNAV) information rapidly


		// GP = GPS device
		// RMC = Recommended Minimum Navigation Information 'C'
		// "$GPRMC,092750.000,A,5321.6802,N,00630.3372,W,0.02,31.66,280511,,,A*43",
		//
		// 1) Time (UTC)
		// 2) Status, A=valid, V=Navigation receiver warning
		// 3) Latitude
		// 4) N or S
		// 5) Longitude
		// 6) E or W
		// 7) Speed over ground, knots
		// 8) Track made good, degrees true
		// 9) Date, ddmmyy
		// 10) Magnetic Variation, degrees
		// 11) E or W
		//                       1  2 34 56 7     8     9 10&11 missing
		sprintf(nmea_buf,"$GPRMC,%s,A,%s,%s,%0.1f,%0.1f,%s,,,",
			standardTime(),
			standardLat(boat.getLat()),
			standardLon(boat.getLon()),
			boat.getSOG(),
			boat.getCOG(),
			standardDate());
		checksum();
		display(show_0183,"gpsInst --> %s",nmea_buf);
		sendNMEA0183();
	}
}


void engineInst::send0183()
{
}


void gensetInst::send0183()
{
}


//--------------------------------------------------------
// experiment
//--------------------------------------------------------
// The declared maximum length of an NMEA0183 sentence is 82 bytes
// which includes everything from the leading $ to the *hh checksum
// and <cr><lf> that follows it.  We use println() to send the messages
// and call checksum separately, so our limit is down to 77 bytes.
// In addition the number of messages may grow beyond one charcter,
// so I use 70 bytes.
//
// Our nmea_buffer is only limted to MAX_NMEA_MSG=180 and has space for
// terminating zero, so it's not a factor.

#define MAX_BYTES_PER_MSG	70


void sendNMEA0183Route(String route_name)
	// We have to do two passes .. one to count the number
	// of needed messags, and a second to build them and send them.
{
	const route_t *found = boat.getRoute(route_name.c_str());
	if (!found) return;
	const char *name = found->name;
	const int num_wpts = found->num_wpts;
	const waypoint_t *wpts = found->wpts;

	display(0,"sendNMEA0183Route(%s) with %d waypoints",found->name,found->num_wpts);

	if (0)
	{
		// count the number of RTE messages needed assuming x and y are one digit
		// the 70 byte number allows for 3 digits each

		sprintf(nmea_buf,"$ECRTE,x,y,%s",found->name);
			// was $GP

		int num_msgs = 0;
		int header_len = strlen(nmea_buf);
		int message_len = header_len;
		for (int i=0; i<num_wpts; i++)
		{
			const char *wp_name = wpts[i].name;
			int wp_len = strlen(wp_name);
			if (message_len + wp_len + 1 >= MAX_BYTES_PER_MSG)	// one for leading comma
			{
				num_msgs++;
				message_len = header_len;
			}
			message_len += wp_len;
		}
		if (message_len != header_len)
			num_msgs++;

		// build and send the RTE message

		int wp_num = 0;
		for (int msg_num=1; msg_num<=num_msgs; msg_num++)
		{
			sprintf(nmea_buf,"$ECRTE,%d,%d,%s",num_msgs,msg_num,found->name);
			while (wp_num < num_wpts && strlen(nmea_buf) < MAX_BYTES_PER_MSG)
			{
				strcat(nmea_buf,",");
				strcat(nmea_buf,wpts[wp_num++].name);
			}
			checksum();
			display(0,"   --->%s",nmea_buf);
			sendNMEA0183();
		}
	}
	

	// build and send the WPL messags

	for (int i=0; i<num_wpts; i++)
	{
		const waypoint_t *wp = &wpts[i];
		sprintf(nmea_buf,"$ECWPL,%s,%s,%s",
			standardLat(wp->lat),
			standardLon(wp->lon),
			wp->name);
		checksum();
		display(0,"   ===>%s",nmea_buf);
		sendNMEA0183();
	}

	display(0,"sendNMEA0183Route(%s) finshed",found->name);

}


// end of inst0183_out.cpp
