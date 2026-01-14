//---------------------------------------------
// inst0183_out.cpp
//---------------------------------------------
// Implementation of simulated NMEA0183 instruments.

#include "inst0183.h"
#include "instSimulator.h"
#include "boatSimulator.h"
#include "boatBinary.h"
#include <myDebug.h>

// #define show_0183 (1-g_MON_0183)


#define MAX_NMEA_MSG	180
	// my buffer size is way bigger than official NMEA0183
	// maximum of 80 bytes
static char nmea_buf[MAX_NMEA_MSG+1];



#define BINARY_BUF_SIZE		200
#define SEND_NMEA0183_AS_BINARY		1
	// send these messages out as binary so teensyBoat.pm can
	// forward them to a VSPE COM29 for RNS

static void sendNMEA0183(bool portB)
{
	if (SEND_NMEA0183_AS_BINARY)
	{
		uint8_t buf[BINARY_BUF_SIZE+1];
		int offset = startBinary(buf,portB?BINARY_TYPE_0183B:BINARY_TYPE_0183A);
		offset = binaryVarStr(buf,offset,nmea_buf,BINARY_BUF_SIZE);
		endBinary(buf,offset);
		// display(0,"sending type(%d) %d bytes to binary serial port",BINARY_TYPE_0183,offset);
		Serial.write(buf,offset);
		#ifdef SERIAL_ESP32
			if (udp_enabled)
				SERIAL_ESP32.write(buf,offset);
		#endif
	}

	int port_num = portB ? PORT_83B : PORT_83A;
	bool b_mon_all = inst_sim.g_MON[port_num] & MON83_ALL;
	if (b_mon_all)
		display(0,"83%c --> %s",portB?'B':'A',nmea_buf);

	if (portB)
	{
		SERIAL_83B.println(nmea_buf);
	}
	else
	{
		SERIAL_83A.println(nmea_buf);
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
	int year = boat_sim.getYear();
	year = year > 2000 ? year-2000 : 0;
	int month = boat_sim.getMonth();
	int day = boat_sim.getDay();
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

	int hour = boat_sim.getHour();
	int minute = boat_sim.getMinute();
	int secs = boat_sim.getSecond();

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
// simulated instruments
//-----------------------------

void depthInst::send0183(bool portB)
	// SD = Sounder device
	// DPT = Water Depth
{
	double d_meters = boat_sim.getDepth() * FEET_TO_METERS;
	// 1) Depth, meters
	// 2) Offset from transducer;
	// 	  positive means distance from transducer to water line,
	// 	  negative means distance from transducer to keel
	//                       1     2
	sprintf(nmea_buf,"$SDDPT,%0.1f,0.0",d_meters);
	checksum();
	// display(show_0183,"depthInst --> %s",nmea_buf);
	sendNMEA0183(portB);
}


void logInst::send0183(bool portB)
{
	// VW = Velocity Sensor, Speed Log, Water, Mechanical
	// VHW device
	// We only send the speed through the water in knots
	// logInst does not send a heading!
	//
	// 1) Degress True
	// 2) T = True
	// 3) Degrees Magnetic
	// 4) M = Magnetic
	// 5) Knots (speed of vessel relative to the water)
	// 6) N = Knots
	// 7) Kilometers (speed of vessel relative to the water)
	// 8) K = Kilometres
	//                       12345     678
	sprintf(nmea_buf,"$VWVHW,,,,,%0.1f,N,,",
		// boat_sim.getCOG(),
		boat_sim.getWaterSpeed());
	checksum();
	// display(show_0183,"logInst --> %s",nmea_buf);
	sendNMEA0183(portB);

	if (1)	// The E80 is not seeing this message
	{
		// VLW = Distance Traveled through Water (or over ground in your case)
		// The logInst has the "trip distance" button.
		// The ST50 log probably integrates the spinny wheel over time, but
		// our simulator does better by integrating SOG over time.

		// 1) Total cumulative distance (NM)
		// 2) N = Nautical miles
		// 3) Trip distance (NM)
		// 4) N = Nautical miles
		//                       1    2 3    4
		sprintf(nmea_buf,"$VWVLW,%.2f,N,%.2f,N",
			boat_sim.getLogTotal(),
			boat_sim.getTripDistance());
		checksum();
		// display(show_0183,"logInst --> %s", nmea_buf);
		sendNMEA0183(portB);
	}
}


void windInst::send0183(bool portB)
{
	double heading = boat_sim.getHeading();
	double bow_angle_true = boat_sim.getWindAngle() - heading;
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
		boat_sim.getWindSpeed());
	checksum();
	// display(show_0183,"windInst --> %s",nmea_buf);
	sendNMEA0183(portB);

	// apparentWindAngle() is ALREADY measured relative to the bow!

	sprintf(nmea_buf,"$WIMWV,%0.1f,R,%0.1f,N,A",
		boat_sim.apparentWindAngle(),
		boat_sim.apparentWindSpeed());
	checksum();
	// display(show_0183,"windInst --> %s",nmea_buf);
	sendNMEA0183(portB);
}


void compassInst::send0183(bool portB)
{
	// Device = HC (heading compass)

	if (1)
	{
		// HDT: $MCHDT,237.5,T*hh
		//		237.5 = true heading
		//		T = true
		sprintf(nmea_buf,"$HCHDT,%0.1f,T",
			boat_sim.getHeading() );
		checksum();
		// display(show_0183,"compassInst --> %s",nmea_buf);
		sendNMEA0183(portB);
	}
	if (0)
	{
		// HDM: $HCHDM,238.0,M*hh
		// 		238.0 = magnetic heading
		// 		M = magnetic
		sprintf(nmea_buf,"$HCHDM,%0.1f,M",
			boat_sim.getHeading() );
		checksum();
		// display(show_0183,"compassInst --> %s",nmea_buf);
		sendNMEA0183(portB);
	}
}


void gpsInst::send0183(bool portB)
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
			standardLat(boat_sim.getLat()),
			standardLon(boat_sim.getLon())
		);
		checksum();  // Appends *hh
		sendNMEA0183(portB);
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
		sendNMEA0183(portB);
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
		sendNMEA0183(portB);
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
			standardLat(boat_sim.getLat()),
			standardLon(boat_sim.getLon()),
			boat_sim.getSOG(),
			boat_sim.getCOG(),
			standardDate());
		checksum();
		// display(show_0183,"gpsInst --> %s",nmea_buf);
		sendNMEA0183(portB);
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
			standardLat(boat_sim.getLat()),
			standardLon(boat_sim.getLon()),
			standardTime());
		checksum();
		// display(show_0183,"gpsInst --> %s",nmea_buf);
		sendNMEA0183(portB);
	}
}


void aisInst::send0183(bool portB)
{
}


void apInst::send0183(bool portB)
{
	char lr = 'R';
	const char *arrive_char = boat_sim.getArrived() ? "A" : "V";
	const waypoint_t *start_wp = boat_sim.getWaypoint(boat_sim.getStartWPNum());
	const waypoint_t *target_wp = boat_sim.getWaypoint(boat_sim.getTargetWPNum());
	String start_name(start_wp->name);
	String target_name(target_wp->name);

	// AP = Autopilot device
	// VTG = Velocity and Track over Ground
	//
	//	1) Course over ground (degrees True)
	//	2) T = True
	//	3) Course over ground (degrees Magnetic)
	//	4) M = Magnetic
	//	5) Speed over ground in knots
	//	6) N = Knots
	//	7) Speed over ground in km/h
	//	8) K = Kilometers/hour
	//                       1    2 34 5    6 78
	sprintf(nmea_buf,"$APVTG,%.1f,T,,M,%.2f,N,,K",
		boat_sim.getCOG(),		// 1
		boat_sim.getSOG());		// 3
	checksum();
	// display(show_0183,"apInst --> %s",nmea_buf);

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
	sprintf(nmea_buf,"$APRMC,%s,A,%s,%s,%0.1f,%0.1f,%s,,,",
		standardTime(),
		standardLat(boat_sim.getLat()),		// 3,4
		standardLon(boat_sim.getLon()),		// 5,6
		boat_sim.getSOG(),					// 7
		boat_sim.getCOG(),					// 8
		standardDate());				// 9
	checksum();
	// display(show_0183,"apInst --> %s",nmea_buf);
	sendNMEA0183(portB);


	if (boat_sim.getRouting())
	{
		// RMB = Recommended Minimum Navigation Information 'B'
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

		// NOTE - E80 VMG databox does not show
		// The ECRMB message is echoed back to us without field(12) VMG

		//                       1 2     3  4  5  67 89 10    11    12    13
		sprintf(nmea_buf,"$APRMB,A,%0.3f,%c,%s,%s,%s,%s,%0.3f,%0.1f,%0.4f,%s,",
			boat_sim.getCrossTrackError(),	// 2
			lr,							// 3
			start_name.c_str(),			// 4
			target_name.c_str(),		// 5
			standardLat(boat_sim.getLat()),	// 67
			standardLon(boat_sim.getLon()),	// 89
			boat_sim.distanceToWaypoint(),	// 10
			boat_sim.headingToWaypoint(),	// 11
			boat_sim.getSOG(),				// 12
			arrive_char);				// 13

		checksum();
		// display(show_0183,"apInst --> %s",nmea_buf);
		sendNMEA0183(portB);
		
		// BWC = Bearing and Distance to Waypoint using Great Circle
		//	1) UTC time
		//	2) Waypoint latitude
		//	3) N or S (North or South)
		//	4) Waypoint longitude
		//	5) E or W (East or West)
		//	6) Bearing to waypoint (degrees True)
		//	7) T = True
		//	8) Bearing to waypoint (degrees Magnetic)
		//	9) M = Magnetic
		// 10) Nautical miles to waypoint
		// 11) N = Nautical miles
		// 12) Waypoint ID
		// 13) Status A = Data valid, V = Data invalid
		//                       1  23 45 6    789 10   11 12 13
		sprintf(nmea_buf,"$APBWC,%s,%s,%s,%.1f,T,,,%.2f,N,%s,A",
			standardTime(),					// 1
			standardLat(target_wp->lat),	// 23
			standardLon(target_wp->lon),	// 45
			boat_sim.headingToWaypoint(),		// 6
			boat_sim.distanceToWaypoint(),		// 10
			target_wp->name);				// 12
		checksum();
		// display(show_0183,"apInst --> %s",nmea_buf);
		sendNMEA0183(portB);
	}


    if (1)
	{
		// RSA requires port and starboard angles separately.
		// Convention: positive = starboard, negative = port.
		float rudder = boat_sim.getRudder();   // -30..30 degrees
		float stbd = rudder > 0 ? rudder : 0.0f;
		float port = rudder < 0 ? -rudder : 0.0f;
		sprintf(nmea_buf, "$APRSA,%.1f,A,%.1f,A", stbd, port);
		checksum();
		sendNMEA0183(portB);
	}
}


void engInst::send0183(bool portB)
{
}


void genInst::send0183(bool portB)
{
}


//--------------------------------------------------------
// sendNMEA0183Route
//--------------------------------------------------------
// The E80 does not accept routes via Seatalk/NMEA messages
//
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
	const route_t *found = boat_sim.getRoute(route_name.c_str());
	if (!found) return;
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
			sendNMEA0183(false);
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
		sendNMEA0183(false);
	}

	display(0,"sendNMEA0183Route(%s) finshed",found->name);

}


// end of inst0183_out.cpp
