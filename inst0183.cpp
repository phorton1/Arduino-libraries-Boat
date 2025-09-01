//---------------------------------------------
// inst0183.cpp
//---------------------------------------------
// Implementation of NMEA0183 instruments.

#include "instSimulator.h"
#include "boatSimulator.h"
#include <myDebug.h>

#define MAX_NMEA_MSG	180
	// my buffer size is way bigger than official NMEA0183
	// maximum of 80 bytes
static char nmea_buf[MAX_NMEA_MSG+1];



static void checksum()
{
	int at = 1;
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
	return "100525";	// 2025-05-10
}


static const char *standardTime()
	// returns time of day as millis() since boot
{
	int secs = 12*3600 + millis()/1000;
	int hour = secs / 3600;
	int minute = (secs - hour * 3600) / 60;
	secs = secs % 60;
	if (hour > 23)
		hour = 0;
	static char time_buf[20];
	sprintf(time_buf,"%02d%02d%02d.00",hour,minute,secs);
	return time_buf;
}


static const char *standardLat(float latitude)
	// returns NMEA0183 formated "latitude,N/S" with comma
{
	char ns = 'N';
	if (latitude < 0)
	{
		latitude = abs(latitude);
		ns = 'S';
	}
	int d_lat = latitude;
	float m_lat = (latitude - d_lat) * 60.0;

	static char lat_buf[20];
	sprintf(lat_buf,"%02d%07.4f,%c",d_lat,m_lat,ns);
		// note %0X.Yf print spec is for Y decimal
		// places padded with zeros to overal length X
	return lat_buf;
}


static const char *standardLon(float longitude)
	// returns NMEA0183 formated "longitude,N/S" with comma
{
	char ew = 'W';
	if (longitude < 0)
	{
		longitude = abs(longitude);
		ew = 'W';
	}
	int d_lon = longitude;
	float m_lon = (longitude - d_lon) * 60.0;

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
	const float FEET_TO_METERS = 0.3048;
	float d_meters = boat.getDepth() * FEET_TO_METERS;
	// 1) Depth, meters
	// 2) Offset from transducer;
	// 	  positive means distance from transducer to water line,
	// 	  negative means distance from transducer to keel
	//                       1     2
	sprintf(nmea_buf,"$SDDPT,%0.1f,0.0",d_meters);
	checksum();
	display(0,"Sending %s",nmea_buf);
	SERIAL_E80_0183.println(nmea_buf);
}

void logInst::send0183()
{
}

void windInst::send0183()
{
}

void compassInst::send0183()
{
}

void gpsInst::send0183()
{
}

void autopilotInst::send0183()
{
}

void engineInst::send0183()
{
}

void gensetInst::send0183()
{
}


// end of inst0183.cpp
