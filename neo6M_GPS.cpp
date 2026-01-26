//----------------------------------------
// neo6M_GPS.cpp
//----------------------------------------
// The neo6m sends a burst of NMEA0183 messages every second.
// If full debugging is turned on, the debugging can take enough time so that
//		characters on the wire may be lost, resulting in checksum errors
//
// IMPORTANT NOTE REGARDING PRNS
//		GPS uses 1..32
//		GLONASS 1–24
//		Galileo uses 1–36
//		BeiDou uses 1–63
//		SBAS uses 120–158
//		QZSS uses 193–199
// WE ARE ONLY USING GPS AND OUR ARRAY IS INDEXED BY PRN-1


#include "instSimulator.h"
#include "inst2000.h"
#include <myDebug.h>
#include <N2kMessages.h>
#include "TimeLib.h"


#define dbg_neo			1			// 0,-1 = show parseNeo0183 info
#define dbg_raw			1			// 0 = show raw 0183 messages
#define dbg_neo_2000	1			// 0,-1 = show sent 2000 pgns


#define MAX_PRN 	32

#define SAT_IN_VIEW     		0x01   // appeared in GSV this cycle
#define SAT_USED_IN_SOLUTION    0x02   // listed in GSA (used in solution)


typedef struct
{
    int  elev;
    int  azim;
    int  snr;
    uint32_t flags;
} gps_sat_t;


typedef struct
{
    int    fix_type;
    double lat;
    double lon;
    double altitude;
    float  hdop;
	float  vdop;
    float  pdop;
    float  sog;      			// knots
    float  cog;      			// degrees
    int    sats_used;			// in solution from GSA
    gps_sat_t sats[MAX_PRN];
    int    year;
    int    month;
    int    day;
    int    hour;
    int    minute;
    float  seconds;

} gps_model_t;


// we initialize with invalid values which can
// be -1 for everything except for the lat, lon, and altitude doubles.

static gps_model_t gps_out;
static gps_model_t gps_in =
{
    .fix_type      = -1,				// must be 0..3 in NMEA0183
    .lat           = N2kDoubleNA,		// can be negative after conversion
    .lon           = N2kDoubleNA,		// can be negative after conversion
    .altitude      = N2kDoubleNA,		// can be negative
    .hdop          = -1.0f,
    .vdop          = -1.0f,
    .pdop          = -1.0f,
    .sog           = -1.0f,
    .cog           = -1.0f,
    .sats_used     = -1,
    .year          = -1,
    .month         = -1,
    .day           = -1,
    .hour          = -1,
    .minute        = -1,
    .seconds       = -1.0f
};

// static int neo_started = 0;


void initNeo6M_GPS()
	// extern'd in instSimulator.h
{
	// I was getting a lot of checksum errors and the system was crashing, and
	// I felt like there were likely bytes being lost in the uart buffers
	// due to timing issues in my program, so I added the addMemoryForRead()
	// call below to expand the buffer size which generally helped.
	//
	// With the SERIAL_9BIT_SUPPORT define undcommented in HardwareSerial.h,
	// this revealed subtle bug that the parameters to add
	// Then later, I added code to eading from a Neo6m GPS module using "regular"
	// 8 bit serial datat.  NEO_SERIAL is defined as #define NEO_SERIAL Serial5
	//
	//
	// This helped in that I stopped receiving gobs of checksum errors, but
	// then highlighted the fact that the general use of the serial port
	// with SERIAL_9BIT_SUPPORT defined, which I need for talking 9bit
	// "Seatalk" protocol within my program, has issues and definitely.
	// in combination with the addMemoryForRead() method.
	//
	// Working with coPilot AI, he identified that the addMemoryForRead()
	// method is documented to take bytes, but functionally (in 9 bit mode)
	//
	//		expects “number of BUFTYPE elements”, but every caller (including
	//		you, and every example PJRC ever published) passes bytes.
	//
	// When I changed the addMemoryForRead() call to take the number of
	// elements, rather than the number of bytes the bug went away and
	// my program seems to work, and is undergoing further long-term testing.
	
	#if 1
		// Allocate a larger RX buffer for NEO_SERIAL
		#define NUMBER_BUF_ELEMENTS 10240
		static uint16_t serial1_rxbuf[NUMBER_BUF_ELEMENTS];
		// And the use of uint8_t for the buffer is not correct
		// within the context of the SERIAL_9BIT_SUPPORT:
		// 		static uint8_t serial1_rxbuf[NUMBER_BUF_ELEMENTS];
		NEO_SERIAL.addMemoryForRead(serial1_rxbuf, NUMBER_BUF_ELEMENTS);
		// This causes hard crashes:
		// 		NEO_SERIAL.addMemoryForRead(serial1_rxbuf, sizeof(serial1_rxbuf));
	#endif

	NEO_SERIAL.begin(9600);
	display(0,"GPS_SERIAL5 started",0);
	delay(500);
}





#if 0
	// unused kept for posterities sake

	void setNEOBaud(bool save, int baud)
		// untried method from coPilot to change the module's
		// baud rate and optionally save it persistently
	{
		const uint8_t set38400[] = {
			0xB5,0x62,0x06,0x00,0x14,0x00,
			0x01,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,
			0x00,0x96,0x00,0x00,   // 38400 LE
			0x07,0x00,0x03,0x00,
			0x00,0x00,
			0xA2,0xB5
		};

		const uint8_t set9600[] = {
			0xB5,0x62,0x06,0x00,0x14,0x00,
			0x01,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,
			0x80,0x25,0x00,0x00,   // 9600 LE
			0x07,0x00,0x03,0x00,
			0x00,0x00,
			0xC0,0x7E
		};

		const uint8_t saveCfg[] = {
			0xB5,0x62,0x06,0x09,0x0D,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,0x00,
			0x17,0x31
		};

		const uint8_t *pkt = nullptr;
		size_t len = 0;

		if (baud == 38400) {
			pkt = set38400;
			len = sizeof(set38400);
		} else if (baud == 9600) {
			pkt = set9600;
			len = sizeof(set9600);
		} else {
			return;   // unsupported baud
		}

		for (size_t i = 0; i < len; i++)
			GPS_SERIAL5.write(pkt[i]);

		if (save) {
			for (size_t i = 0; i < sizeof(saveCfg); i++)
				GPS_SERIAL5.write(saveCfg[i]);
		}
	}
#endif	//0


//-----------------------------------------------------------
// NMEA0183 Parser
//-----------------------------------------------------------

static int hexval(char c)
{
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	return -1;
}



static bool checkOK(const char *msg)
	// Return true if checksum matches, false otherwise
{
    // get transmitted checksum (two hex chars)

    const char *star = strchr(msg, '*');
    if (!star || !star[1] || !star[2])
        return false;
    int hi = hexval(star[1]);
    int lo = hexval(star[2]);
    if (hi < 0 || lo < 0)
        return false;
    uint8_t chk = (hi << 4) | lo;

    // Compute XOR of all chars between '$' and '*'

    uint8_t sum = 0;
    const char *p = msg;
    if (*p == '$')
        p++;
    while (p < star)
    {
        sum ^= (uint8_t)(*p);
        p++;
    }

	if (sum != chk)
	{
		my_error("neo6M_GPS CSERR exp=0x%02x calc=0x%02x len=%d",
			chk, sum, (int)strlen(msg));
		display_bytes(0,"ERR",(const uint8_t *)msg,strlen(msg));
		warning(0,"CSERR MSG=%s",msg);
		return false;
	}
    return true;
}


static int tokenize0183(const char *msg, String *out, int max)
{
    int count = 0;
    const char *p = msg;
    while (*p && count < max)
    {
        const char *start = p;
        while (*p && *p != ',' && *p != '*')
            p++;

        out[count++] = String(start).substring(0, p - start);

        if (*p == ',')
            p++;
        else if (*p == '*')
            break;
    }
    return count;
}


static double getLat0183(const String &val, const String &ns)
{
    if (!val.length() || !ns.length()) return 0.0;
    double raw = val.toFloat();          // ddmm.mmmm
    int deg = (int)(raw / 100);
    double minutes = raw - deg * 100;
    double lat = deg + minutes / 60.0;
    if (ns == "S") lat = -lat;
    return lat;
}

static double getLon0183(const String &val, const String &ew)
{
    if (!val.length() || !ew.length()) return 0.0;
    double raw = val.toFloat();          // dddmm.mmmm
    int deg = (int)(raw / 100);
    double minutes = raw - deg * 100;
    double lon = deg + minutes / 60.0;
    if (ew == "W") lon = -lon;
    return lon;
}



#if 0
	void find_bug(const char *s)
	{
		if (debug_level)
		{
			my_error("BUG %s",s);
			while (1) { delay(1000); }
		}
	}
#endif



static void parseNeo0183(const char *msg)
{
	display(dbg_raw, "parseNeo0183: %s", msg);
	if (!checkOK(msg))
		return;

	static int last_fix     = -1;
	static int last_view    = -1;
	static int sats_in_view = 0;
	static int num_msgs     = 0;

    if (dbg_raw>0 && num_msgs++ % 100 == 0)
	{
        display(0,"parseNeo183(%d) secs(%d) fix_type=%d sats_in_view=%d sats_used=%d",
			num_msgs, millis()/1000,gps_in.fix_type,
			sats_in_view, gps_in.sats_used);
		Serial.print("    View Sats: ");
		for (int i=0; i<MAX_PRN; i++)
		{
			if (gps_out.sats[i].flags & SAT_IN_VIEW)
			{
				Serial.print(i+1);
				Serial.print(" ");
			}
		}
		Serial.println();
		Serial.print("    Used Sats: ");
		for (int i=0; i<MAX_PRN; i++)
		{
			if (gps_out.sats[i].flags & SAT_USED_IN_SOLUTION)
			{
				Serial.print(i+1);
				Serial.print(" ");
			}
		}
		Serial.println();
	}


    String tok[20];
    int num_toks = tokenize0183(msg, tok, 20);
    if (num_toks < 0)
	{
		if (num_toks == 0)
			warning(dbg_neo,"neo6M_GPS empty message",0);
		return;
	}


	//------------------
	// messages
	//------------------

	proc_entry();

    if (tok[0].endsWith("GGA"))
    {
        // fix type gotten preferentially from GSA
		if (tok[7].length()) gps_in.sats_used = tok[7].toInt();
        if (tok[8].length()) gps_in.hdop = tok[8].toFloat();
        if (tok[9].length()) gps_in.altitude = tok[9].toFloat();

        display(dbg_neo, "GGA fix=%d used=%d hdop=%.1f alt=%.1f",
			gps_in.fix_type, gps_in.sats_used,
			gps_in.hdop, gps_in.altitude);
    }
    else if (tok[0].endsWith("GSA"))
    {
		if (num_toks != 19)
		{
			my_error("MALFORMED GSA Sentence: %s",msg);
			proc_leave();
			return;
		}

		// the neo apparently sends out an empty GSA for GLONAS, and in addition
		// it sends a 2nd GSA for some other constellation with a satellite
		// that is not even "in view" sheesh.
		//
		// Since I moved the clearing of the used bits to the GSB end marker
		// (our cycle transition), we can accept an empty record here now,
		// and just note that "in_view" might not contain the used sats

		bool valid_gps = false;
		for (int i=3; i<=14 && i<num_toks; i++)
		{
			if (tok[i].length())
			{
				valid_gps = true;
				break;
			}
		}
		if (!valid_gps)
		{
			proc_leave();
			return;
		}

		// pdop is required by NMEA0183 spec but we treat it as optional

        gps_in.fix_type = tok[2].toInt();
		if (tok[15].length()) gps_in.pdop = tok[15].toFloat();
        if (tok[16].length()) gps_in.hdop = tok[16].toFloat();
        if (tok[17].length()) gps_in.vdop = tok[17].toFloat();

		// mark used PRNs

		int b_ptr = 0;
		char used_buf[120];		// debugging only
		for (int i=3; i<=14 && i<num_toks; i++)
		{
			if (!tok[i].length()) continue;
			int prn = tok[i].toInt();
			if (prn<=0 || prn>MAX_PRN)		// and it defintitely should not be 0 or negative
			{
				my_error("INVALID GSV PRN(%d)!!!!!",prn);
				continue;
			}
			gps_in.sats[prn-1].flags |= SAT_USED_IN_SOLUTION;
			sprintf(&used_buf[b_ptr*3]," %02d",prn);
			b_ptr++;
		}
		used_buf[b_ptr*3] = 0;

        display(dbg_neo, "GSA fix=%d pdop=%s hdop=%s vdop=%s used: %s",
            gps_in.fix_type,
			tok[15].c_str(),
			tok[16].c_str(),
			tok[17].c_str(),
			used_buf);
		
		// debuging only if fix type changes

        if (gps_in.fix_type != last_fix)
        {
            warning(0, "fix_type %d->%d at %lu",
                last_fix, gps_in.fix_type, millis()/1000);
            last_fix = gps_in.fix_type;
        }
    }
    else if (tok[0] == "$GPGSV")		//  tok[0].endsWith("GSV"))
    {
		// Note that we are explicitly just "skipping" $BDGSV,
		// the neo's view of Beidou satellites with the idea that
		// (a) it prioritizes use of the GPS constellation
		// (b) the E80 only expects a GPS constellation
		// (c) if there are no bars or icons on the e80 Sat status dialog
		//     it is not really important, the only thing that really
		//	   matters is the lat/lon
		
        int total_msgs = tok[1].length() ? tok[1].toInt() : -1;		// used from this message to copy to gps_out
        int msg_num    = tok[2].length() ? tok[2].toInt() : -1;		// used from this message to copy to gps_out
        sats_in_view   = tok[3].length() ? tok[3].toInt() : -1;		// static only for debugging

        display(dbg_neo, "GSV %d/%d ideal sats_in_view=%d",
            msg_num, total_msgs, sats_in_view);

		// on the 1st msg we clear all the IN_VIEW bits
		// as the "least incorrect" way to deal with NMEA0183

		if (msg_num == 1)
		{
			for (int i=0; i<MAX_PRN; i++)
				gps_in.sats[i].flags &= ~SAT_IN_VIEW;
		}

		// Then we loop through the PRNS presented in this GSV message and
		// set their SAT_IN_VIEW bits and elev/azim/snr if provided with
		// notion that we are maintaining old elev/azim/snr's as the
		// last known values for satellites that come in and go out ov view

        proc_entry();
        for (int i = 0; i < 4; i++)
        {
            int idx = 4 + i * 4;			// the index of the ith' satellites TOKEN in the message
            if (idx + 3 >= num_toks)		// which might not exists for the last (shortened) message
                break;						// in which case we terminate the loop

			int prn = tok[idx].toInt();		// get the prn, where ,, might, but shouldn't == 0
			if (prn<=0 || prn>MAX_PRN)		// and it defintitely should not be 0 or negative
			{
				// coPilot says PRNSs > 32 are likely GLONASS and
				// can be ignored.  GLONASS can be disable by UBX protocol,
				// but the larger issues is that UBX protocol is probably
				// "better" than NMEA0183 for using the neo6m.

				warning(dbg_neo+1,"INVALID GSV PRN(%d)!!!!!",prn);
				continue;
			}

			gps_in.sats[prn-1].flags |= SAT_IN_VIEW;

			// assign any elev,asim, or snr that is not ,,

			if (tok[idx+1].length())
				gps_in.sats[prn-1].elev = tok[idx+1].toInt();
			if (tok[idx+2].length())
				gps_in.sats[prn-1].azim = tok[idx+2].toInt();
			if (tok[idx+3].length())
				gps_in.sats[prn-1].snr  = tok[idx+3].toInt();

            display(dbg_neo+1, "sat[%02d] prn(%d) elev=%s az=%s snr=%s",
				i,
				prn,
				tok[idx+1].c_str(),
				tok[idx+2].c_str(),
				tok[idx+3].c_str());
        }

        proc_leave();

        if (sats_in_view > last_view)
        {
			// debug only message to help me visualize acquisition
            warning(0, "sats_in_view %d->%d at %lu",
                last_view, sats_in_view, millis()/1000);
            last_view = sats_in_view;
        }

		// COMPLETE THE CYCLE by copying gps_in to gps_out and
		// clearing the USED bits for the next one

		if (msg_num == total_msgs)
		{
			memcpy(&gps_out,&gps_in,sizeof(gps_model_t));
			// clear used flags
			for (int i=0; i<MAX_PRN; i++)
				gps_in.sats[i].flags &= ~SAT_USED_IN_SOLUTION;

		}
		
    }

	else if (tok[0].endsWith("RMC"))
	{
		// time: hhmmss.ss
		if (tok[1].length() >= 6)
		{
			int hh = tok[1].substring(0, 2).toInt();
			int mm = tok[1].substring(2, 4).toInt();
			int ss = tok[1].substring(4, 6).toInt();
			gps_in.hour   = hh;
			gps_in.minute = mm;
			gps_in.seconds = ss;
		}

		// date: ddmmyy
		if (tok[9].length() == 6)
		{
			int dd = tok[9].substring(0, 2).toInt();
			int mo = tok[9].substring(2, 4).toInt();
			int yy = tok[9].substring(4, 6).toInt() + 2000;
			gps_in.day   = dd;
			gps_in.month = mo;
			gps_in.year  = yy;
		}


		if (1)
		{
			// set the teensy's clock
			setTime(gps_in.hour,
					gps_in.minute,
					gps_in.seconds,
					gps_in.day,
					gps_in.month,
					gps_in.year);
		}

		if (tok[3].length() && tok[4].length())
			gps_in.lat = getLat0183(tok[3], tok[4]);
		if (tok[5].length() && tok[6].length())
			gps_in.lon = getLon0183(tok[5], tok[6]);

		if (tok[7].length()) gps_in.sog = tok[7].toFloat();
		if (tok[8].length()) gps_in.cog = tok[8].toFloat();

		display(dbg_neo,
			"RMC lat=%.5f lon=%.5f sog=%.1f cog=%.1f %04d-%02d-%02d %02d:%02d:%02d",
			gps_in.lat, gps_in.lon, gps_in.sog, gps_in.cog,
			gps_in.year, gps_in.month, gps_in.day,
			gps_in.hour, gps_in.minute, gps_in.seconds);
	}

	proc_leave();

}	// parseNeo0183()




//----------------------------------------------------
// sendNEO2000
//----------------------------------------------------
// Sends the same messages in the same order as inst2000_out.cpp
// gpsInst::send2000() which is known to work with the E80
// GPS status window to show bars and icons.
//
// MY invalid values are -1.
// Here I provide methods to map my invalid values to the
// NMEAInvalid values as needed.

static double mapDoubleNA(double v)
{
    return (v < 0.0) ? N2kDoubleNA : v;
}

static uint8_t mapUInt8NA(int v)
{
    return (v < 0) ? N2kUInt8NA : (uint8_t)v;
}

static double mapDegToRadNA(double v)
	// NOT USED BELOW FOR lat/lon/altituged which were
	// initialized to NA
{
    return (v < 0) ? N2kDoubleNA : DegToRad(v);
}


void sendNeo2000()
	// We use gps_out.year>0 both as a flag that the year is valid for
	// PGN 126992, but perhaps more importantly, as a flag that the
	// entire gps_out data structure is valid for PGN 129540
{
    tN2kMsg msg;

    // PGN 129029 - GNSS Position Data

	if (gps_out.fix_type >= 1)
	{
		// teensy TimeLib.h has already been updated in the RMC parser:
		// ^^^ this may be a bad assumption since fix_type is not from RMC,
		// but at least they're valid values (not NA) in all cases below

		time_t t = now();
		uint32_t days = t / 86400;
		uint32_t secs = t % 86400;

		display(dbg_neo_2000,
			"129029 GNSS lat=%.6f lon=%.6f alt=%.1f fix=%d sats=%d hdop=%.1f vdop=%.1f pdop=%.1f",
			gps_out.lat,
			gps_out.lon,
			gps_out.altitude,
			gps_out.fix_type,
			gps_out.sats_used,
			gps_out.hdop,
			gps_out.vdop,
			gps_out.pdop
		);

		SetN2kPGN129029(msg,
			255,							// SID (sequence ID)
			days,							// Days since 1970-01-01	gotten from teensy clock, whatever that is
			secs,							// Seconds since midnight	gotten from teensy clock, whatever that is
			gps_out.lat,					// Latitude (deg)			already mapped to NA if un-inited
			gps_out.lon,					// Longitude (deg)			already mapped to NA if un-inited
			gps_out.altitude,				// Altitude (m)				already mapped to NA if un-inited
			N2kGNSSt_GPS,					// GNSS type (GPS)
			N2kGNSSm_GNSSfix,				// GNSS method (GNSS fix)
			gps_out.sats_used,				// Number of satellites used
			mapDoubleNA(gps_out.hdop),		// HDOP						turned into NA if < 0
			mapDoubleNA(gps_out.pdop),		// PDOP						turned into NA if < 0
			0,								// Geoidal separation (m)
			0,								// Position accuracy estimate (m)
			N2kGNSSt_GPS,					// Integrity type
			0,								// Reserved
			0 );							// Reserved
		nmea2000.SendMsg(msg);
	}


	// inst2000 instGPS sends PGN_DIRECTION_DATA 130577 here
	// but we dont.  It still works in the E80 GPS Status window

	if (gps_out.year > 0)		// valid DT from GPS
	{
		// PGN 126992 - System Time

		time_t now = time(NULL);
		uint32_t days = now / 86400;
		uint32_t secs = now - days * 86400;

		display(dbg_neo_2000,
			"126992 Time %04d-%02d-%02d %02d:%02d:%.1f",
			gps_out.year,
			gps_out.month,
			gps_out.day,
			gps_out.hour,
			gps_out.minute,
			gps_out.seconds
		);

		SetN2kPGN126992(msg,
			255,
			days,
			secs,
			N2ktimes_GPS
		);
		nmea2000.SendMsg(msg);
	}


    if (gps_out.year > 0)	// Valid (if partly unitialized) gps_out record from a cycle
    {
	    // PGN 129540 - Satellites in View
		// we count the sats in view based on the SAT_IN_VIEW bit, and
		// only emit this message if the number is nonzero

		int sats_in_view = 0;
		for (int i=0; i<MAX_PRN; i++)
		{
			if (gps_out.sats[i].flags & SAT_IN_VIEW)
				sats_in_view++;
		}

        if (sats_in_view > 0)
        {
            display(dbg_neo_2000,"129540 SatsInView in_view(%d)",sats_in_view);
			proc_entry();

			int sid = 7;  // or any fixed non-255 SID
            SetN2kPGN129540(msg,sid,N2kDD072_Unavailable);
				// N2kDD072_Unavailable == the GNSS receiver did not provide any integrity/differential status information

            for (int prn_m1=0; prn_m1<MAX_PRN; prn_m1++)
            {
				if (!(gps_out.sats[prn_m1].flags & SAT_IN_VIEW))
					continue;  // not in most recent GSV
				bool used = gps_out.sats[prn_m1].flags & SAT_USED_IN_SOLUTION ? 1 : 0;

				display(dbg_neo_2000 + 1,
					"PGN SAT PRN[%d] elev(%d) azim(%d) snr(%d) used(%d)",
					prn_m1 + 1,
					gps_out.sats[prn_m1].elev,
					gps_out.sats[prn_m1].azim,
					gps_out.sats[prn_m1].snr,
					used);

                tSatelliteInfo sat;
				if (0)
				{
					sat.PRN = prn_m1 + 1;
					sat.Elevation = DegToRad(gps_out.sats[prn_m1].elev);
					sat.Azimuth = DegToRad(gps_out.sats[prn_m1].azim);
					sat.SNR = gps_out.sats[prn_m1].snr;
					sat.RangeResiduals = N2kDoubleNA;
				}
				else
				{
					sat.PRN        = prn_m1 + 1;
					sat.Elevation  = mapDegToRadNA(gps_out.sats[prn_m1].elev);
					sat.Azimuth    = mapDegToRadNA(gps_out.sats[prn_m1].azim);
					sat.SNR        = mapUInt8NA(gps_out.sats[prn_m1].snr);
					sat.RangeResiduals = N2kDoubleNA;
				}
				sat.UsageStatus =  used ?
					N2kDD124_UsedInSolutionWithoutDifferentialCorrections :
					N2kDD124_TrackedButNotUsedInSolution;
                AppendN2kPGN129540(msg, sat);
            }

            nmea2000.SendMsg(msg);
			proc_leave();
        }
    }
}



//---------------------------------------------------
// doNeo6M_GPS
//---------------------------------------------------

void doNeo6M_GPS()
	// extern'd in instSimulator.h
{
	while (NEO_SERIAL.available())
	{
		#define MAX_0183_MSG 180
		int c = NEO_SERIAL.read();
		static char buf[MAX_0183_MSG+1];
		static volatile int buf_ptr = 0;

		if (buf_ptr >= MAX_0183_MSG)
		{
			buf[buf_ptr] = 0;
			my_error("NEO_TOO_LONG: `%s'",buf);
			buf_ptr = 0;
		}
		else if (c == '\n')
		{
			buf[buf_ptr] = 0;
			parseNeo0183(buf);
			buf_ptr = 0;
			return;
		}
		else if (c != '\r')
		{
			if (buf_ptr && c == '$')
			{
				buf[buf_ptr] = 0;
				my_error("NEO 2nd dollar after: `%s'",buf);
				buf[0] = c;
				buf_ptr = 1;
			}
			buf[buf_ptr++] = c;
		}
	}

	#if 1
		#define NEO_SEND_INTERVAL	1000	// send gps messages every second
		uint32_t now = millis();
		static uint32_t last_neo_send = 0;
		if (now - last_neo_send > NEO_SEND_INTERVAL)
		{
			last_neo_send = now;
			sendNeo2000();
		}
	#endif
}




// end of neo6M_GPS.cpp
