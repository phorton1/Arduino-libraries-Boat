//----------------------------------------
// neo6M_GPS.cpp
//----------------------------------------
// The neo6m sends a burst of NMEA0183 messages every second.
// We "frame" the burst as a "cycle" based on an idle time of at least 50ms
//
// IMPORTANT NOTE REGARDING PRNS
//		GPS uses 1..32
//		GLONASS 1–24
//		Galileo uses 1–36
//		BeiDou uses 1–63
//		SBAS uses 120–158
//		QZSS uses 193–199
//
// WE ARE ONLY USING GPS AND OUR ARRAY IS INDEXED BY PRN-1

#include <myDebug.h>
#include "TimeLib.h"


// Seatalk
#define E80_PORT2	1
#include "instST.h"
#include "boatSimulator.h"

volatile bool st_neo_device_query_pending;
	// set by client code (instST_in.cpp or teensyGPS.ino) when a device query is
	// received, in which case, the neo will reply with a device id message
	


// NMEA200
#include "inst2000.h"
#include <N2kMessages.h>


#define dbg_neo			0			// lifecycle

#define dbg_raw			1			// 0 = show raw 0183 messages
#define dbg_0183		1			// 0,-1 = show parseNeo0183 info
#define dbg_neo_ST		1
#define dbg_neo_2000	1			// 0,-1 = show sent 2000 pgns

#define DBG_STATUS		1			// show msg every 100 parses + status advances


static HardwareSerial *NEO_SERIAL;
static bool st_enabled = 1;
static bool nmea2000_enabled = 0;
static uint8_t version;
static uint8_t subversion;


#define FRAME_IDLE_TIME	50			// 50 ms idle defines a frame
	// we skip first, possibly partial, frame after initialization

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
    int    	fix_type;			// NMEA0183 Fix type
    double 	lat;
    double 	lon;
    double 	altitude;
    float  	hdop;
	float  	vdop;
    float  	pdop;
    float 	sog;      			// knots
    float 	cog;      			// degrees
	int    	num_viewed;			// as per GSV
    int    	num_used;			// in solution as per GSA
    gps_sat_t sats[MAX_PRN];
    int    	year;
    int    	month;
    int    	day;
    int    	hour;
    int    	minute;
    int  	seconds;

} gps_model_t;


// we initialize with invalid values which can
// be -1 for everything except for the lat, lon, and altitude doubles.

static gps_model_t gps_model;
static uint32_t last_receive_time = 0;
static bool neo_started = 0;
	// skip one frame to start

// statics for progress indicators


static int got_gsa = 0;
#if DBG_STATUS
	static int last_fix     = -1;
	static int last_view    = -1;
#endif


static void initModel()
{
	memset(&gps_model,0,sizeof(gps_model));
    gps_model.fix_type      = -1;				// must be 0..3 in NMEA0183
	gps_model.lat           = 0.00;				// ST just sends zeros until fix
	gps_model.lon           = 0.00;
	gps_model.altitude      = 0.00;
	gps_model.hdop          = -1;
	gps_model.vdop          = -1;
	gps_model.pdop          = -1;
	gps_model.sog           = -1;
	gps_model.cog           = -1;
	gps_model.num_viewed 	= -1;
    gps_model.num_used      = -1;
    gps_model.year          = -1;
    gps_model.month         = -1;
    gps_model.day           = -1;
    gps_model.hour          = -1;
    gps_model.minute        = -1;
	gps_model.seconds       = -1;

	got_gsa = 0;

	#if DBG_STATUS
		last_fix     = -1;
		last_view    = -1;
	#endif
};


static void initCycle()
{
	got_gsa = 0;
	for (int i=0; i<MAX_PRN; i++)
	{
		gps_model.sats[i].flags = 0;
	}
}


//---------------------------------------
// API
//---------------------------------------


bool NeoSeatalkEnabled()	{ return st_enabled; }
bool NeoNMEA2000Enabled()   { return nmea2000_enabled; }

extern void enableNeoSeatalk(bool enable)
{
	display(dbg_neo,"SEATALK ENABLED=%d",enable);
	st_enabled = enable;
}

extern void enableNeoNMEA200(bool enable)
{
	display(dbg_neo,"NMEA200 ENABLED=%d",enable);
	nmea2000_enabled = enable;
}


//-----------------------------------------------------------
// initialization
//-----------------------------------------------------------

static bool genuineNeoModule()
	// Ironically, we dont need to configure a genuine module
	// to not send GLONASS and Bedieu messages, and we CANNOT
	// configure a clone to not send them.
{
    display(0,"Requesting UBX-MON-VER...",0);
	const uint8_t ubx_mon_ver_request[] = {
		0xB5,0x62,        // UBX header
		0x0A,0x04,        // MON-VER
		0x00,0x00,        // length = 0
		0x0E,0x34         // checksum
	};

		NEO_SERIAL->write(ubx_mon_ver_request, sizeof(ubx_mon_ver_request));

    uint32_t start = millis();
    int state = 0;

	#define MAX_VER_BUF  255
	static char buf[MAX_VER_BUF+1];
	uint16_t len = 0;
    uint16_t count = 0;

    while (1)
	{
		if (millis() - start > 300)
		{
			my_error("genuineNeoModule() UBX-MON_VER timeout",0);
			return false;
		}

        if (!NEO_SERIAL->available()) {
            yield();
            continue;
        }

        uint8_t b = NEO_SERIAL->read();

        switch (state) {

            case 0: // hunt for 0xB5
                if (b == 0xB5) state = 1;
                break;

            case 1: // hunt for 0x62
                if (b == 0x62) state = 2;
                else state = 0;
                break;

            case 2: // class
                if (b != 0x0A)
				{
					my_error("genuineNeoModule() expected UBX class(0x0a) got 0x%02x",b);
					return false;
				}
                state = 3;
                break;

            case 3: // id
                if (b != 0x04)
				{
					my_error("genuineNeoModule() expected UBX id(0x04) got 0x%02x",b);
					return false;
				}
                state = 4;
                break;

            case 4: // length LSB
                len = b;
                state = 5;
                break;

            case 5: // length MSB
                len |= (uint16_t)b << 8;
				if (len > MAX_VER_BUF)
				{
					my_error("genuineNeoModule() expected len<%d, got %d",MAX_VER_BUF,len);
					return false;
				}
				state = 6;
				break;

            case 6: // read MON-VER payload
                if (dbg_neo<=0)
					Serial.write(b);
				buf[count++] = b;

                if (count == len)
				{
					buf[len] = 0;
					if (dbg_neo<=0)
						Serial.println();

					// return TRUE if genuine neo module
					// apparently the chinese were at least kind enough to not
					// fake the genuine signature that looks something like
					//
					//		SW VERSION: 7.03 (45969)
					//		HW VERSION: 00040007
					//		EXT CORE 2.00 ...

					if (strstr(buf, "SW VERSION:"))
						return true;   // genuine u-blox
					else
						return false;  // clone
				}
				break;
        }
    }
}



void initNeo6M_GPS(HardwareSerial *neo_serial, uint8_t v, uint8_t sv)
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
	

	display(dbg_neo,"initNeo6M_GPS(%02x.%02x) called",v,sv);
	proc_entry();

	NEO_SERIAL = neo_serial;
	version = v;
	subversion = sv;

	#if 1
		// Allocate a larger RX buffer for NEO_SERIAL
		display(dbg_neo,"increasing NEO_SERIAL buffer size",0);

		#define NUMBER_BUF_ELEMENTS 10240
		static uint16_t serial1_rxbuf[NUMBER_BUF_ELEMENTS];
		// And the use of uint8_t for the buffer is not correct
		// within the context of the SERIAL_9BIT_SUPPORT:
		// 		static uint8_t serial1_rxbuf[NUMBER_BUF_ELEMENTS];
		NEO_SERIAL->addMemoryForRead(serial1_rxbuf, NUMBER_BUF_ELEMENTS);
		// This causes hard crashes:
		// 		NEO_SERIAL->addMemoryForRead(serial1_rxbuf, sizeof(serial1_rxbuf));
	#endif


	neo_started = 0;
	last_receive_time = 0;
	initModel();

	NEO_SERIAL->begin(9600);
	display(dbg_neo,"NEO_SERIAL started",0);
	delay(300);

	if (genuineNeoModule())
		warning(dbg_neo,"GENUINE uBLOX NEO6M MODULE FOUND",0);
	else
		warning(dbg_neo,"CLONE NEO6M MODULE!!",0);

	proc_leave();
	display(dbg_neo,"initNeo6M_GPS() finished",0);
}




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



static void parseNeo0183(const char *msg)
{
	display(dbg_raw, "parseNeo0183: %s", msg);
	if (!checkOK(msg))
		return;

    String tok[20];
    int num_toks = tokenize0183(msg, tok, 20);
    if (num_toks < 0)
	{
		if (num_toks == 0)
			warning(dbg_0183,"neo6M_GPS empty message",0);
		return;
	}


	//------------------
	// messages
	//------------------

	proc_entry();

    if (tok[0].endsWith("GGA"))
    {
        // fix type gotten preferentially from GSA
		if (tok[7].length()) gps_model.num_used = tok[7].toInt();
        if (tok[8].length()) gps_model.hdop = tok[8].toFloat();
        if (tok[9].length()) gps_model.altitude = tok[9].toFloat();

        display(dbg_0183, "GGA fix=%d used=%d hdop=%.1f alt=%.1f",
			gps_model.fix_type, gps_model.num_used,
			gps_model.hdop, gps_model.altitude);
    }
    else if (tok[0].endsWith("GSA"))
    {
		if (num_toks != 19)
		{
			my_error("MALFORMED GSA Sentence: %s",msg);
			proc_leave();
			return;
		}

		// the clone neo apparently sends out an empty GSA for GLONAS, and in
		// addition it may send another GSA for some other constellation with
		// a satellite. We accept only the first GSA per cycle.

		if (got_gsa)
		{
			// Serial.println("    skipGSA");
			proc_leave();
			return;
		}
		got_gsa++;

		// pdop is required by NMEA0183 spec but we treat it as optional

        gps_model.fix_type = tok[2].toInt();
		if (tok[15].length()) gps_model.pdop = tok[15].toFloat();
        if (tok[16].length()) gps_model.hdop = tok[16].toFloat();
        if (tok[17].length()) gps_model.vdop = tok[17].toFloat();

		// mark used PRNs

		int b_ptr = 0;
		char used_buf[120];		// debugging only
		for (int i=3; i<=14 && i<num_toks; i++)
		{
			if (!tok[i].length()) continue;
			int prn = tok[i].toInt();
			if (prn<=0 || prn>MAX_PRN)		// and it defintitely should not be 0 or negative
			{
				my_error("INVALID GSA PRN(%d)!!!!!",prn);
				continue;
			}
			gps_model.sats[prn-1].flags |= SAT_USED_IN_SOLUTION;
			sprintf(&used_buf[b_ptr*3]," %02d",prn);
			b_ptr++;
		}
		used_buf[b_ptr*3] = 0;

        display(dbg_0183, "GSA fix=%d pdop=%s hdop=%s vdop=%s used: %s",
            gps_model.fix_type,
			tok[15].c_str(),
			tok[16].c_str(),
			tok[17].c_str(),
			used_buf);
		

		#if DBG_STATUS
			// show fix type changes
			if (gps_model.fix_type != last_fix)
			{
				warning(0, "fix_type %d->%d at %lu",
					last_fix, gps_model.fix_type, millis()/1000);
				last_fix = gps_model.fix_type;
			}
		#endif
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
		
        int total_msgs = tok[1].length() ? tok[1].toInt() : -1;		// used from this message to copy to gps_model
        int msg_num    = tok[2].length() ? tok[2].toInt() : -1;		// used from this message to copy to gps_model
        if (tok[3].length())										// used only for debugging
			gps_model.num_viewed = tok[3].toInt();

        display(dbg_0183, "GSV %d/%d ideal num_viewed=%d",
            msg_num, total_msgs, gps_model.num_viewed);

		// Loop through the PRNS presented in this GSV message and
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

				warning(dbg_0183+1,"INVALID GSV PRN(%d)!!!!!",prn);
				continue;
			}

			gps_model.sats[prn-1].flags |= SAT_IN_VIEW;

			// assign any elev,asim, or snr that is not ,,

			if (tok[idx+1].length())
				gps_model.sats[prn-1].elev = tok[idx+1].toInt();
			if (tok[idx+2].length())
				gps_model.sats[prn-1].azim = tok[idx+2].toInt();
			if (tok[idx+3].length())
				gps_model.sats[prn-1].snr  = tok[idx+3].toInt();

            display(dbg_0183+1, "sat[%02d] prn(%d) elev=%s az=%s snr=%s",
				i,
				prn,
				tok[idx+1].c_str(),
				tok[idx+2].c_str(),
				tok[idx+3].c_str());
        }

        proc_leave();

		#if DBG_STATUS
			// show increasing number of sats in view
			if (gps_model.num_viewed > last_view)
			{
				warning(0, "num_viewed %d->%d at %lu",
					last_view, gps_model.num_viewed, millis()/1000);
				last_view = gps_model.num_viewed;
			}
		#endif
    }

	else if (tok[0].endsWith("RMC"))
	{
		// time: hhmmss.ss
		int got_dt = 0;

		if (tok[1].length() >= 6)
		{
			int hh = tok[1].substring(0, 2).toInt();
			int mm = tok[1].substring(2, 4).toInt();
			int ss = tok[1].substring(4, 6).toInt();
			#if DBG_STATUS
				if (gps_model.year == -1)
				{
					warning(0,"got time(%02d:%02d:%02d)",hh,mm,ss);
				}
			#endif
			gps_model.hour   = hh;
			gps_model.minute = mm;
			gps_model.seconds = ss;
			got_dt++;
		}

		// date: ddmmyy
		if (tok[9].length() == 6)
		{
			int dd = tok[9].substring(0, 2).toInt();
			int mo = tok[9].substring(2, 4).toInt();
			int yy = tok[9].substring(4, 6).toInt() + 2000;
			#if DBG_STATUS
				if (gps_model.year == -1)
				{
					warning(0,"got date(%d-%02d-%02d)",yy,mo,dd);
				}
			#endif
			gps_model.day   = dd;
			gps_model.month = mo;
			gps_model.year  = yy;
			got_dt++;

		}

		if (got_dt == 2)
		{
			// set the teensy's clock if got both date and time
			
			setTime(gps_model.hour,
					gps_model.minute,
					gps_model.seconds,
					gps_model.day,
					gps_model.month,
					gps_model.year);
		}

		if (tok[3].length() && tok[4].length())
			gps_model.lat = getLat0183(tok[3], tok[4]);
		if (tok[5].length() && tok[6].length())
			gps_model.lon = getLon0183(tok[5], tok[6]);

		if (tok[7].length()) gps_model.sog = tok[7].toFloat();
		if (tok[8].length()) gps_model.cog = tok[8].toFloat();

		display(dbg_0183,
			"RMC lat=%.5f lon=%.5f sog=%.1f cog=%.1f %04d-%02d-%02d %02d:%02d:%02d",
			gps_model.lat, gps_model.lon, gps_model.sog, gps_model.cog,
			gps_model.year, gps_model.month, gps_model.day,
			gps_model.hour, gps_model.minute, gps_model.seconds);
	}

	proc_leave();

}	// parseNeo0183()




//===================================================
//===================================================
// Seatalk Sender and replyToRestartGPSButton()
//===================================================
//===================================================


void replyToRestartGPSButton()
	// Called from instST_in.cpp while parsing "in" SAT_DETAIL
	// and DIF_DETAIL messages that are not otherwise expected.
{
	if (!st_enabled) return;
	warning(0,"replyToRestartGPSButton()",0);

	// This very specific signature and response allows the "Restart GPS" button
	// to "work" as a signal to this code which normally sends:
	//
	//		ST2_SAT_DETAIL  a5 4d 00 00 00 00 00 00 00 00 00 00 00 00 00 08
	//      ST2_DIF_DETAIL  a7 06 ff `ff 07 00 00 00 fe
	//
	// We do not retspond to the DIF_DETAIL message(s)
	// We will get a number of messages until the system calms down,
	// including some 0x1a5 4d's, and if we respond to them, we create
	// an endless loop of them.
	//
	// Therefore clients are careful to call this method except in response
	// to this one specific signature
	//  	dg[0] == 0xa5 && dg[1]==0x4d && dg[15] == 0x08

	uint16_t out_dg[MAX_ST_BUF];

	// minimal low order reply determined empirically

	out_dg[0] = 0x1a5;		// ST_SAT_DETAIL = 0x1a5
	out_dg[1]  = 0x4d;
	out_dg[2]  = 0x00;
	out_dg[3]  = 0x80;	// 0x80 works
	out_dg[4]  = 0x00;
	out_dg[5]  = 0x00;
	out_dg[6]  = 0x00;
	out_dg[7]  = 0x00;
	out_dg[8]  = 0x00;
	out_dg[9]  = 0x00;
	out_dg[10] = 0x00;
	out_dg[11] = 0x00;
	out_dg[12] = 0x00;
	out_dg[13] = 0x00;
	out_dg[14] = 0x00;
	out_dg[15] = 0x08;
		// 0x08, 0x10, and 0x18 all "works" to satisfy the Restart GPS button.
		// 		all of these show the default "Mode: Non-Differential"
		// If the 0x04 bit added it shows "Mode: Automatic Differential"
		// 		and then subsequently returns 0x0c (not 0x08) in the request
		//		which blows are signature.

	clearSTQueues();
	queueDatagram(E80_PORT2,out_dg);
	sendDatagram(E80_PORT2);
}



static void sendNeoST()
	// implementation copied from instST_out.cpp::gpsInst:;sendSeatalk()
	// and just fills up to ST limits in PRN order
{
	display(dbg_neo_ST,"sendNeoST(%d)",E80_PORT2);

	// handle device requires separate from instSimulator
	// we return version 1.99 to differentiate from instST_out which returns version 1.01
	
	uint16_t dg[MAX_ST_BUF];
	if (st_neo_device_query_pending)
	{
		uint8_t dev_id = 0xc5;	// Unit ID = RS125 GPS
		warning(0,"neoGPS sending DEV_QUERY(%02x)",dev_id);

		dg[0] = ST_DEV_QUERY;	// 0x1a4
		dg[1] = 0x12;      		// 0x10 constant + length
		dg[2] = dev_id;      	// Unit ID
		dg[3] = version;      	// Main SW version
		dg[4] = subversion;     // Minor SW version
		queueDatagram(E80_PORT2,dg);

		st_neo_device_query_pending = 0;
		return;
	}

	proc_entry();


	//-----------------------
	// ST_SAT_INFO
	//-----------------------
	// Use arbitray value of 0/5 satelites based on fix

	if (1)
	{
		// NOTE ST_SAT_INFO does not like numsats=11 (0xb0)
		// *perhaps* it is actually the number of sats used in the solution
		// and, thus would be limited to 9

		display(dbg_neo_ST+1,"neoGPS sending st_sat_info",0);
		dg[0] = ST_SAT_INFO;	// 0x157
		dg[1] = gps_model.fix_type ? 0x50 : 0x00;      		// num_sats USED=5 << 4
		dg[2] = gps_model.fix_type ? 0x02 : 0xff;          	// HDOP = 2
		queueDatagram(E80_PORT2,dg);
	}

	//------------------------------------------------
	// SAT_DET_INFO (A5 57)  — GPS Fix + HDOP block
	//------------------------------------------------
	// note we send hdop 3 from this as opposed to 2 above

	if (1)
	{
		display(dbg_neo_ST+1,"neoGPS sending st_sat_detail info",0);

		dg[0] = ST_SAT_DETAIL;		// // 0x1A5
		dg[1] = 0x57;

		// QQ
		// 		fix = QQ&0xF 						    = .... 00nn = 0x0nn = 1 "Fix"
		//		fix_available = QQ&0x10 			    = ...1 .... = 0x10 = 1
		// 		high 3 bits of numsats=5= QQ&0xE0/16  	= 010. .... = 0x40 = 2
		// Knauf uses QQ&0xE0/16 to mean (QQ&0xE0)>>4 in exprssion for numsats

		uint8_t QQ = 0;
		if (gps_model.fix_type > 0)
			QQ = (gps_model.fix_type & 0x0f) | 0x10 | 0x40;
		dg[2] = QQ;

		// HH:
		//		HDOP = HH&0x7C = HH & 01111100			= .000 11.. = 0x0C = 3
		//		HDOP availability = HH&0x80 		    = 1... .... = 0x80 = 1
		//		low bit of numsats = HH&0x01			= .... ...1 = 0x01 = 1
		//		numsats available = HH&0x02				= .... ..1. = 0x20 = 1
		// hdop = 3, hdop_available = 1, num_sats_available = 1, low bit of num_sats = 1, numsats available = 1

		int ihdop = roundf(gps_model.hdop);

		uint8_t HH = 0;
		if (gps_model.fix_type > 0)
			HH = 0x80 | ((ihdop & 0x3f) << 2) | 0x01 | 0x02;
		dg[3] = HH;

		dg[4] = 0x00;               // ?? = unknown
		dg[5] = 0x33;               // AA = antenna height; apparenly 0x33 is a constant that is seen in real Raystar GPS devices
		dg[6] = 0x20;               // GG = geoidal separation (32 * 16 = 512 m)
		dg[7] = 0x00;               // ZZ = differential age high bits
		dg[8] = 0x00;               // YY = diff age low bits + flags + station ID high bits
		dg[9] = 0x00;               // DD = station ID low

		queueDatagram(E80_PORT2,dg);
	}


	//-----------------------------------
	// SAT_DETAIL1..4 && SATS_USED
	//-----------------------------------

	if (gps_model.year > 0)
	{
		int num_total = 0;
		int num_used = 0;
		initStSatMessages();

		display(dbg_neo_ST+1,"neoGPS sending st_sat_details",0);
		proc_entry();

		for (int prn_m1=0; prn_m1<MAX_PRN && num_total < ST_MAX_VIEW; prn_m1++)
		{
			gps_sat_t *sat = &gps_model.sats[prn_m1];
			if (!sat->elev) continue;		// SEEN
			num_total++;

			bool used = sat->flags & SAT_USED_IN_SOLUTION;
			bool viewed = sat->flags & SAT_IN_VIEW;

			if (used)	// prevent used overflow
			{
				if (num_used<ST_MAX_TRACKED)
					num_used++;
				else
					used = 0;
			}

			// map snr to 0 for E80 to show "search" if !viewed and !used
			uint8_t snr = sat->snr;
			if (!used && !viewed) snr = 0;

			// add it
			display(dbg_neo_ST+2,"neoGPS sending prn(%d) elev(%d) asim(%d) snr(%d) used(%d)",
				prn_m1+1, sat->elev, sat->azim, snr, used);
			addStSatMessage(prn_m1+1, sat->elev, sat->azim, snr, used?2:0);
		}
		proc_leave();

		// send all 5 datagrams

		sendStSatMessags(E80_PORT2);
	}


	//----------------------------------
	// SATS_DONE
	//----------------------------------

	if (gps_model.year > 0)
	{
		display(dbg_neo_ST+1,"neoGPS sending st_sats_done",0);
		dg[0] = ST_SAT_DETAIL;
		dg[1] = 0x98;
		dg[2] = 0;
		dg[3] = 0;
		dg[4] = 0;
		dg[5] = 0;
		dg[6] = 0;
		dg[7] = 0;
		dg[8] = 0;
		dg[9] = 0;
		dg[10] = 0;
		queueDatagram(E80_PORT2,dg);
	}


	//------------------------------------------
	// LATLON
	//------------------------------------------

	if (1)
	{
		double lat = gps_model.lat;
		double lon = gps_model.lon;
		display(dbg_neo_ST+1,"neoGPS sending LatLon(%0.6f,%0.6f)",lat,lon);

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
		display(dbg_neo_ST+1+1,"i_lat(%d) frac_lat(%0.6f) min_lat(%0.6f) imin_lat(%d)",i_lat,frac_lat,min_lat,imin_lat);
		display(dbg_neo_ST+1+1,"i_lon(%d) frac_lon(%0.6f) min_lon(%0.6f) imin_lon(%d)",i_lon,frac_lon,min_lon,imin_lon);
		proc_leave();

		dg[0] = ST_LATLON;					// 0x158
		dg[1] = 0x5 | Z1 | Z2;
		dg[2] = i_lat;
		dg[3] = (imin_lat >> 8) & 0xff;
		dg[4] = imin_lat & 0xff;
		dg[5] = i_lon;
		dg[6] = (imin_lon >> 8) & 0xff;
		dg[7] = imin_lon & 0xff;
		queueDatagram(E80_PORT2,dg);
	}


	//------------------------------------------
	// COG/SOG
	//------------------------------------------

	if (gps_model.year > 0)
	{
		if (gps_model.sog >= 0)
		{
			float degrees = boat_sim.makeMagnetic(gps_model.cog);

			int halfs_total = roundf(degrees * 2.0);
			int nineties = halfs_total / 180;
			int rem = halfs_total % 180;
			int twos = rem / 4;
			int halfs = rem % 4;

			display(dbg_neo_ST+1,"neoGPS sending COG(%0.1f) = nineties(%d) twos(%d) halfs(%d)",degrees,nineties,twos,halfs);

			dg[0] = ST_COG;		// 0x153
			dg[1] = 0 | (nineties << 4) | (halfs<<6);
			dg[2] = twos;
			queueDatagram(E80_PORT2,dg);
		}

		if (gps_model.sog >= 0)
		{
			double speed = gps_model.sog ;
			int ispeed = (speed+ 0.05) * 10;
			display(dbg_neo_ST+1,"neoGPS sending SOG(%0.1f)",speed);

			dg[0] = ST_SOG;		// 0x152
			dg[1] = 0x01;
			dg[2] = ispeed & 0xff;
			dg[3] = (ispeed >> 8) & 0xff;
			queueDatagram(E80_PORT2,dg);
		}
	}


	//------------------------------------------
	// DATE and TIME
	//------------------------------------------

	if (gps_model.year > 0)
	{
		int y = gps_model.year % 100;
		int m = gps_model.month;
		int d = gps_model.day;

		display(dbg_neo_ST+1,"neoGPS sending Date(%02d/%02d/%02d)",y,m,d);
		dg[0] = ST_DATE;			// 0x156
		dg[1] = 0x01 | (m << 4);
		dg[2] = d;
		dg[3] = y;
		queueDatagram(E80_PORT2,dg);

		// RST is 12 bits (6 bits for minute, 6 bits for second)
		// T is four bits (low order four bits of second)
		// RS is eight bits (6 bits of minute followed by 2 bits of second)

		int s = gps_model.seconds;
		int h = gps_model.hour;
		int mm = gps_model.minute;

		uint16_t RST = (mm << 6) | s;
		uint16_t T = RST & 0xf;
		uint16_t RS = RST >> 4;

		display(dbg_neo_ST+1,"neoGPS sending Time(%02d:%02d:%02d)",h,mm,s);
		dg[0] = ST_TIME;				// 0x154
		dg[1] = 0x01 | (T << 4);
		dg[2] = RS;
		dg[3] = h;
		queueDatagram(E80_PORT2,dg);
	}

	proc_leave();

}	// sendNeoST()



//===================================================
//===================================================
// NMEA2000 Sender
//===================================================
//===================================================
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


static void sendNeo2000()
	// We use gps_model.year>0 both as a flag that the year is valid for
	// PGN 126992, but perhaps more importantly, as a flag that the
	// entire gps_model data structure is valid for PGN 129540
{
	tN2kMsg msg;

	int actual_num_viewed = 0;
	int actual_num_used = 0;
	for (int i=0; i<MAX_PRN; i++)
	{
		if (gps_model.sats[i].flags & SAT_IN_VIEW)
			actual_num_viewed++;
		if (gps_model.sats[i].flags & SAT_USED_IN_SOLUTION)
			actual_num_used++;
	}

	// PGN 129029 - GNSS Position Data

	if (gps_model.fix_type >= 1)
	{
		// teensy TimeLib.h has already been updated in the RMC parser:
		// ^^^ this may be a bad assumption since fix_type is not from RMC,
		// but at least they're valid values (not NA) in all cases below

		time_t t = now();
		uint32_t days = t / 86400;
		uint32_t secs = t % 86400;

		display(dbg_neo_2000,
			"129029 GNSS lat=%.6f lon=%.6f alt=%.1f fix=%d actual_num_used=%d hdop=%.1f vdop=%.1f pdop=%.1f",
			gps_model.lat,
			gps_model.lon,
			gps_model.altitude,
			gps_model.fix_type,
			actual_num_used,
			gps_model.hdop,
			gps_model.vdop,
			gps_model.pdop
		);

		SetN2kPGN129029(msg,
			255,							// SID (sequence ID)
			days,							// Days since 1970-01-01	gotten from teensy clock, whatever that is
			secs,							// Seconds since midnight	gotten from teensy clock, whatever that is
			gps_model.lat,					// Latitude (deg)			already mapped to NA if un-inited
			gps_model.lon,					// Longitude (deg)			already mapped to NA if un-inited
			gps_model.altitude,				// Altitude (m)				already mapped to NA if un-inited
			N2kGNSSt_GPS,					// GNSS type (GPS)
			N2kGNSSm_GNSSfix,				// GNSS method (GNSS fix)
			actual_num_used,				// Number of satellites used
			mapDoubleNA(gps_model.hdop),		// HDOP						turned into NA if < 0
			mapDoubleNA(gps_model.pdop),		// PDOP						turned into NA if < 0
			0,								// Geoidal separation (m)
			0,								// Position accuracy estimate (m)
			N2kGNSSt_GPS,					// Integrity type
			0,								// Reserved
			0 );							// Reserved
		nmea2000.SendMsg(msg);
	}


	// inst2000 instGPS sends PGN_DIRECTION_DATA 130577 here
	// we only send sog and cog, we don't presume to know the heading
	// or anything about the water speed, set, or drift

	if (gps_model.year > 0 && (gps_model.sog >= 0 || gps_model.cog >= 0))		// valid DT from GPS
	{
		// PGN_DIRECTION_DATA

		double sog_mps = gps_model.sog < 0 ? N2kDoubleNA : KnotsToms(gps_model.sog);

		SetN2kPGN130577(msg, 					// msg
			N2kDD025_Estimated,					// tN2kDataMode
			N2khr_true,							// tN2kHeadingReference,
			255,								// sid,
			mapDegToRadNA(gps_model.cog),			// COG in radians
			sog_mps,							// SOG in m/s
			N2kDoubleNA,						// heading in radians
			N2kDoubleNA,						// speed through water in m/s
			N2kDoubleNA,						// Set
			N2kDoubleNA);						// Drift
		nmea2000.SendMsg(msg);
	}

	if (gps_model.year > 0)		// valid DT from GPS
	{
		// PGN 126992 - System Time

		time_t now = time(NULL);
		uint32_t days = now / 86400;
		uint32_t secs = now - days * 86400;

		display(dbg_neo_2000,
			"126992 Time %04d-%02d-%02d %02d:%02d:%.1f",
			gps_model.year,
			gps_model.month,
			gps_model.day,
			gps_model.hour,
			gps_model.minute,
			gps_model.seconds
		);

		SetN2kPGN126992(msg,
			255,
			days,
			secs,
			N2ktimes_GPS
		);
		nmea2000.SendMsg(msg);
	}


	if (gps_model.year > 0)	// Valid (if partly unitialized) gps_model record from a cycle
	{
		// PGN 129540 - Satellites in View
		// we count the sats in view based on the SAT_IN_VIEW bit, and
		// only emit this message if the number is nonzero

		if (actual_num_viewed > 0)
		{
			display(dbg_neo_2000,"129540 SatsInView actual_num_viewed(%d)",actual_num_viewed);
			proc_entry();

			int sid = 7;  // or any fixed non-255 SID
			SetN2kPGN129540(msg,sid,N2kDD072_Unavailable);
				// N2kDD072_Unavailable == the GNSS receiver did not provide any integrity/differential status information

			for (int prn_m1=0; prn_m1<MAX_PRN; prn_m1++)
			{
				if (!(gps_model.sats[prn_m1].flags & SAT_IN_VIEW))
					continue;  // not in most recent GSV
				bool used = gps_model.sats[prn_m1].flags & SAT_USED_IN_SOLUTION ? 1 : 0;

				display(dbg_neo_2000 + 1,
					"PGN SAT PRN[%d] elev(%d) azim(%d) snr(%d) used(%d)",
					prn_m1 + 1,
					gps_model.sats[prn_m1].elev,
					gps_model.sats[prn_m1].azim,
					gps_model.sats[prn_m1].snr,
					used);

				tSatelliteInfo sat;
				if (0)
				{
					sat.PRN = prn_m1 + 1;
					sat.Elevation = DegToRad(gps_model.sats[prn_m1].elev);
					sat.Azimuth = DegToRad(gps_model.sats[prn_m1].azim);
					sat.SNR = gps_model.sats[prn_m1].snr;
					sat.RangeResiduals = N2kDoubleNA;
				}
				else
				{
					sat.PRN        = prn_m1 + 1;
					sat.Elevation  = mapDegToRadNA(gps_model.sats[prn_m1].elev);
					sat.Azimuth    = mapDegToRadNA(gps_model.sats[prn_m1].azim);
					sat.SNR        = mapUInt8NA(gps_model.sats[prn_m1].snr);
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
	// called from loop()
	// extern'd in instSimulator.h
{
	while (NEO_SERIAL->available())
	{
		#define MAX_0183_MSG 180

		int c = NEO_SERIAL->read();
		static char buf[MAX_0183_MSG+1];
		static volatile int buf_ptr = 0;
		last_receive_time = millis();

		if (neo_started)
		{
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
	}


	static int cycle_count;

	if (last_receive_time &&
		millis() - last_receive_time > FRAME_IDLE_TIME)
	{
		if (neo_started)
		{
			#if DBG_STATUS

				#define SHOW_EVERY		10

				if (cycle_count % SHOW_EVERY == 0)
				{
					Serial.print("neo status secs(");
					Serial.print(millis()/1000);
					Serial.print(") fix(");
					Serial.print(gps_model.fix_type);
					Serial.print(") num_viewed(");
					Serial.print(gps_model.num_viewed);
					Serial.print(") num_used(");
					Serial.print(gps_model.num_used);
					Serial.print(") hdop(");
					Serial.print(gps_model.hdop);
					Serial.print(") year(");
					Serial.print(gps_model.year);
					Serial.println(")");
		
					// a PRN has been "seen" if it's elev is > 0, which defines the "almanac" for the neo
					//
					// the sats IN_VIEW bit is cleared when we start a GSV cycle, the end of which causes a sendNeo()
					//		GSV sets the az,ele, and snr values for a satellite if provided
					//
					// the sats USED bit is set by a GSA message that occurs once per cycle, before, or after the GSV,
					//		but is cleared at the end of the cycle after calling sendNeo()
					//

					Serial.print("    Seen Sats: ");
					for (int i=0; i<MAX_PRN; i++)
					{
						if (gps_model.sats[i].elev)
						{
							Serial.print(i+1);
							Serial.print(" ");
						}
					}
					Serial.println();
					Serial.print("    View Sats: ");
					for (int i=0; i<MAX_PRN; i++)
					{
						if (gps_model.sats[i].flags & SAT_IN_VIEW)
						{
							Serial.print(i+1);
							Serial.print(" ");
						}
					}
					Serial.println();
					Serial.print("    Used Sats: ");
					for (int i=0; i<MAX_PRN; i++)
					{
						if (gps_model.sats[i].flags & SAT_USED_IN_SOLUTION)
						{
							Serial.print(i+1);
							Serial.print(" ");
						}
					}
					Serial.println();
				}
			#endif

			cycle_count++;
			if (st_enabled)
				sendNeoST();
			if (nmea2000_enabled)
				sendNeo2000();
		}
		else
		{
			display(dbg_neo,"skipping 0th cycle",0);
			cycle_count = 0;
		}
		
		initCycle();
		last_receive_time = 0;
		if (!neo_started)
			warning(dbg_neo,"NEO STARTED",0);
		neo_started = 1;
	}
}




// end of neo6M_GPS.cpp
