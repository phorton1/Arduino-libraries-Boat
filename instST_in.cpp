//---------------------------------------------
// instST_in.cpp
//---------------------------------------------
// Show (decode) datagrams.

#include "instST.h"
#include <myDebug.h>
#include "instSimulator.h"
#include "boatUtils.h"
#include "boatBinary.h"

#if WITH_NEO6M
	#include "neoGPS.h"
#endif


#define dbg_st7000	0

// notes
//
// even with sim_0183, which is the only one to get the E80 to
//		display a target wp name, the E80 only gives out internal
//		numberic target wp names


#define MAX_ST_NAME		12



typedef struct
{
	uint16_t	st;
	const char *name;
	int 		out_inst;

} 	st_info_type;

const st_info_type st_known[] =
{
	//                              name
	/* 0x100 */ { ST_DEPTH,			"DEPTH",		},
	/* 0x105 */ { ST_RPM,			"RPM",			},
	/* 0x110 */ { ST_WIND_ANGLE,	"WIND_ANGLE",	},
	/* 0x111 */ { ST_WIND_SPEED,	"WIND_SPEED",	},
	/* 0x120 */ { ST_WATER_SPEED,	"WATER_SPEED",	},
	/* 0x121 */	{ ST_TRIP,			"TRIP",			},
	/* 0x121 */	{ ST_LOG_TOTAL,		"TOTAL",		},
	/* 0x123 */ { ST_WATER_TEMPR,	"WATER_TEMP",	},
	/* 0x124 */ { ST_DISP_UNITS,	"DISP_UNITS",	},
	/* 0x125 */	{ ST_TRIP_TOTAL,	"TRIP_TOTAL",	},
	/* 0x126 */ { ST_LOG_SPEED,		"LOG_SPEED",	},
	/* 0x127 */ { ST_WATER_CELSIUS,	"WATER_CELS",	},
	/* 0x130 */ { ST_LAMP_INTENSITY,"LAMP_LEVEL",   },
	/* 0x150 */ { ST_LAT,			"LAT",			},
	/* 0x151 */ { ST_LON,			"LON",			},
	/* 0x152 */ { ST_SOG,			"SOG",			},
	/* 0x153 */ { ST_COG,			"COG",			},
	/* 0x154 */ { ST_TIME,			"TIME",			},
	/* 0x156 */ { ST_DATE,			"DATE",			},
	/* 0x157 */ { ST_SAT_INFO,		"SAT_INFO",		},
	/* 0x158 */ { ST_LATLON,		"LATLON",		},
	/* 0x159 */ { ST_59,			"59",			},
	/* 0x161 */ { ST_E80_SIG,		"E80_SIG",		},
	/* 0x182 */ { ST_TARGET_ID,		"TARGET_NAME",	},
	/* 0x184 */	{ ST_AUTOPILOT,		"AUTOPILOT",	},
	/* 0x185 */ { ST_NAV_TO_WP,		"NAV_TO_WP",	},
	/* 0x186 */ { ST_AP_KEYSTROKE,	"AP_KEY",		},
	/* 0x189 */ { ST_HEADING,		"HEADING",		},
	/* 0x197 */	{ ST_ST7000,		"ST7000",		},
	/* 0x198 */ { ST_AP_CPU,		"AP_CPU",		},
	/* 0x199 */ { ST_COMPASS_VAR,	"COMPASS_VAR",	},
	/* 0x19c */ { ST_RUDDER,		"RUDDER",		},
	/* 0x19e */	{ ST_WP_DEF,		"WP_DEF",		},
	/* 0x1a1 */ { ST_TARGET_NAME,	"TARGET_NAME",  },
	/* 0x1a2 */ { ST_ARRIVAL,		"ARRIVAL",		},
	/* 0z1a4 */	{ ST_DEV_QUERY,		"DEV_QUERY",	},
	/* 0z1a5 */	{ ST_SAT_DETAIL,	"SAT_DETAIL",	},
	/* 0x1a7 */	{ ST_DIF_DETAIL,	"DIF_DETAIL",	},
	/* 0x1ad */	{ ST_AD,			"AD"			},

	0,
};



static const char* keyName(bool longpress, uint8_t key_code)
{
	if (key_code == 0x01) return "Auto";
	if (key_code == 0x02) return "Standby";
	if (key_code == 0x03) return "Track";
	if (key_code == 0x04) return "Display";
	if (key_code == 0x05) return "-1";
	if (key_code == 0x06) return "-10";
	if (key_code == 0x07) return "+1";
	if (key_code == 0x08) return "+10";
	if (key_code == 0x09) return "-Resp";
	if (key_code == 0x0a) return "+Resp";
	if (key_code == 0x20) return "-1+1";
	if (key_code == 0x21) return "-1+10";
	if (key_code == 0x22) return "+1+10";
	if (key_code == 0x23) return "StandbyAuto";
	if (key_code == 0x24) return (longpress ? "-10+10" : "DisplayTrack");			// DisplayTrack longpress outputs 0x40 | 0x28 !!!
	if (key_code == 0x25) return "Standby-10";
	if (key_code == 0x28) return (longpress ? "DisplayTrack": "-10+10");			// -10+10 longpress outputs 0x40 | 0x24 !!!
	if (key_code == 0x2e) return "-Resp+Resp";
	if (key_code == 0x30) return "Standby-1";
	return "UNKNOWN";
}

static const char *deviceName(uint8_t device_id)
{
	if (device_id == 0x01) return "Depth";
	if (device_id == 0x02) return "Speed";
	if (device_id == 0x03) return "Multi";
	if (device_id == 0x04) return "Tridata";
	if (device_id == 0x05) return "Tridata Repeater";
	if (device_id == 0x06) return "Wind";
	if (device_id == 0x07) return "WMG";
	if (device_id == 0x08) return "Navdata GPS";
	if (device_id == 0x09) return "Maxview";
	if (device_id == 0x0A) return "Steering compass";
	if (device_id == 0x0B) return "Windtrim";
	if (device_id == 0x0C) return "Speedtrim";
	if (device_id == 0x0D) return "Seatalk GPS";
	if (device_id == 0x0E) return "Seatalk Radar ST50";
	if (device_id == 0x0F) return "Rudder Angle indicator";
	if (device_id == 0x10) return "ST30 Wind";
	if (device_id == 0x11) return "ST30 Bidata";
	if (device_id == 0x12) return "ST30 Speed";
	if (device_id == 0x13) return "ST30 Depth";
	if (device_id == 0x14) return "LCD Navcenter";
	if (device_id == 0x15) return "Apelco LCD Chartplotter";
	if (device_id == 0x16) return "Analog Speed";
	if (device_id == 0x17) return "Analog Depth";
	if (device_id == 0x18) return "ST30 Compass";
	if (device_id == 0x19) return "ST50 NMEA bridge";

	// Knauf's description does not match my E80's Device Enumeration here
	// if (device_id == 0xA8) return "ST80 Masterview";   <-- This should be the ST40 Wind

	// my own post Knauf additions

	if (device_id == 0x21) return "E80 Analogue Compass";
	if (device_id == 0x22) return "E80 Analogue Multitrim";
	if (device_id == 0x23) return "E80 Analogue Wind";
	if (device_id == 0x24) return "E80 Analogue CH/Wind";
	if (device_id == 0x25) return "E80 Rudder Angle Indicator";
	if (device_id == 0x26) return "E80 Masterview";
	if (device_id == 0x27) return "E80 Multiview";
	if (device_id == 0x28) return "Rate Gyro Compass";
	if (device_id == 0x2b) return "ST30 Round Bidata";
	if (device_id == 0x2c) return "Navcenter 700";
	if (device_id == 0x2d) return "Inboard Autpilot";

	if (device_id == 0x30) return "Maxview Display Heads";

    if (device_id == 0x41) return "ST80 Remote Keypad";
    if (device_id == 0x42) return "ST80 MOB Button";
    if (device_id == 0x43) return "ST80 Autopilot";
    if (device_id == 0x44) return "ST80 Masterkey";

    if (device_id == 0x51) return "ST80 Active Speed";
    if (device_id == 0x52) return "ST80 Active Depth";
    if (device_id == 0x53) return "ST80 Active deep Depth";
    if (device_id == 0x54) return "ST80 Active Wind";
    if (device_id == 0x55) return "ST80 Active Compass";
    if (device_id == 0x56) return "ST80 NMEA Bridge";

	if (device_id == 0x60) return "ST80 Course Computer";
    if (device_id == 0x61) return "SmartPilot";

	if (device_id == 0x70) return "ST60 Speed";
    if (device_id == 0x71) return "ST60 Depth";
    if (device_id == 0x72) return "ST60 TriData";
    if (device_id == 0x73) return "ST60 Wind Analogue Head";
    if (device_id == 0x74) return "ST60 CH/Wind Analogue Head";
    if (device_id == 0x75) return "ST60 Compass Analogue Head";
    if (device_id == 0x76) return "ST60 Multi";
    if (device_id == 0x77) return "ST60 Maxview";
    if (device_id == 0x78) return "ST60 Speed Sail RR";
    if (device_id == 0x79) return "ST60 Speed Power RR";
    if (device_id == 0x7a) return "ST60 Depth Feet RR";
    if (device_id == 0x7b) return "ST60 Depth Metres RR";
    if (device_id == 0x7c) return "ST60 Rudder Angle Indicator RR";
    if (device_id == 0x7d) return "ST60 Navigator";
    if (device_id == 0x7e) return "ST60 Wind Round Rptr";
    if (device_id == 0x7f) return "ST60 Compass Round Rptr";

	if (device_id == 0x80) return "ST6000 Control Unit";
    if (device_id == 0x81) return "ST7000 Control Unit";
    if (device_id == 0x82) return "ST60 GPS HEad";
    if (device_id == 0x83) return "ST60 Wired HHC";
    if (device_id == 0x84) return "ST60 RF HHC";
    if (device_id == 0x85) return "Raydata";
    if (device_id == 0x86) return "ST60/80 RR Group Code";
    if (device_id == 0x87) return "Zodiac GPS";
    if (device_id == 0x88) return "ST5/5000 (ST60";
    if (device_id == 0x89) return "Raychart 620";
    if (device_id == 0x8a) return "Navcenter 600";
    if (device_id == 0x8b) return "ST60 Rudder Angle Indicator";

	if (device_id == 0x90) return "SL70 Radar Standalone";
    if (device_id == 0x91) return "SV7 Radar";
    if (device_id == 0x92) return "RL70";
    if (device_id == 0x93) return "RL70C";
    if (device_id == 0x94) return "RC520";
    if (device_id == 0x95) return "R70";
    if (device_id == 0x96) return "R70RC";
    if (device_id == 0x97) return "RL70/RL80C";
    if (device_id == 0x98) return "RL70RC/RL80RC";
    if (device_id == 0x99) return "RC530/RC631";

	if (device_id == 0xa0) return "RS112LP GPS";
    if (device_id == 0xa1) return "RS114 GPS";
    if (device_id == 0xa2) return "Raychart 830";
    if (device_id == 0xa5) return "ST40 Speed";
    if (device_id == 0xa6) return "ST40 Depth";
    if (device_id == 0xa7) return "ST40 Bidata";
    if (device_id == 0xa8) return "ST40 Wind";
    if (device_id == 0xa9) return "ST40 Compass";

	if (device_id == 0xb0) return "L755";
    if (device_id == 0xb1) return "L760";
    if (device_id == 0xb2) return "L1250";
    if (device_id == 0xb3) return "L1250RC";
    if (device_id == 0xb4) return "L760";

	if (device_id == 0xc0) return "RN300 Navigator";
    if (device_id == 0xc1) return "RN301 Navigator";
    if (device_id == 0xc2) return "RC320 Chartplotter";
    if (device_id == 0xc3) return "RC321 Chartplotter";
    if (device_id == 0xc4) return "RS120 GPS";
    if (device_id == 0xc5) return "RS125 GPS";
    if (device_id == 0xc7) return "C70 Display";
    if (device_id == 0xc8) return "C80 Display";
    if (device_id == 0xc9) return "C120 Display";
    if (device_id == 0xca) return "E80 Display";
    if (device_id == 0xcb) return "E120 Display";
    if (device_id == 0xcc) return "Entry Level Pilot";
    if (device_id == 0xcd) return "New ST4000";
    if (device_id == 0xce) return "RF - Base Station";
    if (device_id == 0xcf) return "RF - S100 FOB";

	if (device_id == 0xd0) return "RF - Smart Controller";
    if (device_id == 0xd1) return "ST8000 Control Unit (ST60)";
    if (device_id == 0xd3) return "GPM400 (US)";
    if (device_id == 0xd4) return "GPM400 (EU)";
    if (device_id == 0xd5) return "GPM400 (ROW)";
	
	return "UNKNOWN";
}





static String decodeST(bool out, uint16_t st, const uint8_t *dg, const char **p_name)
{
	String retval;

	if (st == ST_DEPTH)		// 0x100
	{
		float feet = dg[4]*256 + dg[3];
		feet /= 10;
		retval = "feet(";
		retval += feet;
		retval += ")";
	}
	else if (st == ST_RPM)			// 0x105
	{
		int16_t rpm = (int16_t) ((uint16_t)(dg[3]*256 + dg[4]));
		int8_t pitch = (int8_t) dg[5];
		retval = "rpm(";
		retval += rpm;
		retval += ") pitch(";
		retval += pitch;
		retval += ")";
	}
	else if (st == ST_WIND_ANGLE)	// 0x110
	{
		float angle = dg[2]*256 + dg[3];
		angle /= 2;
		retval += "apparent_angle(";
		retval += angle;
		retval += ")";
	}
	else if (st == ST_WIND_SPEED)	// 0x111
	{
		float speed = dg[3];
		speed /= 10;
		speed += (dg[2] & 0x7f);
		retval = "apparent_knots(";
		retval += speed;
		retval += ")";
	}
	else if (st == ST_WATER_SPEED)	// 0x120
	{
		float speed = dg[3]*256 + dg[2];
		speed /= 10;
		retval += "knots(";
		retval += speed;
		retval += ")";
	}
	else if (st == ST_TRIP)			// 0x121
	{
		uint32_t trip = dg[4]&0x0f;
		trip <<= 16;
		trip |= dg[3];
		trip <<= 8;
		trip |= dg[2];

		float f_trip = trip;
		f_trip /= 100;
		char buf[20];
		sprintf(buf,"trip(%0.2f)",f_trip);
		retval = buf;
	}
	else if (st == ST_LOG_TOTAL)		// 0x122
	{
		uint32_t total = dg[4];
		total <<= 8;
		total |= dg[3];
		total <<= 8;
		total |= dg[2];
			// knaufman's spec is wrong here, he ignores the last byte

		float f_total = total;
		f_total /= 10;
		char buf[20];
		sprintf(buf,"total(%0.2f)",f_total);
		retval = buf;
	}
	else if (st == ST_WATER_TEMPR)		// 0x123
	{
		const char *err = dg[1] & 4 ?
			" defective/not connected" : "";
		char buf[40];
		sprintf(buf,"%dC %dF",dg[2],dg[3]);
		retval = buf;
		retval += err;
	}
	else if (st == ST_DISP_UNITS)		// 0x124
	{
		retval = (
			dg[4] == 0x86 ? "km/kmph" :
			dg[4] == 0x06 ? "imperial_miles/imph" :
			"nm/knots" );
	}
	else if (st == ST_TRIP_TOTAL)		// 0x125
	{
		uint32_t total = dg[1] & 0xf0;
		total <<= 4;
		total |= dg[3];
		total <<= 8;
		total |= dg[2];

		uint32_t trip = dg[6] & 0x0f;
		trip <<= 8;
		trip |= dg[5];
		trip <<= 8;
		trip |= dg[4];

		float f_total = total;
		float f_trip = trip;
		f_total /= 10;
		f_trip /= 100;
		char buf[40];
		sprintf(buf,"total(%0.1f) trip(%0.2f)",f_total,f_trip);
		retval = buf;
	}
	else if (st == ST_LOG_SPEED)	// 0x126
	{
		float speed = dg[3]*256 + dg[2];
		float avg = dg[5]*256 + dg[4];
		speed /= 100;
		avg /= 100;
		uint8_t flags = dg[6];
			// my flags are 0x10 and the speed
			// appears valid even if !0x04
		if (1)	// flags & 0x04)
		{
			retval += "knots(";
			retval += speed;
			retval += ") ";
		}
		if (flags & 0x08)
		{
			retval += "speed2(";
			retval += avg;
			retval += ")";
		}
		else
		{
			retval += "avg(";
			retval += avg;
			retval += ")";
		}
	}
	else if (st == ST_WATER_CELSIUS)		// 0x127
	{
		uint32_t cels = dg[3];
		cels <<= 8;
		cels |= dg[2];

		float tempr = cels - 100;
		tempr /= 10;
		char buf[12];
		sprintf(buf,"%0.1fC",tempr);
		retval = buf;
	}
	else if (st == ST_LAMP_INTENSITY)		// 0x130
	{
		//  30  00  0X      Set lamp Intensity; X=0: L0, X=4: L1, X=8: L2, X=C: L3
        //  (only sent once when setting the lamp intensity)
		const char *level =
			dg[2] == 0x0c ? "high" :
			dg[2] == 0x08 ? "medium" :
			dg[2] == 0x04 ? "low" :
			"off";
		retval = level;
	}
	else if (st == ST_LAT)			// 0x150
	{
		float lat = dg[2];
		uint16_t YYYY = dg[4];
		YYYY <<= 8;
		YYYY |= dg[3];

		bool south = YYYY & 0x8000 ? 1 : 0;
		YYYY &= 0x7fff;
		float mins = YYYY;
		mins /= 100;

		lat += (mins / 60.0);
		if (south) lat = -lat;
		retval = "lat(";
		retval += strDegreeMinutes(lat);
		retval += ")";
	}
	else if (st == ST_LON)			// 0x150
	{
		float lon = dg[2];
		uint16_t YYYY = dg[4];
		YYYY <<= 8;
		YYYY |= dg[3];

		bool east = YYYY & 0x8000 ? 1 : 0;
		YYYY &= 0x7fff;
		float mins = YYYY;
		mins /= 100;

		lon += mins / 60.0;
		if (!east) lon = -lon;
		retval = "lon(";
		retval += strDegreeMinutes(lon);
		retval += ")";
	}
	else if (st == ST_SOG)			// 0x152
	{
		float speed = dg[3]*256 + dg[2];
		speed /= 10;
		retval += "knots(";
		retval += speed;
		retval += ")";
	}
	else if (st == ST_COG)			// 0x153
	{
		float cog = (dg[1] * 0x0c);
		cog /= 8;
		cog += ((dg[1] & 0x30) >> 4)*90 + (dg[1] & 0x3f) * 2;
		retval = "cog(";
		retval += cog;
		retval += ")";
	}
	else if (st == ST_TIME)			// 0x154
	{
		uint8_t T = dg[1] >> 4;
		uint8_t RS = dg[2];
		uint8_t ST = (RS << 4) | T;

		uint8_t hour = dg[3];
		uint8_t minutes = (RS & 0xfc) / 4;
		uint8_t secs = ST & 0x3F;
		char buf[32];
		sprintf(buf,"time(%02d:%02d:%02d)",hour,minutes,secs);
		retval = buf;
	}
	else if (st == ST_DATE)			// 0x156
	{
		uint16_t year = dg[3] + 2000;
		uint8_t day = dg[2];
		uint8_t month = dg[1]>>4;
		char buf[32];
		sprintf(buf,"date(%04d-%02d-%02d)",year,month,day);
		retval = buf;
	}
	else if (st == ST_SAT_INFO)		// 0x157
	{
		uint8_t num_sats = dg[1] >> 4;
		retval = "num_sats(";
		retval += num_sats;
		retval += ")";
		if (num_sats > 1)
		{
			uint8_t hdop = dg[2];
			retval += " hdop(";
			retval += hdop;
			retval += ")";
		}
	}
	else if (st == ST_LATLON)		// 0x158
	{
		// degrees
		double lat = dg[2];
		double lon = dg[5];
		// minutes
		double lat_mins = dg[3]*256.0 + dg[4];
		double lon_mins = dg[6]*256.0 + dg[7];
		lat_mins /= 1000.0;
		lon_mins /= 1000.0;
		// minutes to fractional degrees
		lat += lat_mins/60.0;
		lon += lon_mins/60.0;
		// sign
		if (dg[1] & 0x10) lat = -lat;
		if (!(dg[1] & 0x20)) lon = -lon;

		retval = "lat(";
		retval += strDegreeMinutes(lat);
		retval += ")  lon(";
		retval += strDegreeMinutes(lon);
		retval += ")";
	}
	else if (st == ST_TARGET_ID)	// 0x182
	{
		uint8_t XX = dg[2];
		uint8_t YY = dg[4];
		uint8_t ZZ = dg[6];
		uint8_t char1 = XX & 0x3f;
		uint8_t char2 = (YY & 0xf)*4 + (XX & 0xc0)/64;
		uint8_t char3 = (ZZ & 0x3)*16 + (YY & 0xf0)/16;
		uint8_t char4 = (ZZ & 0xfc)/4;
		retval = "id(";
		retval += (char) (char1 + 0x30);
		retval += (char) (char2 + 0x30);
		retval += (char) (char3 + 0x30);
		retval += (char) (char4 + 0x30);
		retval += ")";
	}
	else if (st == ST_AUTOPILOT)	// 0x184
	{
		// see notes in instST_out.cpp regarding of the ST_AUTOPILOT
		// datagram for what I perceive to be an inherent bug in knauf's
		// description and his overloading of the high bit of the U nibble

		uint8_t U  = dg[1] >> 4;
		uint8_t VW = dg[2];
		uint8_t XY = dg[3];
		uint8_t Z  = dg[4];				// Always 0x4Z on my ST7000
		uint8_t M  = dg[5];
		uint8_t RR = dg[6];
		uint8_t SS = dg[7];
		uint8_t TT = dg[8];				// TT is always 0x08 from my ST7000 ap-cpu
		uint8_t V  = VW >> 4;

		// Decode heading using my corrected interpretation:
		//   - low 2 bits of U = quadrant
		//   - low 6 bits of VW = "twos"
		//   - high 2 bits of U = half-degrees (0..3)

		float heading = (U & 0x3) * 90.0f;			// quadrant
		heading += (VW & 0x3F) * 2.0f;				// twos
		heading += ((U >> 2) & 0x3) * 0.5f;			// half-degrees
		if (heading >= 360.0f) heading -= 360.0f;

		float course = (V >> 2)*90 + ((float)XY / 2.0f);
		int8_t rudder = (int8_t)RR;

		// Note that the Z BYTE always has 0x40 (4 in the high nibble) from my ST7000 ap-cpu
		// and that the ST7000 head DEPENDS on it to properly display the rudder indicator

		const char *mode="STBY";
		if (Z & 0x08) mode = "TRACK";
		else if (Z & 0x04) mode = "VANE";
		else if (Z & 0x02) mode = "AUTO";

		// --- Build retval ---
		retval  = mode;
		retval += " head(";
		retval += heading;
		retval += " mag) rud(";
		retval += rudder;
		retval += ") ap(";
		retval += course;
		retval += " mag) alarm(";
		retval += M;
		retval += ") flags(";
		retval += SS;
		retval += ") TT(";
		retval += TT;
		retval += ")";
	}
	else if (st == ST_NAV_TO_WP)	// 0x185
	{
		uint16_t XXX = dg[2];
		XXX <<= 4;
		XXX |= dg[1] >> 4;
		float xte = XXX;
		xte /= 100;

		uint8_t VU = dg[3];
		uint8_t ZW = dg[4];
		uint8_t ZZ = dg[5];
		uint8_t YF = dg[6];

		uint8_t WV = (ZW << 4) | (VU >> 4);
		float part = WV;
		part /= 2;
		float bearing = (VU & 0x03);
		bearing *= 90;
		bearing += part;

		const char *b_type = VU & 0x08 ? "true" : "magnetic";

		uint8_t Y = (YF>>4);
		uint16_t ZZZ = (ZZ << 4) | (ZW >> 4);
		ZZZ |= ZZ;
		float range = ZZZ;
		range /= (Y&1) ? 100.0 : 10.0;

		bool has_xte = YF & 0x01;
		bool has_bearing = YF & 0x02;
		bool has_range = YF & 0x04;

		retval = "xte(";
		retval += xte;
		retval += ") ";
		retval += b_type;
		retval += " bearing(";
		retval += bearing;
		retval += ") range(";
		retval += range;
		retval += ") flags(";
		retval += has_xte ? "X" : " ";
		retval += has_bearing ? "B" : " ";
		retval += has_range ? "R" : " ";
		retval += ")";
	}
	else if (st == ST_AP_KEYSTROKE)	// 0x186
	{
		// This long comment eventually belongs in documentation and not in the code
		//
		// tested ST7000 is complicated
		//		- buttons are not simple "sends"; some affect the state of the ST7000 itself
		//		- buttons are sometimes modal and affect the state of the ST7000 depending on the mode
		//		- some single keypresses appear to be modeless
		// 		- some are sent on press, some on up, and some after an interval
		//      - the ST7000 appears to send out the ST_ST7000=0x197 00 00 datagram in some error combinations
		//		  possibly to get the cpu to affirm its state (even though I believe the cpu sends state once per second)
		//
		// I referred back to the ST7000 "operation handbook" to see what valid
		// combinations they present:
		//
		// Autotack - operates in both "compass=AUTO" and "VANE" modes
		//		+1 & +10 = tack 100 degrees to starboard
		//		-1 & -10 = tack 100 degrees to port
		// Wind Trim = "vane" mode
		//		STANDBY & AUTO = enter "vane" mode and maintain current apparent wind angle
		//			If the calculated ap_heading changes by more than 15 degrees a
		//			WIND_CHANGE_ALARM will be issued, and STANDBY & AUTO (brief) will
		//				clear it.
		//			VANE mode requires one minute of changed apparent wind before it
		//				adjusts the ap course
		//		STANDBY & AUTO long = return to "previous apparent wind angle"
		//			but the documentation is unclear as it apears to labelled "Previous Automatic Heading"
		//			and appears to possibly return to "auto" mode.
		//
		// And from the "installation handbook"
		//
		//	TRACK and DISPLAY (short) then RESP+/- to adjust Display Contrast
		//
		//	the following about "calibration"
		//		in STANDBY mode (press STANDBY first)
		//
		//		"to select calibration mode"
		//			TRACK and DISPLAY long (2 seconds) to (display calibration settings?)
		//		"to enter calibration mode"
		//			TRACK and DISPLAY long (again 2 seconds) to entier calibration mode
		//			I am not 100% sure they meant to repeat this as it appears that you
		//			can edit values possibly in "calibration display mode"
		//		"to save changes  made in calibration mode"
		//			TRACK and DISPLAY long (again 2 seconds) to save mode
		//		"to exit calibration display/modification modes without saving"
		//			press STANDBY
		//
		// 		When in calibration mode, the DISPLAY button will traverse through values and
		//			RESP+ and RESP- will modify them.
		//
		//	Rudder Gain Adjustment
		//		Pressing Resp+ and Resp- for one second will go into Rudder Gain Adjustment
		//		mode which will "adjust to either side of the calibrated setting" for
		//		"optimal autopilot steering".
		//
		// IMPLEMENTATION
		//
		//	The Seatalk Autopilot instrument is special, and is the only case where
		//		an "input" message affects the "state" of a virtual simulated instrument.
		//		IFF a Seatalk apInst is being simulated on the ST1 port (which *may* be
		//		forwarded to the E80 on the ST2 port) then
		//
		//		- it "responds" to keypresses
		//		- it "drives" the boatSimulator autopilot to some degreee
		//		- it has the ability to emulate parameter setting mode
		//		- it generally emulates a subset of the real ap-cpu to facilitate
		//		  testing of the ST7000.
		//
		//	This allows me to work on, understand, and verify encoding and decoding
		//		of the ST7000<->ap-cpu conversation while working on my desk instead of
		//		the cramped confines of the nav station.
		//
		// 	To implement this and understand the keystrokes, I first modified boatSimulator
		//		m_autopilot to be a uint8_t and added 2=AP_VANE_MODE, currently unimplemented
		//		in the boatSimulator, to the to be able to set "VANE" mode on the ST7000.


		//--------------------------------------------------------
		// This description supercede's Knauf's description
		//--------------------------------------------------------
		// byte2 is the only relevant byte
		// byte3 is a checksum = 0xff minus byte2
		// here we assert the checksum

		uint8_t key = dg[2];
		uint8_t cs  = 0xff - key;
		if (dg[3] != cs) warning(0,"invalid key(0x%02x) checksum(0x%02x) expected(0x%02x)",key,cs,dg[3]);

		// A long press is indicated by the 0x40 bit

		#define LONG_BIT 0x40
		bool longpress = key & LONG_BIT ? 1 : 0;
		uint8_t single_key = key & ~LONG_BIT;

		// Single Keys
		//		These are modeless in terms of being output
		//			- they beep and possibly output a short press in any mode
		//			  NOT ALL KEYS OUTPUT SHORTS BEFORE LONGS
		//			- they will beep and output a long press in any mode
		//		They do not, in general, affect the state of the ST7000,
		//			with the exception of Display when in AUTO/VANE/TRACK modes
		//			which will affect the display, but not the behavior, of the ST7000
		//		Not all Single Keys "do" something in any mode.
		//			Behavior is byond the scope of this comment to describe.
		//
		//		Name			SHORT_BEFORE_LONG
		//		-------------------------------------------------------------
		// 		01 = Auto		yes
		// 		02 = Standby	no
		// 		03 = Track		no
		// 		04 = Display	no
		// 		05 = -1			yes
		// 		06 = -10		yes
		// 		07 = +1			yes
		// 		08 = +10		yes
		// 		09 = -Resp		yes
		// 		0a = +Resp		yes
		//
		// Combined Keys
		//
		//		KEY_NAME 				Ref			SHORT_B4_LONG	Notes
		//		------------------------------------------------------------------
		// 		20 = -1+1				yes
		//		21 = -1+10				yes
		//		22 = +1+10				yes
		// 		23 = StandbyAuto		yes
		//    	24 = DisplayTrack		no			longpress outputs 0x40 | 0x28 !!!
		// 		25 = Standby-10			yes
		// 		28 = -10+10				no 			longpress outputs 0x40 | 0x24 !!!
		//		2e = -Resp+Resp			long_only	"Rudder Gain Adjustment" for planing vessels
		// 		30 = Standby-1			yes			not reliable; enters funny mode; crashes
		//
		// Notes:
		//
		//		- Display_Track(short) will output the keystroke, then enter a
		//		  	local ST7000 mode to adjust the Display Contrast using Resp-/+
		//		  	and or the +/- 1 & 10 keys.
		//		  While in this mode the Resp-/+ keys are eaten by the ST7000
		//			wheras the 1/10 and other keys are not.
		//		  The mode is left by pressing Display_Track(short) again
		//			or Standby; other keys do not leave this local mode.
		//		- There does not seem to be a "Factory Reset Button'.
		//		- Backlight: The documentation says to press the Display button for
		//		  one second, then press the Display button again within 10 seconds
		//		  to toggle through illumination values.  This does not happen locally
		//		  on the ST7000 as you might expect, so it must be that the ap-cpu is
		//		  interpreting those keys and putting the ST_LAMP datagrams on the bus.
		//      - Funny Mode
		//	    	The Standby_Minus1 button ia not reliable and repeated
		//				presses of it have led to the ST7000 backlights coming on
		//				and the display showing all segments and/or crashing and
		//				possibly rebooting the ST7000
		//		- A search for the "missing" 0x26,0x27, and 0x29 or other
		//			combined keycode values did not yield any fruitful results.

		const char *key_name = keyName(longpress,single_key);
		char buf[50];
		sprintf(buf,"key(0x%02x) long(%d) single_key(0x%02x) = %s",key,longpress,single_key,key_name);
		retval = buf;
		display(dbg_st7000,buf,0);

	}
	else if (st == ST_HEADING)		// 0x189
	{
		uint8_t U = dg[1] >> 4;
		uint8_t VW = dg[2];
		float heading = U & 0xC;
		heading /= 8;
		heading += (U & 0x3) * 90;
		heading += (VW & 0x3F) * 2;
		retval = "heading(";
		retval += heading;
		retval += ") magnetic";
	}
	else if (st == ST_ST7000)		// 0x197	undocumented by Knauf
	{
		// The ST7000 autopilot head sends 0x197 00 00 at startup
		// and under certain keystroke error conditions as a probe
		// to determine if the autopilot cpu is present.
		//
		// As a first step towards possibly emulating the conversation
		// between the ap-cpu (our simlated Seatalk apInst), we use a
		// global variable here in instST_in.cpp to communicate with
		// instST_out.cpp to emulate the handshaking.

		warning(0,"ST_ST7000 setting ap_linked=1",0);
		ap_linked = 1;
		retval = "ST7000";
	}
	else if (st == ST_AP_CPU)		// 0x198	undocumented by Knauf
	{
		// The ST7000 autopilot cpu (model 400?) sends 0x198 00 00 in
		// response to receiving the ST_ST7000 0x197 00 00 datagram.
		retval = "AP_CPU";
	}
	else if (st == ST_COMPASS_VAR)	// 0x199
	{
		char buf[20];
		int8_t var = (int8_t) dg[2];
		const char *direction = "west";
		if (var < 0)
		{
			var = -var;
			direction = "east";
		}
		sprintf(buf,"%s(%d)",direction,var);
		retval = buf;
	}
	else if (st == ST_RUDDER)		// 0x19c
	{
		// 9C  U1  VW  RR  - see 0x184 above
		// note that I had done this using floats previously
		// in my original earliest ST_HEADING block above.
		//
		// see notes in instST_out.cpp regarding of the ST_AUTOPILOT
		// datagram for what I perceive to be an inherent bug in knauf's
		// description and his overloading of the high bit of the U nibble

		uint8_t U  = dg[1] >> 4;
		uint8_t VW = dg[2];
		uint8_t RR = dg[3];

		int8_t rudder = (int8_t)RR;

		// Decode heading using my corrected interpretation:
		//   - low 2 bits of U = quadrant
		//   - low 6 bits of VW = "twos"
		//   - high 2 bits of U = half-degrees (0..3)
		//
		// Knauf's "odd bit" description is wrong for the ST7000/E80 generation.

		float heading = (U & 0x3) * 90.0f;			// quadrant
		heading += (VW & 0x3F) * 2.0f;				// twos
		heading += ((U >> 2) & 0x3) * 0.5f;			// half-degrees
		if (heading >= 360.0f) heading -= 360.0f;

		retval = "head(";
		retval += heading;
		retval += ") rud(";
		retval += rudder;
		retval += ")";
	}
	else if (st == ST_TARGET_NAME)		// 0x1a1
	{
		int part = (dg[1] & 0xf0) >> 4;
		static char st_name[20];
		sprintf(st_name,"Z%c%c%c%c%c%c_%X",
			dg[2],
			dg[3],
			dg[4],
			dg[5],
			dg[6],
			dg[7],
			part);
		*p_name = st_name;
	
		retval = " name(";
		for (int i=0; i<8 && dg[8+i]; i++)
			retval += (char) dg[8+i];
		retval += ")";
	}
	else if (st == ST_ARRIVAL)		// 0x1a2
	{
		// #define ST_ARRIVAL		0x1A2
		//	A2 X4 00 WW XX YY ZZ Arrival Info
		//		X&0x2=Arrival perpendicular passed, X&0x4=Arrival circle entered
		//		WW,XX,YY,ZZ = Ascii char's of waypoint id.   (0..9,A..Z)
		//		   Takes the last 4 chars of name, assumes upper case only
		//		Corresponding NMEA sentences: APB, AA

		bool perp = dg[1] & 0x20 ? 1 : 0;
		bool circ = dg[1] & 0x40 ? 1 : 0;
		String wp_name;
		wp_name += (char) (dg[3]);	//  + 0x30);
		wp_name += (char) (dg[4]);	//  + 0x30);
		wp_name += (char) (dg[5]);	//  + 0x30);
		wp_name += (char) (dg[6]);	//  + 0x30);
		retval = wp_name;
		if (perp) retval += " perp";
		if (circ) retval += " circ";
	}
	else if (st == ST_DEV_QUERY)	// 0x1a4
	{
		// st_device_query_pending Set when the system receives the ST_DEV_QUERY
		// device query from the E80, in which case, ST specific code
		// sends out 0x1a4 12 ID VV vv replies and clears the boolean

		if (dg[1] == 0x06 || dg[1] == 0x02)
		{
			retval = "QUERY";
			st_device_query_pending = 1;
			#if WITH_NEO6M
				st_neo_device_query_pending = 1;
			#endif
		}
		else if (dg[1] == 0x12)
		{
			static char name[50];
			static char version[50];
			sprintf(name,"ST_DEV_%02x",dg[2]);
			sprintf(version,"%d.%02d",dg[3],dg[4]);
			retval = deviceName(dg[2]);
			retval += " ";
			retval += version;
			*p_name = name;
		}
		else
		{
			retval = "???";
		}
	}


	else if (st == ST_SAT_DETAIL ||	// 0x1a5
			 st == ST_DIF_DETAIL)	// 0x1a7
	{
		//  my Correction/Rewrite of knaufs A5 comment
		//  A5  GPS and DGPS Info
		// 	A5 57 QQ HH ?? AA GG ZZ YY DD   GPS and DGPS Fix Info (mostly unchanged from knauf)
		//		Fix Type = QQ&0xF where 0=none, 1=fix, 2=D Fix;  QQ&0x10: Fix Type available flag
		//		HDOP= HH&0x7C, HH&0x80: HDOP available flag
		//		Antenna Height= AA
		//		Number of Sats= (QQ&0xE0)/16+(HH&0x1), HH&0x2: NumSats available flag
		//		GeoSeperation= GG*16 (-2048....+2047 meters)
		//		Differential age=(ZZ&0xE0)/2+(YY&0xF), YY&0x10: Diff. age available flag
		//		Differential Station ID=(YY&0xC0)*4+DD, YY&0x20: Diff.St.ID available flag
		//		Corresponding NMEA sentences: GGA, RMC, GSV, GLL, GGA
		//	A5 XQ NN AA EE SS MM BB FF GG OO CC DD XX YY ZZ   GPS Info: Sat Position and Signal
		//			Data of up to three satellites [1,2,3] per datagram, or 11 in total accross 4 datagrasm in a series
		//			where the subtype/length byte XQ is one of the four values 0x0D, 0x0C, 0x2D, or 0x8D
		//           Satellite PRN: [1] (NN&0xFE)>>1, [2] (MM&0x70)/2+(BB&0x7), [3] CC&0x3F
		//           Satellite azimuth:[1] AA*2+(EE&0x1), [2] (BB&0xF8)*2+(FF&0xF), [3] (CC&0xC0)*2+(DD&0x7F)
		//           Satellite elevation:[1] (EE&0xFE)/2, [2] (FF&0xF0)/2+(GG&0x7), [3] XX&0x7F
		//           Satellite signal: [1] (SS&0xFE)/2 with low 3 bits in (MM&0x7), [2] (GG&0x80)/2+OO&0x3F,  [3] (YY>>1)
		//           Entry valid:     [1] MM&0x8, [2] OO&0x40, [3] ZZ&0x2
		//		There can be upto four similar messages in the series.  The first with XQ=0x0D carries the position
		//      and signal data of the 1st 3 satellites. The second XQ=0x0C and the NN&0x1 bit set (A5 0C subtype)
		//      carries the data of the next 2 satellites, but instead of a 3rd satellite, this shortened message
		//		carries the PRN_USED values	(see below) in CC DD XX YY for the 1st 4 sats used in the solution (see below).
		//      The third datagram, XQ=0x2D carries data of 3 more sats [6,7,8], and finally the fourth datagram with
		//		XQ=0x8D	carries data of sats [9,10,11].
		//  A5 74 PU PU PU PU PU  - Identification of additional PRNS used in the solution
		//		This message carries upto 5 additional PRN_USED values over the initial
		//		four in the A5 0C message.
		//  A5 98 00 00 00 00 00 00 00 00 00 - End of A5 Series messages, completes the transmission of the A5 series of messages
		//	A5 B5 00 04 80 00 00 00 - Explicitly Specifies the WGS 1984 Datum isin use
		//	A5 - PRN_USED bytes:  The A5 0C and A5 74 message contain PRN_USED bytes that indicate which sat PRNS
		//		were used in the solution, where PU (or CC DD XX YY for A5-0C) = PRN | 0x80
		//
		//  A7 09 NN AA EE SS MM 00 00 00 00 76	 Special SBAS/Differential message
		//   i.e  85 82 47 42 8B 00 00 00 00 76  This message reports a single satellite used for SBAS/DGPS.
		//     	in a shortened form very similar to the A5 XQ series
		//		The PRN in this message, NN is not shifted; the entire byte NN is the PRN.
		//		See A5 XQ  messages above.


		if (dg[1] == 0x98)
			*p_name = "SATS_DONE";
		else if (dg[1] == 0xb5)
			*p_name = "SAT_WGS84";
		else if (dg[1] == 0x57)
		{
			*p_name = "SAT_DET_INF";
			bool fflag = dg[2] & 0x10 ? 1 : 0;
			uint8_t fix = dg[2] & 0x0f;

			bool hflag = dg[3] & 0x80 ? 1 : 0;
			uint8_t hdop = (dg[3] & 0x7c)>>2;

			bool nflag = dg[3] & 0x2 ? 1 : 0;
			uint8_t nsats = ((dg[2] & 0xe0) >> 4) + (dg[3] & 0x01);

			uint8_t question = dg[4];
			uint8_t ant_height = dg[5];

			uint16_t g_sep = (dg[6] * 16);

			bool dflag = dg[8] & 0x10 ? 1 : 0;
			bool iflag = dg[8] & 0x20 ? 1 : 0;

			uint8_t dage = ((dg[7] & 0xe0) >> 1) + (dg[8] & 0x0f);
			uint16_t did = ((dg[8] & 0xc0) << 2) + dg[9];

			retval = "fflag(";
			retval += fflag;
			retval += ") fix(";
			retval += fix;
			retval += ") hflag(";
			retval += hflag;
			retval += ") hdop(";
			retval += hdop;
			retval += ") nflag(";
			retval += nflag;
			retval += ") nsats(";
			retval += nsats;
			retval += ") qq(";
			retval += question;
			retval += ") ant(";
			retval += ant_height;
			retval += ") gsep(";
			retval += g_sep;
			retval += ") diff(";
			retval += dflag;
			retval += ",";
			retval += iflag;
			retval += ",";
			retval += dage;
			retval += ",";
			retval += did;
			retval += ")";
		}
		else if (dg[1] == 0x74)
		{
			*p_name = "SATS_USED";
			for (int i=0; i<5; i++)
			{
				uint8_t PU = dg[2+i];
				bool used = PU & 0x80;
				PU &= ~0x80;
				if (dg[2+i])
				{
					retval += used && PU ? " PU(" : " NA(";
					retval += PU;
					retval += ") ";
				}
			}
		}
		else
		{
			if (dg[1] == 0x0D)
				*p_name = "SAT_DETAIL1";
			else if (dg[1] == 0x0C)
				*p_name = "SAT_DETAIL2";
			else if (dg[1] == 0x2D)
				*p_name = "SAT_DETAIL3";
			else if (dg[1] == 0x8D)
				*p_name = "SAT_DETAIL4";
			else
			{
				if (st != ST_DIF_DETAIL)	// 0x1a7
					*p_name = "SAT_DETAIL";
				#if WITH_NEO6M
					if (!out && dg[0] == 0xa5 && dg[1]==0x4d && dg[15] == 0x08)
						replyToRestartGPSButton();
				#endif
			}

			uint8_t NN = dg[2];
			uint8_t AA = dg[3];
			uint8_t EE = dg[4];
			uint8_t SS = dg[5];
			uint8_t MM = dg[6];
			uint8_t BB = dg[7];
			uint8_t FF = dg[8];
			uint8_t GG = dg[9];
			uint8_t OO = dg[10];
			uint8_t CC = dg[11];
			uint8_t DD = dg[12];
			uint8_t XX = dg[13];
			uint8_t YY = dg[14];
			// uint8_t ZZ = dg[15]; unused by decoder

			uint8_t prn0 = (st == ST_DIF_DETAIL) ?
				dg[2] : 			// apparently the whole byte is the prn for special sbas/diff message
				(NN & 0xfe) / 2;	// otherwise, note the /2 missing from knaufs doc

			// 5229   <-> S12_SAT_DETAIL1 a5 0d 2c 43 77 52 2b 5d 7d 11 6d 4e 0f 27 56 02
			// 5230   <-> S12_SAT_DETAIL2 a5 0c 0d 01 49 00 28 0b 4f 03 40 85 91 8e 95
			// 5231   <-> S12_SAT_DETAIL3 a5 2d 22 1e 3c 32 0d 65 3e 14 69 98 0c 0d 00 02
			// 5233   <-> S12_SAT_DETAIL4 a5 8d 1a 65 0a 42 0d 00 00 00 00 00 00 00 00 00
			// 5232   <-> S12_SATS_USED   a5 74 96 00 00 00 80              ID(150) ID(128)
			// 5234   <-> S12_SATS_DONE   a5 98 00 00 00 00 00 00 00 00 00

			uint16_t az0 = AA*2 + (EE & 0x01);
			uint8_t ele0 = (EE & 0xfe)/2;
			uint8_t snr0 = (SS & 0xfe)/2;

			uint8_t prn1 = (MM & 0x70)/2 + (BB & 0x07);
			uint16_t az1 = (BB & 0xf8)*2 + (FF & 0x0f);
			uint8_t ele1 = (FF & 0xf0)/2 + (GG & 0x07);
			uint8_t snr1 = (GG & 0x80)/2 + (OO & 0x3f);

			uint8_t prn2 = (CC & 0x3f);
			uint16_t az2 = (CC & 0xc0)*2 + (DD & 0x7f);
			uint8_t ele2 = (XX & 0x7f);
			uint8_t snr2 = YY >> 1;

			char buf[255];
			if (st == ST_DIF_DETAIL)	// 0x1a7)
				sprintf(buf,"prn/az/ele/snr sat0(%d,%d,%d,%d) DIFF",
					prn0,az0,ele0,snr0);
			else if (dg[1] == 0x0c)
				sprintf(buf,"prn/az/ele/snr sat0(%d,%d,%d,%d) sat1(%d,%d,%d,%d)",
					prn0,az0,ele0,snr0,
					prn1,az1,ele1,snr1);
			else
				sprintf(buf,"prn/az/ele/snr sat0(%d,%d,%d,%d) sat1(%d,%d,%d,%d) sat2(%d,%d,%d,%d)",
					prn0,az0,ele0,snr0,
					prn1,az1,ele1,snr1,
					prn2,az2,ele2,snr2);
			retval = buf;

			// add the PUs for A5 0C

			if (dg[1] == 0x0c)
			{
				for (int i=0; i<4; i++)
				{
					uint8_t PU = dg[11+i];
					bool used = PU & 0x80;
					PU &= ~0x80;
					if (dg[11+i])
					{
						retval += used && PU ? " PU(" : " NA(";
						retval += PU;
						retval += ") ";
					}
				}
			}
		}
	}

	return retval;
}




void showDatagram16(bool port2, bool out, const uint16_t *dg)
{
	int len = (dg[1] & 0xf) + 3;
	uint8_t echo_dg[MAX_ST_BUF];
	for (int i=0; i<len; i++)
	{
		echo_dg[i] = dg[i];
	}
	showDatagram(port2,true,echo_dg);
}


void showDatagram(bool port2, bool out, const uint8_t *dg)
{
	uint16_t st = dg[0] | 0x100;
	int port_num = port2 ? PORT_ST2 : PORT_ST1;

	bool mon = inst_sim.g_MON[port_num];

	int binary_type = port2 ? BINARY_TYPE_ST2 : BINARY_TYPE_ST1;
	bool bin = g_BINARY & binary_type;

	bool fwd = 0;
	if (!out && (
		(port2 && (inst_sim.g_FWD & FWD_ST2_TO_1)) ||
		(!port2 && (inst_sim.g_FWD & FWD_ST1_TO_2)) ))
	{
		fwd = 1;
	}



	static int in_counter = 0;
	in_counter++;

	const char *name = NULL;
	const st_info_type *found = 0;

	if (mon || bin)
	{
		#define WIDTH_OF_HEX	3
		#define PAD_HEX         (20 * WIDTH_OF_HEX)	// for monitor only

		const st_info_type *search = st_known;
		while (!found && search->st)
		{
			if (search->st == st) found = search;
			search++;
		}
		if (found) name = found->name;
	}

	// decode needed for listeners

	String decode = decodeST(out,st,dg,&name);
	if (name == NULL) name = "unknown";

	if (mon || bin)
	{
		String st_name(fwd ? "S" : "ST");
		st_name += port2 ? '2' : '1';			// STx for sends and non-forwarded receives
		if (fwd)								// Sxy for forwarded receives
			st_name += port2 ? '1' : '2';

		st_name += '_';
		st_name += name;
		if (!found)
		{
			char buf[10];
			sprintf(buf,"(%02x)",dg[0] & 0xff);
			st_name += buf;
		}

		// fill out the hex buf

		String hex;
		int len = (dg[1] & 0xf) + 3;
		char hex_buf[WIDTH_OF_HEX + 1];
		for (int i=0; i<len && i<MAX_ST_BUF; i++)
		{
			uint8_t byte = dg[i];
			sprintf(hex_buf,"%02x ",byte);
			hex += hex_buf;
		}

		String arrow(fwd ? "<->" : out ? "-->" : "<--");

		if (mon)
		{
			String out = pad(in_counter,7);
			out += arrow;
			out += " ";
			out += pad(st_name,MAX_ST_NAME+3);
			out += " ";
			out += pad(hex,PAD_HEX);
			out += " ";
			out += decode;
			Serial.println(out.c_str());
			#if WITH_TB_ESP32
				if (inst_sim.doTbEsp32())
					SERIAL_ESP32.println(out.c_str());
			#endif
		}

		if (bin)
		{
			#define MSG_BUF_SIZE 256
			String bin =
				arrow + "\t" +
				st_name + "\t" +
				hex + "\t" +
				decode;
			/*static*/ uint8_t binary_buf[BINARY_HEADER_LEN + MSG_BUF_SIZE];
			int offset = startBinary(binary_buf,binary_type);
			offset = binaryVarStr(binary_buf, offset, bin.c_str(), MSG_BUF_SIZE);
			endBinary(binary_buf,offset);
			Serial.write(binary_buf,offset);
			#if WITH_TB_ESP32
				if (inst_sim.doTbEsp32())
					SERIAL_ESP32.write(binary_buf,offset);
			#endif
		}
	}
	

	if (fwd)
	{
		queueDatagram8(!port2,dg, true);
	}
}


// end of instST_in.cpp
