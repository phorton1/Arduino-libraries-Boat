//---------------------------------------------
// instST_in.cpp
//---------------------------------------------
// Show (decode) datagrams.

#include "instST.h"
#include <myDebug.h>
#include "instSimulator.h"
#include "boatUtils.h"
#include "boatBinary.h"


// notes
//
// even with sim_0183, which is the only one to get the E80 to
//		display a target wp name, the E80 only gives out internal
//		numberic target wp names



uint32_t g_last_st_receive_time;

#define MAX_ST_NAME		12



typedef struct
{
	uint16_t	st;
	const char *name;
	int 		out_inst;
	int			standard;	// 0=opt_off, 1=opt_on; 2=standard
} 	st_info_type;

const st_info_type st_known[] =
{
	//                              name			out_inst
	/* 0x100 */ { ST_DEPTH,			"DEPTH",		INST_DEPTH,		2,	},
	/* 0x105 */ { ST_RPM,			"RPM",			INST_ENGINE,    2,	},
	/* 0x110 */ { ST_WIND_ANGLE,	"WIND_ANGLE",	INST_WIND,		2,	},
	/* 0x111 */ { ST_WIND_SPEED,	"WIND_SPEED",	INST_WIND,		2,	},
	/* 0x120 */ { ST_WATER_SPEED,	"WATER_SPEED",	INST_LOG,		2,	},
	/* 0x121 */	{ ST_TRIP,			"TRIP",			INST_LOG,		0,	},
	/* 0x121 */	{ ST_LOG_TOTAL,		"TOTAL",		INST_LOG,		0,	},
	/* 0x125 */	{ ST_TRIP_TOTAL,	"TRIP_TOTAL",	INST_LOG,		1,	},
	/* 0x126 */ { ST_LOG_SPEED,		"LOG_SPEED",	-1,				0,	},
	/* 0x150 */ { ST_LAT,			"LAT",			-1,				0,	},
	/* 0x151 */ { ST_LON,			"LON",			-1,				0,	},
	/* 0x152 */ { ST_SOG,			"SOG",			INST_GPS,		1,	},
	/* 0x153 */ { ST_COG,			"COG",			INST_GPS,		1,	},
	/* 0x154 */ { ST_TIME,			"TIME",			-1,				0,	},
	/* 0x156 */ { ST_DATE,			"DATE",			-1,				0,	},
	/* 0x157 */ { ST_SAT_INFO,		"SAT_INFO",		INST_GPS,		2,	},
	/* 0x158 */ { ST_LATLON,		"LATLON",		INST_GPS,		2,	},
	/* 0x159 */ { ST_59,			"59",			-1,				0,	},
	/* 0x161 */ { ST_E80_SIG,		"E80_SIG",		-1,				0,	},
	/* 0x182 */ { ST_TARGET_NAME,	"TARGET_NAME",	INST_AUTOPILOT,	1,	},
	/* 0x185 */ { ST_NAV_TO_WP,		"NAV_TO_WP",	INST_AUTOPILOT,	1,	},
	/* 0x189 */ { ST_HEADING,		"HEADING",		INST_COMPASS,	2,	},
	/* 0x199 */ { ST_COMPASS_VAR,	"COMPASS_VAR",	-1,				0,	},
	/* 0x1a2 */ { ST_ARRIVAL,		"ARRIVAL",		INST_AUTOPILOT,	1,	},
	/* 0x19E */	{ ST_WP_DEF,		"WP_DEF",		-1, 			0,	},
	/* 0z1A4 */	{ ST_DEV_QUERY,		"DEV_QUERY",	-1,				0,	},
	/* 0z1A4 */	{ ST_SAT_DETAIL,	"SAT_DETAIL",	INST_GPS,		0,	},
	/* 0x1A7 */	{ ST_A7,			"A7",			-1,				0,	},
	/* 0x1AD */	{ ST_AD,			"AD"			-1,				0,	},

	0,
};



static String decodeST(uint16_t st, const uint8_t *dg)
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
		uint32_t total = dg[2];
		total <<= 8;
		total |= dg[3];

		float f_total = total;
		f_total /= 10;
		char buf[20];
		sprintf(buf,"total(%0.2f)",f_total);
		retval = buf;
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
		retval = "lat(";
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
			uint8_t prec = dg[2];
			retval += " prec(";
			retval += prec;
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
	else if (st == ST_TARGET_NAME)	// 0x182
	{
		uint8_t XX = dg[2];
		uint8_t YY = dg[4];
		uint8_t ZZ = dg[6];
		uint8_t char1 = XX & 0x3f;
		uint8_t char2 = (YY & 0xf)*4 + (XX & 0xc0)/64;
		uint8_t char3 = (ZZ & 0x3)*16 + (YY & 0xf0)/16;
		uint8_t char4 = (ZZ & 0xfc)/4;
		retval = "name(";
		retval += (char) (char1 + 0x30);
		retval += (char) (char2 + 0x30);
		retval += (char) (char3 + 0x30);
		retval += (char) (char4 + 0x30);
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
	else if (st == ST_ARRIVAL)		// 0x1a2
	{
		// #define ST_ARRIVAL		0x1A2
		//	A2  X4  00  WW XX YY ZZ Arrival Info
		//					X&0x2=Arrival perpendicular passed, X&0x4=Arrival circle entered
		//					WW,XX,YY,ZZ = Ascii char's of waypoint id.   (0..9,A..Z)
		//									Takes the last 4 chars of name, assumes upper case only
		//					Corresponding NMEA sentences: APB, AA

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

	return retval;
}





void showDatagram(bool out_direction, const uint8_t *dg)
{
	if (!instruments.g_MON[PORT_ST] && !(g_BINARY & BINARY_TYPE_ST))
		return;

	#define WIDTH_OF_HEX	3
	#define PAD_HEX         (MAX_ST_SEEN * WIDTH_OF_HEX)

	static int in_counter = 0;
	in_counter++;

	uint16_t st = dg[0] | 0x100;
	String decode = decodeST(st,dg);

	const st_info_type *found = 0;
	const st_info_type *search = st_known;
	while (!found && search->st)
	{
		if (search->st == st) found = search;
		search++;
	}
	const char *name = found ?
		found->name : "unknown";
	const char *inst = found && found->out_inst>= 0?
		instruments.getInst(found->out_inst)->getName() : "";

	String st_name("ST_");
	st_name += name;
	String out_inst(inst);
	out_inst = out_inst.toLowerCase();

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

	String arrow(out_direction ? "-->" : "<--");



	if (instruments.g_MON[PORT_ST])
	{
		String out = pad(in_counter,7);
		out += arrow;
		out += " ";
		out += pad(st_name,MAX_ST_NAME+3);
		out += " ";
		out += pad(hex,PAD_HEX);
		out += pad(out_inst,MAX_INST_NAME);
		out += " ";
		out += decode;
		Serial.println(out.c_str());
	}

	if (g_BINARY & BINARY_TYPE_ST)
	{
		#define MSG_BUF_SIZE 256
		String bin =
			arrow + "\t" +
			st_name + "\t" +
			hex + "\t" +
			decode;
		/*static*/ uint8_t binary_buf[BINARY_HEADER_LEN + MSG_BUF_SIZE];
		int offset = startBinary(binary_buf,BINARY_TYPE_ST);
		offset = binaryVarStr(binary_buf, offset, bin.c_str(), MSG_BUF_SIZE);
		endBinary(binary_buf,offset);
		Serial.write(binary_buf,offset);
	}
}


// end of decode.cpp
