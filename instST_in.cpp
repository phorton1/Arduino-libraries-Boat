//---------------------------------------------
// instST_in.cpp
//---------------------------------------------
// Show (decode) datagrams.

#include "instST.h"
#include <myDebug.h>
#include "instSimulator.h"
#include "boatUtils.h"
#include "boatBinary.h"

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
	/* 0x182 */ { ST_TARGET_NAME,	"TARGET_NAME",	},
	/* 0x184 */	{ ST_AUTOPILOT,		"AUTOPILOT",	},
	/* 0x185 */ { ST_NAV_TO_WP,		"NAV_TO_WP",	},
	/* 0x186 */ { ST_AP_KEYSTROKE,	"AP_KEY",		},
	/* 0x189 */ { ST_HEADING,		"HEADING",		},
	/* 0x197 */	{ ST_ST7000,		"ST7000",		},
	/* 0x198 */ { ST_AP_CPU,		"AP_CPU",		},
	/* 0x199 */ { ST_COMPASS_VAR,	"COMPASS_VAR",	},
	/* 0x19c */ { ST_RUDDER,		"RUDDER",		},
	/* 0x19e */	{ ST_WP_DEF,		"WP_DEF",		},
	/* 0x1a2 */ { ST_ARRIVAL,		"ARRIVAL",		},
	/* 0z1a4 */	{ ST_DEV_QUERY,		"DEV_QUERY",	},
	/* 0z1a4 */	{ ST_SAT_DETAIL,	"SAT_DETAIL",	},
	/* 0x1a7 */	{ ST_A7,			"A7",			},
	/* 0x1ad */	{ ST_AD,			"AD"			},

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
	else if (st == ST_AUTOPILOT)	// 0x184
	{
		// PRH: see notes in instST_out.cpp regarding of the ST_AUTOPILOT
		//		datagram for what I perceive to be an inherent bug in knauf's
		//		description and his overloading of the high bit of the U nibble

		uint8_t U  = dg[1] >> 4;
		uint8_t VW = dg[2];
		uint8_t XY = dg[3];
		uint8_t Z  = dg[4];
		uint8_t M  = dg[5];
		uint8_t RR = dg[6];
		uint8_t SS = dg[7];
		uint8_t TT = dg[8];
		uint8_t V  = VW >> 4;

		uint16_t heading = (U & 0x3)*90;
		heading += (VW & 0x3F)*2;
		heading += (U >> 2) & 0x1;		// <-- prh: this is my interpretation
		heading %= 360;

		bool right = (U & 0x8) != 0;
		float course = (V >> 2)*90 + ((float)XY / 2.0f);
		int8_t rudder = (int8_t)RR;

		const char *mode="STBY";
		if (Z & 0x08) mode = "TRACK";
		else if (Z & 0x04) mode = "VANE";
		else if (Z & 0x02) mode = "AUTO";

		char course_buf[12];
		sprintf(course_buf,"%0.1f",course);

		// --- Build retval ---
		retval  = mode;
		retval += " head(";
		retval += heading;
		retval += ") rud(";
		retval += rudder;
		retval += ") right(";
		retval += (right ? 1 : 0);
		retval += ") ap(";
		retval += course_buf;
		retval += ") alarm(";
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
		//		Name			Ref			SHORT_BEFORE_LONG
		//		-------------------------------------------------------------
		// 		01 = Auto					yes
		// 		02 = Standby				no
		// 		03 = Track					no
		// 		04 = Display				no
		// 		05 = Minus1		-1			yes
		// 		06 = Minus10	-10			yes
		// 		07 = Plus1		+1			yes
		// 		08 = Plus10		+10			yes
		// 		09 = MinusResp	-Resp		yes
		// 		0a = PlusResp	+Resp		yes
		//
		// Combined Keys
		//
		//		KEY_NAME 				Ref			SHORT_B4_LONG	Notes
		//		------------------------------------------------------------------
		// 		20 = Minus1_Plus1		-1 & +1		yes
		//		21 = Minus1_Minus10		-1 & -10	yes
		//		22 = Plus1_Plus10		+1 & +10	yes
		// 		23 = Standby_Auto					yes
		//    	24 = Display_Track					no			longpress outputs 0x40 | 0x28 !!!
		// 		25 = Standby_Minus10				yes
		// 		28 = Minus10_Plus10		-10 & +10	no 			longpress outputs 0x40 | 0x24 !!!
		//		2e = MinsResp_PlusResp	-Resp&+Resp	long_only	"Rudder Gain Adjustment" for planing vessels
		// 		30 = Standby_Minus1					yes			not reliable; enters funny mode; crashes
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


































		//
		//		KEY_NAME 							OUTPUT MODES		Cannonical Function (not always)
		//		---------------------------------------------------------------------------------
		// 		20 = Minus1_Minus10		-1 & -10	AUTO/VANE			AutoTack_Port
		//		21 = Minus1_Plus1		-1 & +1
		//		22 = Plus1_Plus10		+1 & +10
		//
		// 		23 = Standby_Auto					AUTO/VANE			long = enter VANE Mode	or 	"return to previous auto mode/angle?"
		// 		30 = Standby_Minus1
		// 		25 = Standby_Minus10
		// 		28 = Plus1_Plus10		+1 & +10	AUTO/VANE			AutoTack_Stbd
		//    	24 = Display_Track					STANDBY				Enter Calibration Mode / Save Calibratiion settings


		//		These are only output in certain modes
		//		The ST7000 has "expectations" for certain of them and
		//			the ST7000 will sometimes output a 0x197 ST_ST7000 error/query
		//			message if those expectations are not met
		//		The overall behavior of the ST7000 is beyond the scope of this comment.


		display(dbg_st7000,"key(0x%02x) long(%d) single_key(0x%02x)",key,longpress,single_key);

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
		warning(0,"ST_ST7000 setting ap_linked=1",0);
		ap_linked = 1;
		retval = "ST7000";
	}
	else if (st == ST_AP_CPU)		// 0x198	undocumented by Knauf
	{
		// warning(0,"ST_AP CPU setting ap_linked=2",0);
		// ap_linked = 2;
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
		// 9C  U1  VW  RR	- see 0x184 above
		// note that I had done this using floats previously
		// in my original earliest ST_HEADING block above.
		//
		// PRH: see notes in instST_out.cpp regarding of the ST_AUTOPILOT
		//		datagram for what I perceive to be an inherent bug in knauf's
		//		description and his overloading of the high bit of the U nibble

		uint8_t U  = dg[1] >> 4;
		uint8_t VW = dg[2];
		uint8_t RR = dg[3];

		bool right = (U & 0x8) != 0;
		int8_t rudder = (int8_t) RR;

		uint16_t heading = (U & 0x3)*90;
		heading += (VW & 0x3F)*2;
		heading += (U >> 2) & 0x1;		// <-- prh: this is my interpretation
		heading %= 360;

		retval = "head(";
		retval += heading;
		retval += ") rud(";
		retval += rudder;
		retval += ") right(";
		retval += right;
		retval += ")";
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
	int port_num = port2 ? PORT_ST2 : PORT_ST1;
	int binary_type = port2 ? BINARY_TYPE_ST2 : BINARY_TYPE_ST1;
	if (!inst_sim.g_MON[port_num] && !(g_BINARY & binary_type))
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

	bool fwd = 0;

	if (!out && (
		(port2 && (inst_sim.g_FWD & FWD_ST2_TO_1)) ||
		(!port2 && (inst_sim.g_FWD & FWD_ST1_TO_2)) ))
	{
		fwd = 1;
	}


	const char *name = found ?
		found->name : "unknown";

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

	if (inst_sim.g_MON[port_num])
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
		#ifdef SERIAL_ESP32
			if (udp_enabled)
				SERIAL_ESP32.println(out.c_str());
		#endif
	}

	if (g_BINARY & binary_type)
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
		#ifdef SERIAL_ESP32
			if (udp_enabled)
				SERIAL_ESP32.write(binary_buf,offset);
		#endif
	}

	if (fwd)
	{
		queueDatagram8(!port2,dg, true);
	}
}


// end of decode.cpp
