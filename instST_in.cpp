//---------------------------------------------
// instST_in.cpp
//---------------------------------------------
// Show (decode) datagrams.

#include "instST.h"
#include <myDebug.h>

bool g_MON_ST;
uint32_t g_last_st_receive_time;


// these three happen when the unit is idle
//
//		#define ST_UNKNOWN01	0x159		// in 		59 11 ce ff
//		#define ST_COMPASS_VAR	0x199		// in 		99  00  XX
//		#define ST_E80_SIG		0x161		// in 		61 03 03 00 00 00
//
//
//		#define ST_WPT_NAV_INFO 0x185		// in 		85 06 00 08 00 00 10 00 ef
//
//		// some kind of WAYPOINT info, 9e 0d 00 00 02 00 00 00 00 00 00 00 00 00 00 00
//
//		// no cigar #define ST_DST_WPT_INFO 0x1A1		// in 		a1 9d 30 30 30 30 30 30 47 4f 54 4f 20 57 41 59
//		//                                                             XD 49 49
//		// 000000GOTO WAY
//
//		#define ST_INFORMATIONAL 0x1a1		// my guess, a general purpose text message about waypoints
//			// 0x0000: a1 9d 30 30 30 30 30 30 47 4f 54 4f 20 57 41 59     ..000000GOTO WAY
//			// 0x0000: a1 bd 30 30 30 30 30 30 50 4f 49 4e 54 00 00 00 	   ..000000POINT...




int datagram_counter = 0;

void showDatagram(const uint8_t *datagram)
{
	#define WIDTH_OF_HEX	3
	#define MAX_DISPLAY 	16
	#define MAX_LEFT		MAX_DISPLAY * WIDTH_OF_HEX

	int len = (datagram[1] & 0xf) + 3;

	static char buf_left[MAX_LEFT + 1];
	static char buf_right[MAX_DISPLAY + 1];
	memset(buf_left,0,MAX_LEFT + 1);
	memset(buf_right,0,MAX_DISPLAY + 1);

	for (int i=0; i<len && i<MAX_DISPLAY; i++)
	{
		uint8_t byte = datagram[i];
		char *pr = &buf_right[i];
		char *pl = &buf_left[i * WIDTH_OF_HEX];
		sprintf(pl,"%02x ",byte);
		sprintf(pr,"%c",byte > 32 ? byte : '.');
	}

	// display_bytes_long(0,0x0000,datagram,len);

	static int in_counter = 0;
	in_counter++;

	static char format[32];
	static char obuf[MAX_LEFT + MAX_DISPLAY + 2 + 12];
	sprintf(format,"%%-4d <-- %%-%ds %%s",MAX_LEFT);
	sprintf(obuf,format,in_counter++,buf_left,buf_right);
	Serial.println(obuf);
}


// end of decode.cpp
