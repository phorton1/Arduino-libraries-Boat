//-----------------------------
// boatUtils.cpp
//-----------------------------

#include "boatUtils.h"

String strDegreeMinutes(double coord)
{
	const char DEG_CHAR = 0xB0;

	double deg = round(coord);
	double mins = abs(coord - deg) * 60;
	char buf[20];
	sprintf(buf,"%d%c%0.3f",(int) deg, DEG_CHAR,mins);
	String rslt(buf);
	return rslt;
}


void pad(String &s, uint16_t len)
{
	while (s.length() < len)
	{
		s += " ";
	}
}
