//---------------------------------------------
// decode0183.cpp
//---------------------------------------------

#include <myDebug.h>
#include <math.h>

bool g_MON_0183;
bool g_MON_AIS;


static int input_msg_num = 0;



#define DEBUG_VDM  0

#if DEBUG_VDM
	static const char *binary(int val)
	{
		static char buf[7];
		for (int i=0; i<6; i++)
		{
			buf[i] = (val & (1<<(5-i))) ? '1' : '0';
		}
		buf[6] = 0;
		return buf;
	}
#endif


static int decode_sixbit(char c)
{
	int val = c - 48;
	if (val > 39)
		val -= 8;
	#if DEBUG_VDM > 1
		display(0,"decode6(%c)=val(%d)= %s",c,val,binary(val));
	#endif
	return val;
}



// extern
void decode_vdm(const char *sentence)
{
	#if DEBUG_VDM
		display(0,"decode_vdm(%s)",sentence);
	#endif

	#define MAX_MSG 180
	int len = strlen(sentence);
	if (len >= MAX_MSG)
	{
		my_error("VDM message too long(%d)",len);
		return;
	}

	int i = 0;
	int num_fields = 1;
	char buf[MAX_MSG+1];
	strcpy(buf,sentence);
	const char *field[7];
	field[0] = buf;

	while (num_fields<7 && i<len)
	{
		if (buf[i] == ',')
		{
			buf[i++] = 0;
			if (num_fields < 7)
			   field[num_fields] = &buf[i];
			num_fields++;
		}
		else i++;
	}

	#if DEBUG_VDM
		for (i=0; i<num_fields; i++)
		{
			display(0,"   field[%d] = %s",i,field[i]);
		}
	#endif

	if (num_fields < 7)
	{
		my_error("not enough fields(%d)",num_fields);
		return;
	}

	#if DEBUG_VDM
		int frag_total = atoi(field[1]);
		int frag_num   = atoi(field[2]);
		int sid        = atoi(field[3]);
		const char *channel = field[4];
	#endif

	const char *payload = field[5];

	#if DEBUG_VDM
		int fill_bits = atoi(field[6]);
	#endif

	int payload_len = strlen(payload);

	#if DEBUG_VDM
		display(0,"   frag(%d/%d) sid(%d) channel(%s) fill_bits(%d)",frag_num,frag_total,sid,channel,fill_bits);
		display(0,"   payload(%d)=%s",payload_len,payload);
	#endif

	int bit_len = 0;
	uint8_t bits[1024];
	for (i = 0; i < payload_len; i++)
	{
		int val = decode_sixbit(payload[i]);
		for (int b = 5; b >= 0; b--)
		{
			bits[bit_len++] = (val >> b) & 1;
		}
	}

	#if DEBUG_VDM
		display(0,"   bit_len=%d",bit_len);
		Serial.print("First 36 bits=");
		for (i=0; i<36; i++)
			Serial.print(bits[i]);
		Serial.println();
	#endif

	int pos = 0;
	int msg_type = 0;
	for (i = 0; i < 6; i++)
	{
		msg_type = (msg_type << 1) | bits[pos++];
	}

	#if DEBUG_VDM
		display(0,"   msg_type=%d pos=%d",msg_type,pos);
	#endif

	if (msg_type == 18 && bit_len >= 168)
	{

		int repeat = 0;
		for (i = 0; i < 2; i++)
		    repeat = (repeat << 1) | bits[pos++];

		#if DEBUG_VDM
			Serial.print("MMSI bits = ");
			for (i = 0; i < 30; i++)
				Serial.print(bits[pos + i]);
			Serial.println();
		#endif

		uint32_t mmsi = 0;
		for (i = 0; i < 30; i++)
		{
			mmsi = (mmsi << 1) | bits[pos++];
		}

		#if DEBUG_VDM
			display(0,"   MMSI=%d",mmsi);
		#endif

		pos += 8;
		pos += 10;
		pos += 1;

		int32_t lon_raw = 0;
		for (i = 0; i < 28; i++)
		{
			lon_raw = (lon_raw << 1) | bits[pos++];
		}
		if (lon_raw & (1 << 27))
		{
			lon_raw -= (1 << 28);
		}
		double lon = lon_raw / 600000.0;

		int32_t lat_raw = 0;
		for (i = 0; i < 27; i++)
		{
			lat_raw = (lat_raw << 1) | bits[pos++];
		}
		if (lat_raw & (1 << 26))
		{
			lat_raw -= (1 << 27);
		}
		double lat = lat_raw / 600000.0;

		display(0,"   AIS type(%d) mmsi(%d) lat(%0.3f) lon(%0.3f)",
				msg_type,mmsi,lat,lon);

	}

	else if (msg_type == 24 && bit_len >= 160)
	{
		int repeat = 0;
		for (i = 0; i < 2; i++)
		    repeat = (repeat << 1) | bits[pos++];

		uint32_t mmsi = 0;
		for (i = 0; i < 30; i++)
		{
			mmsi = (mmsi << 1) | bits[pos++];
		}

		int part_num = (bits[pos++] << 1) | bits[pos++];
			// teensy compile gives warning: operation on 'pos' may be undefined 

		if (part_num == 0)
		{
			char vessel_name[21];
			for (i = 0; i < 120 && i / 6 < ((int)sizeof(vessel_name)) - 1; i += 6)
			{
				uint8_t val = 0;
				for (int b = 0; b < 6; b++)
				{
					val = (val << 1) | bits[pos + i + b];
				}
				char c = '@';
				if (val <= 31)
				{
					c = val + 64;
				}
				else if (val <= 63)
				{
					c = val;
				}

				vessel_name[i / 6] = (c == '@' /* prh || c == ' ' */) ? '\0' : c;
			}
			vessel_name[i / 6] = '\0';
			display(0,"   AIS type(%d) mmsi(%d) vessel_name(%s)",
					msg_type,mmsi,vessel_name);
		}
		else if (part_num == 1)
		{
			pos += 8;
			pos += 42;
		    char call_sign[8];

			for (i = 0; i < 42 && i / 6 < ((int)sizeof(call_sign)) - 1; i += 6)
			{
				uint8_t val = 0;
				for (int b = 0; b < 6; b++)
				{
					val = (val << 1) | bits[pos + i + b];
				}
				char c = '@';
				if (val <= 31)
				{
					c = val + 64;
				}
				else if (val <= 63)
				{
					c = val;
				}
				call_sign[i / 6] = (c == '@' || c == ' ') ? '\0' : c;
			}
			call_sign[i / 6] = '\0';
			display(0,"   AIS type(%d) mmsi(%d) call_sign(%s)",
					msg_type,mmsi,call_sign);
		}
	}
	else
		my_error("unknown AIS message type(%d)",msg_type);
}





void handleNMEA0183Input(const char *buf)
{
	input_msg_num++;
	if (g_MON_0183)
		display(0,"%-4d <-- %s",input_msg_num,buf);
	if (g_MON_AIS && strstr(buf,"VDM"))
		decode_vdm(buf);
}



// end of nmeaInput.cpp
