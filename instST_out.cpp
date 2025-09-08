//---------------------------------------------
// instST_out.cpp
//---------------------------------------------
// Implementation of Seatalk instruments.

#include "instST.h"
#include "instSimulator.h"
#include "boatSimulator.h"
#include <myDebug.h>


#define SEND_OK			0
#define SEND_BUSY 		1
#define SEND_COLLISION	2
#define SEND_ERROR		3

#define IDLE_BUS_MS				10		// ms bus must be idle to send next datagram
#define SEND_DELAY				50		// ms between each datagram
#define NUM_RETRIES				6

uint32_t last_seatalk_receive_time;


static uint16_t dg[20];
static int retry_num = 0;


static int sendDatagram(const uint16_t *dg)
{
	if (millis() - last_seatalk_receive_time < IDLE_BUS_MS)
		return SEND_BUSY;	// bus busy

	#define WRITE_TIMEOUT	40

	//	if (show_output)
	//	{
	//		Serial.print("--> [");
	//		Serial.print(dg_num);
	//		Serial.print(",");
	//		Serial.print(retry_num);
	//		Serial.print("]");
	//	}

	int len = (dg[1] & 0xf) + 3;
	for (int i=0; i<len; i++)
	{
		bool ok = false;
		int out_byte = dg[i];

		//	if (show_output)
		//	{
		//		Serial.print(" 0x");
		//		Serial.print(out_byte,HEX);
		//	}

		uint32_t sent_time = millis();
		SERIAL_SEATALK.write9bit(out_byte);
		while (!ok)
		{
			if (SERIAL_SEATALK.available())
			{
				int in_byte = SERIAL_SEATALK.read();
				if (in_byte == out_byte)
				{
					ok = true;
				}
				else
				{
					// if (show_output)
					// 	Serial.println();
					warning(0,"collision(%d)",retry_num);
					return SEND_COLLISION;
				}
			}
			else if (millis() - sent_time >= WRITE_TIMEOUT)
			{
				// if (show_output)
				// 	Serial.println();
				my_error("WRITE_TIMEOUT",0);
				return SEND_ERROR;
			}
		}
	}

	// if (show_output)
	// 	Serial.println();
	return SEND_OK;
}




//-------------------------------------
// instruments
//-------------------------------------


void depthInst::sendSeatalk()
{
	memset(dg,0,20*2);


	uint16_t d10;
	if (boat.getDepth() > 999)
		d10 = 9990;
	else
		d10 = boat.getDepth() * 10.0;

	dg[0] = ST_DEPTH;
	dg[1] = 0x02;
	dg[2] = 0;	// flags
	dg[3] = d10 & 0xff;
	dg[4] = (d10 >> 8) & 0xff;

	sendDatagram(dg);
}

void logInst::sendSeatalk()
{
}

void windInst::sendSeatalk()
{
}

void compassInst::sendSeatalk()
{
}

void gpsInst::sendSeatalk()
{
}

void aisInst::sendSeatalk()
{
	my_error("AIS Instrument not supported on Seatalk",0);
}

void autopilotInst::sendSeatalk()
{
}

void engineInst::sendSeatalk()
{
	// not really supported
}

void gensetInst::sendSeatalk()
{
	// not supported
}


// end of instST_out.cpp
