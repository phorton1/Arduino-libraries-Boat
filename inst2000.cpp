//-----------------------------------------------------
// inst2000.cpp
//-----------------------------------------------------

#include "inst2000.h"
#include "instSimulator.h"
#include <myDebug.h>
#include <N2kMessages.h>
#include <N2kMessagesEnumToStr.h>

#define dbg_mon			0		// general debugging
#define dbg_command 	0		// commands
#define dbg_dl  		0		// device list



inst2000 nmea2000;
	// global static instance


#define BUS_COLOR "\033[37m"
	// 37 = WHITE

#if USE_NMEA2000_MCP
	#include <SPI.h>
	#if USE_HSPI
		SPIClass *hspi;
			// MOSI=13
			// MISO=12
			// SCLK=14
			// default CS = 15, we use 5
	#endif
#endif


static const unsigned long TransmitMessages[] = {
	// these system PGNs may not be necessary here,
	// but it is more conformal to include them.
#if 1
	PGN_REQUEST,
	PGN_ADDRESS_CLAIM,
	PGN_PGN_LIST,
	PGN_HEARTBEAT,
	PGN_PRODUCT_INFO,
	PGN_DEVICE_CONFIG,
#endif

	PGN_SYSTEM_DATE_TIME,
	PGN_VESSEL_HEADING,
	PGN_HEADING_TRACK_CONTROL,
	PGN_ENGINE_RAPID,
	PGN_ENGINE_DYNAMIC,
	PGN_AC_INPUT_STATUS,
    PGN_AC_OUTPUT_STATUS,
	PGN_FLUID_LEVEL,
	PGN_AGS_CONFIG_STATUS,
	//PGN_AGS_STATUS,
	PGN_SPEED_WATER_REF,
	PGN_WATER_DEPTH,
	PGN_DISTANCE_LOG,
	//PGN_POSITION_RAPID_UPDATE,
	//PGN_COG_SOG_RAPID_UPDATE,
	PGN_GNSS_POSITION_DATA,
	//PGN_LOCAL_TIME_OFFSET,
	//PGN_AIS_CLASS_B_POSITION,
	//PGN_DATUM,
	PGN_CROSS_TRACK_ERROR,
	PGN_NAVIGATION_DATA,
	//PGN_ROUTE_WP_INFO,
	//PGN_SET_AND_DRIFT,
	//PGN_GNSS_SATS_IN_VIEW,
	//PGN_AIS_STATIC_B_PART_A,
	//PGN_AIS_STATIC_B_PART_B.
	PGN_WIND_DATA,
	//PGN_ENV_PARAMETERS,
	//PGN_TEMPERATURE,
	PGN_DIRECTION_DATA,
	PGN_SEATALK_ROUTE_INFO,

	PGN_SEATALK_ROUTE_INFO,
	
	PGN_GEN_PHASE_A_AC_POWER,
	PGN_GEN_PHASE_A_BASIC_AC,
	PGN_TOTAL_AC_POWER,
	PGN_AVERAGE_AC_QUANTITIES,
	PGN_SEATALK_GEN_INFO,

	0
};




void inst2000::init(uint8_t source_address)
	// source_address passed as zero => m_source_address=25 and below product definition used
	// otherwise m_source_address=source_address and you must set product def before calling this
{
	display(dbg_mon,"inst2000::init(%d) started",source_address);
	proc_entry();

	//--------------------------------------
	// nmea setup
	//--------------------------------------
	// for device class and functions see
	//		docs/ref/nmea_2000/20120726_nmea_2000_class_&_function_codes_v_2.00-1 obtained from the wayback machine at:
	//		https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
	// for registration/company id's see
	//		docs/ref/nmea2000/20121020_nmea_2000_registration_list.pdf obtained from wayback machine at
	//		https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
	// I am not currently calling SetDeviceInstance() but it's working "ok"

	m_source_address = source_address;
	if (source_address == TEENSYBOAT_NMEA_ADDRESS)
	{
		SetProductInformation(
			"teensyBoat1",            		// Manufacturer's Model serial code
			1000,                        	// Manufacturer's uint8_t product code
			"teensyBoat multi-interface",   // Manufacturer's Model ID
			"tb_sw_1.0",             		// Manufacturer's Software version code
			"tb_v_1.0",             		// Manufacturer's uint8_t Model version
			3,                          	// LoadEquivalency uint8_t 3=150ma; Default=1. x * 50 mA
			2101,                       	// N2kVersion Default=2101
			1,                          	// CertificationLevel Default=1
			0                           	// iDev (int) index of the device on \ref Devices
			);
		SetConfigurationInformation(
			"prhSystems",      // ManufacturerInformation
			"tbInstall1",      // InstallationDescription1
			"tbInstall2"       // InstallationDescription2
			);
		SetDeviceInformation(
			123456,  // uint32_t Unique number, i.e. Serial number.
			130,     // uint8_t  Device function = Analog to NMEA 2000 Gateway
			25,      // uint8_t  Device class = Inter/Intranetwork Device
			2046     // uint16_t Registration/Company) ID // 2046 does not exist; choosen arbitrarily
			);
	}

	// set Device Mode and it's address(99)

	SetMode(tNMEA2000::N2km_ListenAndNode, m_source_address);
		// N2km_NodeOnly
		// N2km_ListenAndNode *
		// N2km_ListenAndSend **
		// N2km_ListenOnly
		// N2km_SendOnly

	// configure forwarding or disable it

	#if 1
		EnableForward(false);
	#else
		SetForwardType(tNMEA2000::fwdt_Text); 	// Show bus data in clear text
		SetForwardStream(&Serial);
		SetForwardOwnMessages(true);
	#endif


	#if 1
		// ExtendReceiveMessages(AllMessages);
		ExtendTransmitMessages(TransmitMessages);
	#endif

	SetMsgHandler(onBusMessage);

	#if USE_NMEA2000_MCP
	#if USE_HSPI
		hspi = new SPIClass(HSPI);
		SetSPI(hspi);
	#endif
	#endif

	// add the device list
	// and add self to it

	display(dbg_mon,"creating deviceList",0);
	m_device_list = new tN2kDeviceList(this);
	addSelfToDeviceList();

	//----------------------------
	// OPEN THE NMEA2000 bus
	//----------------------------

	bool ok = Open();
	if (!ok)
		my_error("NMEA2000::Open() failed",0);

	proc_leave();
	display(dbg_mon,"inst2000::init() finished",0);

}	// inst2000::init()




// virtual
bool inst2000::SendMsg(const tN2kMsg &msg, int deviceIndex /*=0*/)
{

    bool result = tNMEA2000::SendMsg(msg, deviceIndex);

    if (inst_sim.g_MON[PORT_2000] & MON2000_SELF)
		onBusMessage(msg);  // monitor outgoing

	return result;
}


//---------------------------------
// utilities
//---------------------------------

void inst2000::broadcastNMEA2000Info()
{
	#define NUM_INFOS			4
	#define MSG_SEND_INTERVAL	2000
	#define MSG_SEND_TIME		30000

	static int info_sent;
	static uint32_t last_send_time;

	uint32_t now = millis();
	if (info_sent >= NUM_INFOS && now - last_send_time > MSG_SEND_TIME)
	{
		info_sent = 0;
		last_send_time = now;
	}
	else if (info_sent < NUM_INFOS && now - last_send_time > MSG_SEND_INTERVAL)
	{
		last_send_time = now;
		ParseMessages(); // Keep parsing messages

		display(1,"Sending NMEA2000Info(%d)",info_sent);

		// at this time I have not figured out the actisense reader, and how to
		// get the whole system to work so that when it asks for device configuration(s)
		// and stuff, we send it stuff.  However, this code explicitly sends some info
		// at boot, and I have seen the results get to the reader!

		switch (info_sent)
		{
			case 0:
				SendProductInformation(
					255,	// unsigned char Destination,
					0,		// only device
					false);	// bool UseTP);
				break;
			case 1:
				SendConfigurationInformation(255,0,false);
				break;
			case 2:
				SendTxPGNList(255,0,false);
				break;
			case 3:
				SendRxPGNList(255,0,false);	// empty right now for the sensor
				break;
		}

		info_sent++;
		ParseMessages();
	}
}



void inst2000::sendDeviceQuery()
{
	display(dbg_command,"Sending PGN_REQUEST(PGN_PRODUCT_INFO) message",0);
	tN2kMsg msg;
	SetN2kPGN59904(msg, 255, PGN_PRODUCT_INFO);
	SendMsg(msg, 0);
}



//------------------------------------
// mcp2515 debugging
//------------------------------------
// low level canbus output debug display
// would need to be called from instSimulator::run()


#define DEBUG_MCP2515_OUT_LOW	0

#if DEBUG_MCP2515_OUT_LOW
	// overrides weakly linked prh_dbg_mcp2515_write() method
	// in #ifdef PRH_MODS in /libraries/CAN_BUS_Shield/mcp_can.cpp
	// to circular buffer the bytes written in the interrupt handler
	// and display() the in the main thread loop() method.
	// This was useful in the initial debugging of NMEA2000 to my Raymarine E80 MFD.

	#define MAX_CAN_MESSAGES	100

	typedef struct
	{
		byte len;
		byte first[4];
		byte buf[8];
	} dbg_msg_t;


	static volatile bool in_debug = 0;
	static int can_head = 0;
	static int can_tail = 0;
	static dbg_msg_t dbg_msgs[MAX_CAN_MESSAGES];


	void prh_dbg_mcp2515_write(byte *first, volatile const byte *buf, byte len)
	{
		int new_head = can_head  + 1;
		if (new_head >= MAX_CAN_MESSAGES)
			new_head = 0;
		if (new_head == can_tail)
		{
			my_error("DBG_CAN_MSG BUFFER OVERFLOW",0);
			return;
		}

		in_debug = 1;
		dbg_msg_t *msg = &dbg_msgs[can_head++];
		if (can_head >= MAX_CAN_MESSAGES)
			can_head = 0;

		msg->len = len;
		memcpy(msg->first,first,4);
		memcpy(msg->buf,(const void*) buf,len);
		in_debug = 0;
	}


	static void show_dbg_can_messages()
	{
		int head = can_head;
		if (in_debug)
			return;

		while (can_tail != head)
		{
			dbg_msg_t *msg = &dbg_msgs[can_tail++];
			if (can_tail >= MAX_CAN_MESSAGES)
				can_tail = 0;

			static char obuf[80];
			static int ocounter = 0;
			sprintf(obuf,"(%d) --> %02x%02x%02x%02x ",
				ocounter++,
				msg->first[0],
				msg->first[1],
				msg->first[2],
				msg->first[3]);

			int olen = strlen(obuf);
			for (int i=0; i<msg->len; i++)
			{
				sprintf(&obuf[olen],"%02x ",msg->buf[i]);
				olen += 3;
			}
			Serial.println(obuf);
			#if WITH_TB_ESP32
				if (inst_sim.doTbEsp32())
					SERIAL_ESP32.println(obuf);
			#endif
		}
	}
#endif





//-----------------------------------
// device_list
//-----------------------------------

static void displayIntList(const char *prefix, const unsigned long *list)
{
    uint8_t i;
    if (list)
    {
        String rslt(prefix);
        for (i=0; list[i]!=0; i++)
        {
            if (i>0) rslt += ", ";
            rslt += String(list[i]);
        }
        display(0,rslt.c_str(),0);
    }
}


static void displayDevice(const tNMEA2000::tDevice *pDevice)
{
    if (pDevice == 0) return;

    display(0,"----------------------------------------------------------------------",0);
    if (pDevice->GetModelID())
    display(0,"%s",pDevice->GetModelID());
    display(0,"    Source: %d",                     pDevice->GetSource());
    display(0,"    Manufacturer code:  %d",         pDevice->GetManufacturerCode());
    display(0,"    Unique number:      %d",         pDevice->GetUniqueNumber());
    if (pDevice->GetSwCode())
        display(0,"    Software version:   %s",     pDevice->GetSwCode());
    if (pDevice->GetModelVersion())
        display(0,"    Model version:      %s",     pDevice->GetModelVersion());
    if (pDevice->GetManufacturerInformation())
        display(0,"    Manufacturer Info:  %s",     pDevice->GetManufacturerInformation());
    if (pDevice->GetInstallationDescription1())
        display(0,"    Installation1:      %s",     pDevice->GetInstallationDescription1());
    if (pDevice->GetInstallationDescription2())
        display(0,"    Installation2:      %s",     pDevice->GetInstallationDescription2());
    displayIntList("    Transmit PGNs :",           pDevice->GetTransmitPGNs());
    displayIntList("    Receive PGNs  :",           pDevice->GetReceivePGNs());
    display(0,"",0);
}


void inst2000::listDevices()
{
    display(0,"******************************** DEVICE LIST **********************************",0);
    for (uint8_t i=0; i< N2kMaxBusDevices; i++)
    {
        displayDevice(m_device_list->FindDeviceBySource(i));
    }
}


//----------------------------------------
// addSelfToDeviceList()
//----------------------------------------
// Add self to the device list by creating the
// relevant messages and giving them to the
// device list to parse.
//
// It'd sure be nice if we could do this better.
// I don't want to use my NM.cpp constants, but
// rather GET the values from the nmea2000 object
// even though it's not Open yet.
//
// I *may* be able to take advantage of myNMEA2000_mcp
// derived class, and I *might* need to derive my own
// device list, but I haven't after trying to deal with
// the weird protected: schemes, I give up.  Protected
// members with public accessors.  Way obstruficated
// headers.  If it's an implementation variable, effing
// hide it in the cpp file.
//
// Finally, foe the 3rd case, it's a waste of my time trying
// to figure out how to pass my own PGN lists
// this way.  I'd have to extern NMEA2000.cpp implementation
// variables, and then use my own constant lists here
// because the author made everything protected, wheras
// it could have simply been const for safety.
//
// I almost would write my own device list, and then
// I start think of re-writing the whole library.
// It works.  I'll leave it as is.  You don't see
// any PGN's on my SELF object, and it sends everyone
// else on the bus, except me, 4 * 2 times, a request
// for my PGN lists.  Sheesh

void inst2000::addSelfToDeviceList()
{
    for (int i=0; i<3; i++)
    {
        tN2kMsg msg;
        display(dbg_dl,"addSelfToDeviceList(%d)",i);

        switch (i)
        {
            case 0 :
            {
                const tNMEA2000::tDeviceInformation info = GetDeviceInformation();
                SetN2kPGN60928(msg,
                    info.GetUniqueNumber(),
                    info.GetManufacturerCode(),
                    info.GetDeviceFunction(),
                    info.GetDeviceClass(),  // << 1?
                    info.GetDeviceInstance(),
                    info.GetSystemInstance(),
                    info.GetIndustryGroup() );
                msg.Source = m_source_address;
                m_device_list->HandleMsg(msg);
                break;
            }
            case 1 :
            {
                bool info_progmem;
                const tNMEA2000::tProductInformation *info = GetProductInformation(0,info_progmem);   //idev, b_progmem
                if (info)
                {
                    SetN2kPGN126996(msg,
                        info->N2kVersion,
                        info->ProductCode,
                        info->N2kModelID,
                        info->N2kSwCode,
                        info->N2kModelVersion,
                        info->N2kModelSerialCode,
                        info->CertificationLevel,
                        info->LoadEquivalency);
                    msg.Source = m_source_address;
                    m_device_list->HandleMsg(msg);
                }
                else
                    my_error(0,"COULD NOT addSelfToDeviceList(%d) PRODUCT INFO",i);
                break;
            }
            case 2 :
            {
                #define MAX_BUF  32
                char inst1[MAX_BUF+1];
                char inst2[MAX_BUF+1];
                char manuf[MAX_BUF+1];

                GetInstallationDescription1(inst1, MAX_BUF);
                GetInstallationDescription2(inst2, MAX_BUF);
                GetManufacturerInformation(manuf, MAX_BUF);

                SetN2kPGN126998(msg,inst1,inst2,manuf,false);
                    // bool UsePgm=false

                msg.Source = m_source_address;
                m_device_list->HandleMsg(msg);
                break;
            }
        }   // switch
    }   // for
}   // addSelfToDeviceList()




