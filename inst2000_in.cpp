//-------------------------------------------
// inst2000_in.cpp
//-------------------------------------------
// NMEA2000 monitoring, as well as initializion.

#include "inst2000.h"
#include "instSimulator.h"
#include <N2kMessages.h>
#include <myDebug.h>


tNMEA2000_Teensyx nmea2000;
	// (tCANDevice _bus=NMEA2000_TEENSYX_CAN_BUS, tPins _txPin=NMEA2000_TEENSYX_TX_PIN, tPins _rxPin=NMEA2000_TEENSYX_RX_PIN);


//----------------------------------------------
// PGNs known by this program
//----------------------------------------------


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

	PGN_VESSEL_HEADING,			// sent by compass instrument
	PGN_ENGINE_RAPID, 			// sent by engine instrument
	PGN_ENGINE_DYNAMIC, 		// sent by engine instrument
	PGN_FLUID_LEVEL, 			// sent by engine instrument
	PGN_SPEED_WATER_REF,		// sent by log instrument
	PGN_WATER_DEPTH,			// sent by depth instrument
	// PGN_DISTANCE_LOG,
	// PGN_POSITION_RAPID_UPDATE,
	// PGN_COG_SOG_RAPID_UPDATE,
	PGN_GNSS_POSITION_DATA,		// sent by gps instrument
	PGN_LOCAL_TIME_OFFSET,
	// PGN_DATUM,
	// PGN_CROSS_TRACK_ERROR,
	PGN_NAVIGATION_DATA,		// sent by autopilot instrument
	// PGN_SET_AND_DRIFT,
	// PGN_GNSS_SATS_IN_VIEW,
	PGN_WIND_DATA,				// sent by wind instrument
	// PGN_ENV_PARAMETERS,
	PGN_DIRECTION_DATA,			// sent by log instrument
	0
};


static void onNMEA2000Message(const tN2kMsg &msg)
{
	if (instruments.monitorActive(PORT_2000,MONITOR_BUS))
	{
		#define MAX_DBG_BUF	512
		static int bus_num = 0;
		static char bus_buf[MAX_DBG_BUF+1];
		sprintf(bus_buf,"BUS(%d) : pri:%d PGN:%lu src:%d dst:%d len:%d  data:",
			bus_num++,
			msg.Priority,
			msg.PGN,
			msg.Source,
			msg.Destination,
			msg.DataLen);
		int buf_len = strlen(bus_buf);
		for (int i=0; i<msg.DataLen && buf_len<MAX_DBG_BUF+3; i++)
		{
			sprintf(&bus_buf[buf_len],"%02x ",msg.Data[i]);
			buf_len += 3;
		}
		Serial.println(bus_buf);

		// also timestamp (ms since start [max 49days]) of the NMEA2000 message
		// unsigned long MsgTime;

		if (msg.PGN == PGN_REQUEST)
		{
			unsigned long requested_pgn;
			if (ParseN2kPGN59904(msg, requested_pgn))
				warning(0,"    PGN_REQUEST(%d)",requested_pgn);
		}
	}
}


//------------------------
// setup
//------------------------

void init2000()
{
	display(0,"init2000() started",0);
	proc_entry();

	#if 0
		nmea2000.SetN2kCANMsgBufSize(150);
		nmea2000.SetN2kCANSendFrameBufSize(150);
		nmea2000.SetN2kCANReceiveFrameBufSize(150);
	#endif

	#if 1

		nmea2000.SetProductInformation(
			"prh_model_115",            // Manufacturer's Model serial code
			110,                        // Manufacturer's uint8_t product code
			"teensyBoat",       		// Manufacturer's Model ID
			"prh_sw_115.0",             // Manufacturer's Software version code
			"prh_mv_115.0",             // Manufacturer's uint8_t Model version
			1,                          // LoadEquivalency uint8_t 3=150ma; Default=1. x * 50 mA
			2101,                       // N2kVersion Default=2101
			1,                          // CertificationLevel Default=1
			0                           // iDev (int) index of the device on \ref Devices
			);
		nmea2000.SetConfigurationInformation(
			"prhSystems",           	// ManufacturerInformation
			"teensyInstall1",       	// InstallationDescription1
			"teensyInstall2"       		// InstallationDescription2
			);

		// for device class see: https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
		// 	archived at NMEA_Monitor/docs/20120726 nmea 2000 class & function codes v 2.00-1.pdf.
		// for the registration/company id, I guess 2046 arbitrarily chose because its NOT in
		//	https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
		// 	archived at NMEA_Monitor/docs/20121020 nmea 2000 registration list.pdf

		nmea2000.SetDeviceInformation(
			#if 0		 // what I've been using up until now
				1250110, // uint32_t my arbitrary unique (serial) number
				130,     // uint8_t  Function(75,130)=Temperature
				75, 	 // uint16_t Class(75)=Sensor Communication Interface
			#elif 0		 // what I probably should have been using
				1250111, // uint32_t my arbitrary unique (serial) number
				170,     // uint8_t  Function(60,170)=Integrated Navigation
				60, 	 // uint16_t Class=Navigation.
			#else		 // an attempt to become a genset (from co-pilot)
				1250112, // uint32_t my arbitrary unique (serial) number
				130,     // uint8_t  Function(30,130)=Generator
				30, 	 // uint16_t Class(30)=Electrical Generation
			#endif
			2046     // uint16_t Registration/Company) ID
					 // 2046 does not exist
			);
	#endif

	// set its initial bus address to 22

	nmea2000.SetMode(tNMEA2000::N2km_ListenAndNode,	22);
		// N2km_NodeOnly
		// N2km_ListenAndNode
		// N2km_ListenAndSend
		// N2km_ListenOnly
		// N2km_SendOnly

	#if 0	// SHOW_BUS_MESSAGES
		nmea2000.SetForwardStream(&Serial);
		nmea2000.SetForwardType(tNMEA2000::fwdt_Text);
			// Show in clear text.
		nmea2000.SetForwardOwnMessages(false);
	#else
		nmea2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)
	#endif


	#if 1
		// I could not get this to eliminate need for DEBUG_RXANY
		// compiile flag in the Monitor
		//
		// I now believe that I should only send the messages
		// the sensor transmits here ...

		nmea2000.ExtendTransmitMessages(TransmitMessages);
			// nmea2000.ExtendReceiveMessages(AllMessages);
	#endif

	#if	1
		nmea2000.SetMsgHandler(onNMEA2000Message);
	#endif

	if (!nmea2000.Open())
		my_error("nmea2000.Open() failed",0);
	else
		display(0,"nmea2000.Open() succeeded",0);

	proc_leave();
	display(0,"init2000() finished",0);
}



// end of inst2000_in.cpp