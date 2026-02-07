//--------------------------------------
// inst2000.h
//--------------------------------------
// contains defines and externs specific to NMEA2000.
// Note that although I have only built this for teensy, I
// 		included defines and types for old ESP32 and mcp2515
// 		CANBUS implementations.

#pragma once

#define USE_NMEA2000_TEENSY		1	// use onboard teensy canbus peripheral
#define USE_NMEA2000_ESP		2	// use onboard ESP32 canbus peripheral
#define USE_NMEA2000_MCP		3	// use MCP2515 mcp module

#define HOW_NMEA2000			USE_NMEA2000_TEENSY

#if HOW_NMEA2000 == USE_NMEA2000_TEENSY
	#include <NMEA2000_Teensyx.h>
	#define NMEA2000_CLASS 	tNMEA2000_Teensyx
#elif HOW_NMEA2000 == USE_NMEA2000_ESP
	#include <NMEA2000_esp32.h>
	#define NMEA2000_CLASS	tNMEA2000_esp32
	#define CAN_TX_PIN 		GPIO_NUM_17
	#define CAN_RX_PIN		GPIO_NUM_16
#elif HOW_NMEA2000 == USE_NMEA2000_MCP
	// Note that the MCP2515 monitor needs DEBUG_RXANY compile flag
	// in setup_platform.pm for the the CANBUS_Shield library
	#include <NMEA2000_mcp.h>
	#define NMEA2000_CLASS	tNMEA2000_mcp
	#define CAN_CS_PIN		5
		// I use this instead of the default ESP32 CS pin15 because
		// the default ESP32 CS pin15 is soldered on the ideaspark
		// board to the ST7789 oled display, and because this is
		// what I use in the sensor
	#define INT_PIN			GPIO_NUM_22		// 0xff == none
		// It works better with the interrupt pin !
		// Was losing packets in the monitor, then this fixed it
	#define USE_HSPI		0
		// I used to use the ESP32 alternative HSPI for the
		// mcp2515 so that it isn't mucked with by the st7789 display
		// which is soldered to the default ESP32 VSPI pins.
#else
	error NO NMEA2000_CLASS SPECIFIED
#endif

#include <N2kDeviceList.h>


//-------------------------------
// defines
//-------------------------------

#define TEENSYBOAT_NMEA_ADDRESS		25		// uses product definition in init() method
#define TEENSYGPS_NMEA_ADDRESS		30		// otherwise you must set product definition before init


#define PGN_ACK						59392L 
#define PGN_REQUEST					59904L
#define PGN_ADDRESS_CLAIM			60928L
#define PGN_PGN_LIST				126464L
#define PGN_HEARTBEAT				126993L
#define PGN_PRODUCT_INFO			126996L
#define PGN_DEVICE_CONFIG			126998L

// D = decoded, x=sent invariantly, 0/1 sent with ifdef


#define PGN_SYSTEM_DATE_TIME		126992L		//		1 = gpsInst
#define PGN_VESSEL_HEADING			127250L		// D 	x = compassInst
#define PGN_HEADING_TRACK_CONTROL	127237L		// 		x = apInst
#define PGN_ENGINE_RAPID 			127488L		// D	x = engInst
#define PGN_ENGINE_DYNAMIC 			127489L		// D	x = engInst
#define PGN_AC_INPUT_STATUS			127503L		// 		1 = genInst
#define PGN_AC_OUTPUT_STATUS		127504L		// 		1 = genInst
#define PGN_FLUID_LEVEL 			127505L		// D	x+x = engInst
#define PGN_AGS_CONFIG_STATUS		127512L		//
#define PGN_AGS_STATUS				127514L		//
#define PGN_SPEED_WATER_REF			128259L		// D	1 = logInst
#define PGN_WATER_DEPTH				128267L		// D	x = depthInst
#define PGN_DISTANCE_LOG			128275L		// D	1 = logInst
#define PGN_POSITION_RAPID_UPDATE	129025L		// D
#define PGN_COG_SOG_RAPID_UPDATE	129026L		//
#define PGN_GNSS_POSITION_DATA		129029L		// D	x = gpsInst
#define PGN_LOCAL_TIME_OFFSET		129033L		//
#define PGN_AIS_CLASS_B_POSITION	129039L		// D
#define PGN_DATUM					129044L		//
#define PGN_CROSS_TRACK_ERROR		129283L		// D	x = apInst(routing)
#define PGN_NAVIGATION_DATA			129284L		// 		x = apInst(routing)
#define PGN_ROUTE_WP_INFO			129285L		// 		0 = apInst(routing)
#define PGN_SET_AND_DRIFT			129291L		//
#define PGN_GNSS_SATS_IN_VIEW		129540L		// D	1 = gpsInst
#define PGN_AIS_STATIC_B_PART_A		129809L		// D
#define PGN_AIS_STATIC_B_PART_B		129810L		// D
#define PGN_WIND_DATA				130306L		// D	x+0 = wind instrument
#define PGN_ENV_PARAMETERS			130310L		//
#define PGN_TEMPERATURE    			130316L		// D
#define PGN_DIRECTION_DATA			130577L		// D	1 = gpsInst


// Raymarine/Proprietary
// found on https://github.com/canboat/canboat/blob/master/analyzer/pgn.h

#define PGN_SEATALK_ROUTE_INFO		130918L		// 		1 = apInst(routine)

#define PGN_GEN_PHASE_A_AC_POWER	65026L      // 1 = genInst
#define PGN_GEN_PHASE_A_BASIC_AC	65027L      // 1 = genInst
#define PGN_TOTAL_AC_POWER			65029L      // 1 = genInst
#define PGN_AVERAGE_AC_QUANTITIES	65030L      // 1 = genInst
#define PGN_SEATALK_GEN_INFO		65288L      // 1 = genInst

// known proprietary messages from E80

#define PGN_PROP_B_65311			65311L		// proprietary sent by E80
#define PGN_PROP_B_65362			65362L		// proprietary sent by E80
#define PGN_PROP_B_65364			65364L		// proprietary sent by E80
	// Raymarine proprietary “fast wind” mirror of 130306.
#define PGN_PROP_B_129044			129044L		// proprietary sent by E80
	// Raymarine proprietary navigation state/cross track error companion
	// to 129283 (XTE), 129284 (Navigation Data), and 129285 (Route/Waypoint Data).
#define PGN_PROP_B_130846			130846L		// proprietary sent by E80




class inst2000 : public NMEA2000_CLASS
	// statically constructed, so by default, all members are 0
{
public:

	inst2000() :
#if HOW_NMEA2000 == USE_NMEA2000_TEENSY
		NMEA2000_CLASS()
#elif HOW_NMEA2000 == USE_NMEA2000_ESP
		NMEA2000_CLASS(CAN_TX_PIN, CAN_RX_PIN)
#elif HOW_NMEA2000 == USE_NMEA2000_MCP
		NMEA2000_CLASS(CAN_CS_PIN,MCP_8MHz,INT_PIN,MCP_CAN_RX_BUFFER_SIZE)
#endif
	{}

	void init(uint8_t source_address);
		// passing source_address==0 will cause the init method
		// 		to use the pre-defined product settings for teensyBoat
		// 		and actual m_source_address=25.
		// passing source_address!=0 requies that you define the
		//		product settings before calling init() and those
		//		product settings and the source address you pass in
		//		will be used
	void loop();

	void broadcastNMEA2000Info();
	void sendDeviceQuery();
	void listDevices();

#ifdef ESP32
	static void showESP32Memory();
#endif


protected:

	tN2kDeviceList *m_device_list;
	uint8_t m_source_address;

	static void onBusMessage(const tN2kMsg &msg);
		// in inst2000_in.cpp
		
	void addSelfToDeviceList();

}; 	// class myNM


extern inst2000 nmea2000;






// end of inst2000.h
