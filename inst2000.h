//--------------------------------------
// inst2000.h
//--------------------------------------
// contains defines and externs specific to NMEA2000

#pragma once

#include <NMEA2000_Teensyx.h>

#define PGN_REQUEST					59904L
#define PGN_ADDRESS_CLAIM			60928L
#define PGN_PGN_LIST				126464L
#define PGN_HEARTBEAT				126993L
#define PGN_PRODUCT_INFO			126996L
#define PGN_DEVICE_CONFIG			126998L

#define PGN_VESSEL_HEADING			127250L		// sent by compass instrument
#define PGN_ENGINE_RAPID 			127488L		// sent by engine instrument
#define PGN_ENGINE_DYNAMIC 			127489L		// sent by engine instrument
#define PGN_FLUID_LEVEL 			127505L		// sent by engine instrument
#define PGN_SPEED_WATER_REF			128259L		// sent by log instrument
#define PGN_WATER_DEPTH				128267L		// sent by depth instrument
#define PGN_DISTANCE_LOG			128275L
#define PGN_POSITION_RAPID_UPDATE	129025L
#define PGN_COG_SOG_RAPID_UPDATE	129026L
#define PGN_GNSS_POSITION_DATA		129029L		// sent by gps instrument
#define PGN_LOCAL_TIME_OFFSET		129033L
#define PGN_DATUM					129044L
#define PGN_CROSS_TRACK_ERROR		129283L
#define PGN_NAVIGATION_DATA			129284L		// sent by autopilot instrument
#define PGN_SET_AND_DRIFT			129291L
#define PGN_GNSS_SATS_IN_VIEW		129540L
#define PGN_WIND_DATA				130306L		// sent by wind instrument
#define PGN_ENV_PARAMETERS			130310L
#define PGN_DIRECTION_DATA			130577L		// sent by log instrument


extern tNMEA2000_Teensyx nmea2000;
	// in boat2000.cpp
extern void init2000();


// end of inst2000.h
