//---------------------------------------------
// inst2000.cpp
//---------------------------------------------
// Implementation of NMEA2000 instruments.
// Here are the PGNS purportedly sent out by E80, along with
// whether I send them out from this code

// additionally, I believe it tells us it sends out "1308*" other proprietary
// messages (or has a bug) as it shows '1308' in the list of transmitted PGNs.

#include "instSimulator.h"
#include "boatSimulator.h"
#include <myDebug.h>
#include <N2kMessages.h>

#define PGN_ISO_ACK					59392L
#define PGN_ISO_REQUEST				59904L
#define PGN_ISO_ADDRESS_CLAIM		60928L
#define PGN_ISO_COMMANDED_ADDRESS	61184L
#define PGN_PROP_RAYMARINE_65288	65288L
#define PGN_PROP_RAYMARINE_65311	65311L
#define PGN_PROP_RAYMARINE_65361	65361L
#define PGN_PROP_RAYMARINE_65362	65362L
#define PGN_PROP_RAYMARINE_65364	65364L
#define PGN_COMMAND_GROUP_FUNCTION	126208L
#define PGN_TX_RX_PGN_LIST			126464L
#define PGN_CONFIGURATION_INFO		126720L
#define PGN_SYSTEM_TIME				126992L
#define PGN_PRODUCT_INFORMATION		126996L

#define PGN_VESSEL_HEADING			127250L		// sent by compass instrument
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

// ALSO NOTE that I have verified that the E80 LISTENS for the following
// even though they are not in the transmit list, AND this is the only
// way (not Seatalk, not NMEA0183) to get engine info to it

#define PGN_ENGINE_RAPID	127488L
#define PGN_ENGINE_DYNAMIC	127489L
#define PGN_FLUID_LEVEL		127505L


#define FEET_TO_METERS		0.3048
#define NM_TO_METERS		1852.0
#define GALLON_TO_LITRE		3.785
#define PSI_TO_PASCAL		6895.0


//--------------------------------------------------------
// instruments
//--------------------------------------------------------

void depthInst::send2000(tNMEA2000 *nmea2000)
{
	double meters = boat.getDepth() * FEET_TO_METERS;
	tN2kMsg msg;
	// PGN_WATER_DEPTH
	SetN2kPGN128267(msg, 255,		// msg, sid
		meters,						// depth in meters
		0.0);						// offset from keel
	nmea2000->SendMsg(msg);
}


void logInst::send2000(tNMEA2000 *nmea2000)
{
	tN2kMsg msg;

	// with just SPEED, we don't get a COG/SOG on the display
	// so we explicitly send both the COG/SOG and the HEADING and speed through water here

	#if 1
		// PGN_SPEED_WATER_REF
		SetN2kPGN128259(msg, 255,		// msg, sid
			KnotsToms(boat.getSOG()));	// meters per second
		nmea2000->SendMsg(msg);
	#endif

	// PGN_DIRECTION_DATA
	SetN2kPGN130577(msg, 			// msg
		N2kDD025_Estimated,			// tN2kDataMode
		N2khr_true,					// tN2kHeadingReference,
		255,						// sid,
		DegToRad(boat.getCOG()),	// COG in radians
		KnotsToms(boat.getSOG()),	// SOG in m/s
		DegToRad(boat.getCOG()),	// heading in radians
		KnotsToms(boat.getSOG()),	// speed through water in m/s
		0,							// Set
		0);							// Drift
	nmea2000->SendMsg(msg);
}


void windInst::send2000(tNMEA2000 *nmea2000)
{
	tN2kMsg msg;
	// PGN_WIND_DATA	- we are emulating the wind instrument, so we only send "Apparent" angle
	SetN2kPGN130306(msg, 255,
		KnotsToms(boat.apparentWindSpeed()),	// meters per second
		DegToRad(boat.apparentWindAngle()),		// radians
		N2kWind_Apparent);						// tN2kWindReference
	nmea2000->SendMsg(msg);
	#if 0
		SetN2kPGN130306(msg, 255,					// not sending out separate "True" angle
			KnotsToms(boat.getWindSpeed()),			// meters per second
			DegToRad(boat.getWindAngle()),			// radians
			N2kWind_True_North);					// tN2kWindReference
		nmea2000->SendMsg(msg);
	#endif
}


void compassInst::send2000(tNMEA2000 *nmea2000)
{
	tN2kMsg msg;
	// PGN_VESSEL_HEADING
	SetN2kPGN127250(msg, 255,		// msg, sid
		DegToRad(boat.getCOG()),	// heading is in radians
		0.0, 						// Deviation
		0.0, 						// Variation,
		N2khr_true );				// tN2kHeadingReference(0)
	nmea2000->SendMsg(msg);
}


void gpsInst::send2000(tNMEA2000 *nmea2000)
{
	tN2kMsg msg;
		// not using PGN_POSITION
		// SetN2kPGN129025(msg, boat.getLat(), boat.getLon());

	// PGN_GNSS_POSITION_DATA
	SetN2kPGN129029(msg, 255,	// msg, sid
		0, 					// uint16_t DaysSince1970,
		millis()/1000,		// double SecondsSinceMidnight,
		boat.getLat(),		// double Latitude,
		boat.getLon(),		// double Longitude,
		0,					// double Altitude,
        N2kGNSSt_GPS,		// tN2kGNSStype GNSStype,
		N2kGNSSm_GNSSfix,	// tN2kGNSSmethod GNSSmethod,
        5,					// unsigned char nSatellites,
		0,					// double HDOP,
		0,					// double PDOP=0,
		0,					// double GeoidalSeparation=0,
        0,					// unsigned char nReferenceStations=0,
		N2kGNSSt_GPS,		// tN2kGNSStype ReferenceStationType=N2kGNSSt_GPS,
		0,					// uint16_t ReferenceSationID=0,
        0 );				// double AgeOfCorrection=0
	nmea2000->SendMsg(msg);

	// Note: PGN_GNSS_POSITION is sufficient to spoof E80 into showing the vessel,
	// but we probably eventually will want to send PGN_GNSS_SATS_IN_VIEW (129540) calling
	// SetN2kPGN129540() and AppendN2kPGN129540() every so often to spoof the satellilte
	// display and/or the VHF
}



void autopilotInst::send2000(tNMEA2000 *nmea2000)
	// With NMEA2000 I have not been able to get the waypoint name to show up on the E80,
	// nor to get the arrival alarm to beep when I get to a waypoint.
{

	if (boat.getAutopilot())
	{
		uint32_t wp_num = boat.getWaypointNum();
		const waypoint_t *wp = boat.getWaypoint(wp_num);
		double dist = boat.distanceToWaypoint();
		bool arrived = boat.getArrived();

		tN2kMsg msg;
		// PGN_NAVIGATION_DATA,
		// Note that unlike NMEA0183, the waypoint name is not included and does not
			// show up on the E80 ...

		SetN2kPGN129284(msg, 255,		// msg, sid
			dist * NM_TO_METERS,		// double DistanceToWaypoint (undocumented: IN METERS!!)
			N2khr_true,					// tN2kHeadingReference BearingReference
			false, 						// bool PerpendicularCrossed
			arrived,					// bool ArrivalCircleEntered
			N2kdct_GreatCircle,			// tN2kDistanceCalculationType CalculationType
			0.0,						// double BearingOriginToDestinationWaypoint
			0.0,						// double ETATime	// The E80 calculates the time to the waypoint as hh:mm:ss
			0,							// int16_t ETADate
			DegToRad(boat.headingToWaypoint()),	 // double BearingPositionToDestinationWaypoint (I assume this is true radians)
			wp_num-1,					// uint32_t OriginWaypointNumber
			wp_num,						// uint32_t DestinationWaypointNumber,
			wp->lat,					// double DestinationLatitude,
			wp->lon,					// double DestinationLongitude,
			KnotsToms(boat.getCOG()));	// double WaypointClosingVelocity);
		nmea2000->SendMsg(msg);

		// no joy trying to get alarm to beep or E80 start "following::
		// or show the waypoint names (like I CAN do with NMEA0183)

		#if 0
			SetN2kPGN129283(
				msg,
				255,                      // SID (sequence ID, arbitrary or 0xff)
				N2kxtem_Estimated,   	  // tN2kXTEMode
				false,                    // NavigationTerminated
				5.0 );					  // XTE in meters
			nmea2000->SendMsg(msg);

			// try sending a route to the E80

			static bool one_time = true;
			if (one_time)
			{
				display(0,"adding route",0);

				one_time = false;
				SetN2kPGN129285(msg,
					0,				// uint16_t id of the first waypoint
					237,			// uint16_t id of the database Database,
					0,				// uint16_t id of the Route
					N2kdir_forward,	// tN2kNavigationDirection(0),
					"POPA",			// const char* RouteName,
					N2kDD002_No);	// tN2kGenericStatusPair SupplementaryData=N2kDD002_No);

				for (int i=0; i<boat.getNumWaypoints(); i++)
				{
					const waypoint_t *rte_pt = boat.getWaypoint(i);
					display(0,"adding wpt(%d,%s)",i,rte_pt->name);

					if (!AppendN2kPGN129285(msg,
											i,				// uint16_t WPID,
											rte_pt->name,	// const char* WPName,
											rte_pt->lat,	// double Latitude,
											rte_pt->lon))	// double Longitude
					{
						warning(0,"ran out of room for route at i=%d",i);
						i = boat.getNumWaypoints();
						break;
					}
				}
				nmea2000->SendMsg(msg);
			}	// if (one_time)

		#endif

	}	// autopilot engaged
}


void engineInst::send2000(tNMEA2000 *nmea2000)
{
	tN2kMsg msg;

	// PGN_ENGINE_RAPID

	SetN2kPGN127488(
			msg,
			0,					// EngineInstance
			boat.getRPMS(),		// EngineSpeed
			N2kDoubleNA,		// EngineBoostPressure
			N2kUInt8NA);		// EngineTiltTrim
	nmea2000->SendMsg(msg);

	// PGN_ENGINE_DYNAMIC

	static tN2kEngineDiscreteStatus1 status1;		// filled with zeros
	static tN2kEngineDiscreteStatus2 status2;		// filled with zeros

	SetN2kPGN127489(msg,
		0,											// EngineInstance
		boat.getOilPressure() * PSI_TO_PASCAL,		// EngineOilPress      in Pascal
		FToKelvin(boat.getOilTemp()),				// EngineOilTemp       in Kelvin
		FToKelvin(boat.getCoolantTemp()),			// EngineCoolantTemp   in Kelvin
		boat.getAltVoltage(),						// AltenatorVoltage    in Voltage
		boat.getFuelRate() * GALLON_TO_LITRE,		// FuelRate            in litres/hour
		N2kDoubleNA,								// EngineHours         in seconds
		N2kDoubleNA,								// EngineCoolantPress  in Pascal
		N2kDoubleNA,								// EngineFuelPress     in Pascal
		0,											// EngineLoad          in %
		0,											// EngineTorque        in %
		status1,									// Status1             Engine Discrete Status 1
		status2);									// Status2             Engine Discrete Status 2
	nmea2000->SendMsg(msg);

	// PGN_FLUID_LEVEL

	double capacity = 72 * GALLON_TO_LITRE;
	double level0 = boat.getFuelLevel(0);
	double level1 = boat.getFuelLevel(1);

	tN2kFluidType fluid_type = N2kft_Fuel;

	SetN2kPGN127505(msg, 0, fluid_type, level0, capacity);
	nmea2000->SendMsg(msg);

	SetN2kPGN127505(msg, 1, fluid_type, level1, capacity);
	nmea2000->SendMsg(msg);
}




void gensetInst::send2000(tNMEA2000 *nmea2000)
{
	tN2kMsg msg;

	// PGN 127504: AC Input Status
	msg.SetPGN(127504L);
	msg.Priority = 3;

	msg.AddByte(0xFF);               // SID
	msg.AddByte(0);                  // AC Instance
	msg.Add2ByteUDouble(120.0, 0.01); // Line Voltage (120V)
	msg.Add2ByteUDouble(60.0, 0.001); // Line Frequency (60Hz)
	msg.Add2ByteUDouble(0.0, 0.1);    // Current (optional, set to 0)
	msg.AddByte(0);                  // Reserved

	nmea2000->SendMsg(msg);

	return;


	// PGN_ENGINE_RAPID

	SetN2kPGN127488(
			msg,
			1,					// EngineInstance
			boat.getGenRPM(),	// EngineSpeed
			N2kDoubleNA,		// EngineBoostPressure
			N2kUInt8NA);		// EngineTiltTrim
	nmea2000->SendMsg(msg);


	// PGN_ENGINE_DYNAMIC

	#define PSI_TO_PASCAL		6895.0
	#define GALLON_TO_LITRE		3.785

	static tN2kEngineDiscreteStatus1 status1;		// filled with zeros
	static tN2kEngineDiscreteStatus2 status2;		// filled with zeros

	SetN2kPGN127489(msg,
		1,											// EngineInstance
		boat.getGenOilPressure() * PSI_TO_PASCAL,	// EngineOilPress      in Pascal
		FToKelvin(boat.getOilTemp()),								// EngineOilTemp       in Kelvin
		FToKelvin(boat.getGenCoolTemp()),			// EngineCoolantTemp   in Kelvin
		boat.getAltVoltage(),						// AltenatorVoltage    in Voltage
		boat.getFuelRate() * GALLON_TO_LITRE,		// FuelRate            in litres/hour
		N2kDoubleNA,								// EngineHours         in seconds
		N2kDoubleNA,								// EngineCoolantPress  in Pascal
		N2kDoubleNA,								// EngineFuelPress     in Pascal
		0,											// EngineLoad          in %
		0,											// EngineTorque        in %
		status1,									// Status1             Engine Discrete Status 1
		status2);									// Status2             Engine Discrete Status 2
	nmea2000->SendMsg(msg);

}

// end of inst2000.cpp
