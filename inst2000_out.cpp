//---------------------------------------------
// inst2000_out.cpp
//---------------------------------------------
// Implementation of NMEA2000 simulated instruments.
// Here are the PGNS purportedly sent out by E80, along with
// whether I send them out from this code

// additionally, I believe it tells us it sends out "1308*" other proprietary
// messages (or has a bug) as it shows '1308' in the list of transmitted PGNs.

#include "inst2000.h"
#include "instSimulator.h"
#include "boatSimulator.h"
#include <N2kMessages.h>
#include <myDebug.h>
#include <cmath>


//--------------------------------------------------------
// instruments
//--------------------------------------------------------

void depthInst::send2000()
{
	double meters = boat.getDepth() * FEET_TO_METERS;
	tN2kMsg msg;
	// PGN_WATER_DEPTH
	SetN2kPGN128267(msg, 255,		// msg, sid
		meters,						// depth in meters
		0.0);						// offset from keel
	nmea2000.SendMsg(msg);
}


void logInst::send2000()
{
	tN2kMsg msg;

	// with just SPEED, we don't get a COG/SOG on the display
	// so we explicitly send both the COG/SOG and the HEADING and speed through water here

	#if 1
		// PGN_SPEED_WATER_REF
		SetN2kPGN128259(msg, 255,				// msg, sid
			KnotsToms(boat.getWaterSpeed()));	// meters per second
		nmea2000.SendMsg(msg);
	#endif

	#if 1
		// PGN_DISTANCE_LOG
		SetN2kPGN128275(msg,
			0,	// DaysSince1970
			0,	// SecondsSinceMidnight
			boat.getLogTotal() * NM_TO_METERS,
			boat.getTripDistance() * NM_TO_METERS);
		nmea2000.SendMsg(msg);

	#endif
}


void windInst::send2000()
{
	tN2kMsg msg;
	// PGN_WIND_DATA	- we are emulating the wind instrument, so we only send "Apparent" angle
	SetN2kPGN130306(msg, 255,
		KnotsToms(boat.apparentWindSpeed()),	// meters per second
		DegToRad(boat.apparentWindAngle()),		// radians
		N2kWind_Apparent);						// tN2kWindReference
	nmea2000.SendMsg(msg);
	#if 0
		SetN2kPGN130306(msg, 255,					// not sending out separate "True" angle
			KnotsToms(boat.getWindSpeed()),			// meters per second
			DegToRad(boat.getWindAngle()),			// radians
			N2kWind_True_North);					// tN2kWindReference
		nmea2000.SendMsg(msg);
	#endif
}


void compassInst::send2000()
{
	tN2kMsg msg;
	// PGN_VESSEL_HEADING
	SetN2kPGN127250(msg, 255,			// msg, sid
		DegToRad(boat.getHeading()),	// heading is in radians
		0.0, 							// Deviation
		0.0, 							// Variation,
		N2khr_true );					// tN2kHeadingReference(0)
	nmea2000.SendMsg(msg);
}


void gpsInst::send2000()
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
	nmea2000.SendMsg(msg);

	// Note: PGN_GNSS_POSITION is sufficient to spoof E80 into showing the vessel,
	// but we probably eventually will want to send PGN_GNSS_SATS_IN_VIEW (129540) calling
	// SetN2kPGN129540() and AppendN2kPGN129540() every so often to spoof the satellilte
	// display and/or the VHF

	#if 1
		// PGN_DIRECTION_DATA
		SetN2kPGN130577(msg, 			// msg
			N2kDD025_Estimated,			// tN2kDataMode
			N2khr_true,					// tN2kHeadingReference,
			255,						// sid,
			DegToRad(boat.getCOG()),		// COG in radians
			KnotsToms(boat.getSOG()),			// SOG in m/s
			DegToRad(boat.getHeading()),		// heading in radians
			KnotsToms(boat.getWaterSpeed()),	// speed through water in m/s
			KnotsToms(boat.getCurrentSet()),	// Set
			KnotsToms(boat.getCurrentDrift()));	// Drift
		nmea2000.SendMsg(msg);
	#endif

}



void aisInst::send2000()
{
}


void autopilotInst::send2000()
	// With NMEA2000 I have not been able to get the waypoint name to show up on the E80,
	// nor to get the arrival alarm to beep when I get to a waypoint.
{
	// PGN_HEADING_TRACK_CONTROL

	tN2kMsg msg;
	SetN2kPGN127237(
	  msg,
	  N2kOnOff_Unavailable,           // RudderLimitExceeded
	  N2kOnOff_Unavailable,           // OffHeadingLimitExceeded
	  N2kOnOff_Unavailable,           // OffTrackLimitExceeded
	  boat.getAutopilot() ? N2kOnOff_On : N2kOnOff_Off, // Override (used here to indicate autopilot state)
	  N2kSM_HeadingControl,             // SteeringMode
	  N2kTM_RudderLimitControlled,    // TurnMode (safe default)
	  N2khr_true,                 	  // HeadingReference
	  N2kRDO_NoDirectionOrder,        // CommandedRudderDirection
	  N2kDoubleNA,                    // CommandedRudderAngle
	  DegToRad(boat.getDesiredHeading()),		  // HeadingToSteerCourse
	  N2kDoubleNA,                    // Track
	  N2kDoubleNA,                    // RudderLimit
	  N2kDoubleNA,                    // OffHeadingLimit
	  N2kDoubleNA,                    // RadiusOfTurnOrder
	  N2kDoubleNA,                    // RateOfTurnOrder
	  N2kDoubleNA,                    // OffTrackLimit
	  N2kDoubleNA                     // VesselHeading
	);
	nmea2000.SendMsg(msg);

	static bool last_routing = false;
	static int last_route_id = 1;
	static int last_target = -1;
	static const char *last_route = "";

	bool routing = boat.getRouting();

	if (last_routing && !routing)
	{
		last_routing = routing;
		display(0, "inst2000 routing turned off",0);
		last_route_id = 1;
		last_target = -1;
		last_route = "";
	}
	else if (routing)
	{
		if (!last_routing)
			display(0, "Inst2000 routing turned on",0);
		last_routing = routing;

		int target_num = boat.getTargetWPNum();
		if (last_target != target_num || strcmp(last_route, boat.getRouteName()))
		{
			last_route_id++;
			last_target = target_num;
			last_route = boat.getRouteName();
			display(0, "Inst2000 Routing Sending Waypoints for Route(%s)", last_route);

			const uint16_t db_id = 237;
			const uint16_t route_id = last_route_id;
			const tN2kNavigationDirection direction = N2kdir_forward;
			const tN2kGenericStatusPair supplementary = N2kDD002_No;

			tN2kMsg msg;
			SetN2kPGN129285(msg, 0, db_id, route_id, direction, last_route, supplementary);
			for (int i = 0; i < boat.getNumWaypoints(); i++)
			{
				const waypoint_t *rte_pt = boat.getWaypoint(i);
				display(0, "   adding wpt(%d,%s)", i, rte_pt->name);
				if (!AppendN2kPGN129285(msg, i, rte_pt->name, rte_pt->lat, rte_pt->lon))
				{
					// Message full — send and start a new one
					// Start a new message with same route header
					nmea2000.SendMsg(msg);
					SetN2kPGN129285(msg, 0, db_id, route_id, direction, last_route, supplementary);
					if (!AppendN2kPGN129285(msg, i, rte_pt->name, rte_pt->lat, rte_pt->lon))
					{
						warning(0, "Waypoint too large to fit in empty PGN at i=%d", i);
						break;
					}
				}
			}

			// Send final message if it has any waypoints
			if (msg.DataLen > 0)
				nmea2000.SendMsg(msg);

		}	// sending route


		int start_num = boat.getStartWPNum();
		const waypoint_t *start_wp = boat.getWaypoint(start_num);
		const waypoint_t *target_wp = boat.getWaypoint(target_num);

		bool arrived = boat.getArrived();
		double wp_dist = boat.distanceToWaypoint();
		double wp_bearing = boat.headingToWaypoint();
		double rte_heading = boat.headingTo(start_wp->lat,start_wp->lon,target_wp);

		// PGN_NAVIGATION_DATA,
		// Note that unlike NMEA0183, the waypoint name is not included and does not
			// show up on the E80 ...

		time_t now = time(NULL);
		uint16_t eta_date = (now / 86400);  // Days since Jan 1, 1970


		SetN2kPGN129284(msg, 255,		// msg, sid
			wp_dist * NM_TO_METERS,		// double DistanceToWaypoint (undocumented: IN METERS!!)
			N2khr_true,					// tN2kHeadingReference BearingReference
			false, 						// bool PerpendicularCrossed
			arrived,					// bool ArrivalCircleEntered
			N2kdct_GreatCircle,			// tN2kDistanceCalculationType CalculationTyp
			N2kDoubleNA,				// double ETATime	// The E80 calculates the time to the waypoint as hh:mm:ss
			eta_date,					// made up NA variable; was 0, int16_t ETADate
			DegToRad(rte_heading),		// double BearingOriginToDestinationWaypoint
			DegToRad(wp_bearing),	 	// double BearingPositionToDestinationWaypoint (I assume this is true radians)
			start_num,					// uint32_t OriginWaypointNumber
			target_num,					// uint32_t DestinationWaypointNumber,
			target_wp->lat,				// double DestinationLatitude,
			target_wp->lon,				// double DestinationLongitude,
			N2kDoubleNA);				// KnotsToms(boat.getCOG()));	// double WaypointClosingVelocity);
		nmea2000.SendMsg(msg);

		// PGN_CROSS_TRACK_ERROR

		SetN2kPGN129283(
			msg,
			255,                      // SID (sequence ID, arbitrary or 0xff)
			N2kxtem_Estimated,   	  // tN2kXTEMode
			false,                    // NavigationTerminated
			boat.getCrossTrackError() * NM_TO_METERS);	// XTE in meters
		nmea2000.SendMsg(msg);


	}	// Routing
}	// instAutopilot



void engineInst::send2000()
{
	tN2kMsg msg;

	// PGN_ENGINE_RAPID

	SetN2kPGN127488(
			msg,
			0,							// EngineInstance
			boat.getRPM(),				// EngineSpeed
			boat.getBoostPressure()  * PSI_TO_PASCAL,	// EngineBoostPressure
			N2kUInt8NA);				// EngineTiltTrim
	nmea2000.SendMsg(msg);

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
		100 * boat.getRPM() / 7200,					// EngineLoad          in %
		0,											// EngineTorque        in %
		status1,									// Status1             Engine Discrete Status 1
		status2);									// Status2             Engine Discrete Status 2
	nmea2000.SendMsg(msg);

	tN2kFluidType fluid_type = N2kft_Fuel;

	// levels are *100 because apparently the E80
	// predates the N2K specificatiion and that's
	// how THEY implemented it
	
	SetN2kPGN127505(msg, 0, fluid_type,
		boat.getFuelLevel(0) * 100,
		boat.getTankCapacity(0) * GALLON_TO_LITRE);
	nmea2000.SendMsg(msg);

	SetN2kPGN127505(msg, 1, fluid_type,
		boat.getFuelLevel(1) * 100,
		boat.getTankCapacity(1) * GALLON_TO_LITRE);
	nmea2000.SendMsg(msg);
}




void gensetInst::send2000()
{
#if 0	// nothing worked

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
	nmea2000.SendMsg(msg);

	// PGN_ENGINE_RAPID

	SetN2kPGN127488(
			msg,
			1,					// EngineInstance
			boat.getGenRPM(),	// EngineSpeed
			N2kDoubleNA,		// EngineBoostPressure
			N2kUInt8NA);		// EngineTiltTrim
	nmea2000.SendMsg(msg);

	// PGN_ENGINE_DYNAMIC

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
	nmea2000.SendMsg(msg);

#endif	// nothing worked
}

// end of inst2000_out.cpp
