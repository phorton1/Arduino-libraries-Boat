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


	#if 1
		// PGN_SYSTEM_DATE_TIME
		time_t now = time(NULL);
		uint32_t date = (now / SECONDS_PER_DAY);  // Days since Jan 1, 1970
		uint32_t secs = now - date * SECONDS_PER_DAY;
		SetN2kPGN126992(msg, 255, date, secs, N2ktimes_GPS);
		nmea2000.SendMsg(msg);
	#endif
}



void aisInst::send2000()
{
}



static void fillName(int fullbufsize, char *buf, const char *name)
{
	int bufsize = fullbufsize-1;
	int len = strlen(name);
	if (len > bufsize)
		len = bufsize;
	memset(buf,' ',fullbufsize);
	memcpy(buf,name,len);
	// buf[bufsize] = 0;
}


void autopilotInst::send2000()
	// With NMEA2000 I have not been able to get the waypoint name to show up on the E80,
	// nor to get the arrival alarm to beep when I get to a waypoint.
{
	tN2kMsg msg;
	SetN2kPGN127237(		// PGN_HEADING_TRACK_CONTROL
		msg,
		N2kOnOff_Unavailable,           // RudderLimitExceeded
		N2kOnOff_Unavailable,           // OffHeadingLimitExceeded
		N2kOnOff_Unavailable,           // OffTrackLimitExceeded
		boat.getAutopilot() ? N2kOnOff_On : N2kOnOff_Off, // Override (used here to indicate autopilot state)
		N2kSM_HeadingControl,           // SteeringMode
		N2kTM_RudderLimitControlled,    // TurnMode (safe default)
		N2khr_true,                 	// HeadingReference
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
	int start_num = boat.getStartWPNum();
	int target_num = boat.getTargetWPNum();
	const waypoint_t *start_wp = boat.getWaypoint(start_num);
	const waypoint_t *target_wp = boat.getWaypoint(target_num);

	bool arrived = boat.getArrived();
	double wp_dist = boat.distanceToWaypoint();
	double wp_bearing = boat.headingToWaypoint();
	double rte_heading = boat.headingTo(start_wp->lat,start_wp->lon,target_wp);

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

		if (1)
		{
			// proprietary PGN 130918 - Seatalk Route Information
			// found on https://github.com/canboat/canboat/blob/master/analyzer/pgn.h

			char startName[16];
			char targetName[16];
			fillName(16,startName,start_wp->name);
			fillName(16,targetName,target_wp->name);
			
			msg.SetPGN(130918L);
			msg.Priority = 7;  					// Raymarine uses priority 7 for route info
			msg.Add2ByteUInt(1851);				// Company ID (1851 == raymarine)
			msg.Add2ByteUInt(target_num);		// Next waypoint sequence number
			msg.AddStr(targetName, 16);			// Next waypoint name (fixed 16 bytes, padded with nulls)
			msg.Add2ByteUInt(start_num);		// Current waypoint sequence number
			msg.AddStr(startName, 16);			// Current waypoint name (fixed 16 bytes, padded with nulls)
			msg.AddByte(0);  					// Assume True; Unknown byte (possibly direction reference: 0 = True, 1 = Magnetic)
			msg.Add4ByteUDouble(boat.distanceToWaypoint(), 1.0);				// Distance to next waypoint (meters, scaled as UFIX32)
			msg.Add2ByteUDouble(DegToRad(boat.headingToWaypoint()), 0.0001);	// Bearing from current position to next waypoint (True, scaled as 0.0001 rad)
			msg.Add2ByteUDouble(DegToRad(boat.headingTo(						// Bearing from current WP to next WP (True, scaled as 0.0001 rad)
				start_wp->lat,
				start_wp->lon,
				target_wp)), 0.0001);
			nmea2000.SendMsg(msg);
		}


		// didn't work

		if (0 && last_target != target_num || strcmp(last_route, boat.getRouteName()))
		{
			// PGN_ROUTE_WP_INFO

			last_route_id++;
			last_target = target_num;
			last_route = boat.getRouteName();
			display(0, "Inst2000 Routing Sending Waypoints for Route(%s)", last_route);

			const uint16_t db_id = 237;
			const uint16_t route_id = last_route_id;
			const tN2kNavigationDirection direction = N2kdir_forward;
			const tN2kGenericStatusPair supplementary = N2kDD002_No;

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


		// PGN_NAVIGATION_DATA,
		// Note that unlike NMEA0183, the waypoint name is not included.

		time_t now = time(NULL);
		uint16_t eta_date = (now / SECONDS_PER_DAY);  // Days since Jan 1, 1970

		SetN2kPGN129284(msg, 255,		// msg, sid
			wp_dist * NM_TO_METERS,		// double DistanceToWaypoint (undocumented: IN METERS!!)
			N2khr_true,					// tN2kHeadingReference BearingReference
			arrived, 					// bool PerpendicularCrossed
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

	SetN2kPGN127488(			// PGN_ENGINE_RAPID
			msg,
			0,							// EngineInstance
			boat.getRPM(),				// EngineSpeed
			boat.getBoostPressure()  * PSI_TO_PASCAL,	// EngineBoostPressure
			N2kUInt8NA);				// EngineTiltTrim
	nmea2000.SendMsg(msg);

	static tN2kEngineDiscreteStatus1 status1;		// filled with zeros
	static tN2kEngineDiscreteStatus2 status2;		// filled with zeros

	SetN2kPGN127489(msg,		// PGN_ENGINE_DYNAMIC
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

	display(1,"genset2000",0);

	tN2kMsg msg;

#if 1
	msg.SetPGN(65288L);			// PGN_SEATALK_GEN_INFO
	msg.Priority = 3;
	msg.AddByte(0x10);			// Engine Instance (0x10 = genset)
	msg.Add2ByteUInt(3000);		// RPM (3000 * 0.25 = 750 RPM)
	msg.Add2ByteUInt(12000);	// Voltage (120.00 V)
	msg.Add2ByteUInt(100);		// Current (10.0 A)
	msg.AddByte(0x01);			// Status Flags (e.g., running)
	msg.Add2ByteUInt(500);		// Load % (50.0%)
	msg.Add2ByteUInt(80);		// Fuel Rate (8.0 L/h)
	msg.Add2ByteUInt(0);		// Reserved or temp
	nmea2000.SendMsg(msg);
#endif

#if 1
	msg.SetPGN(65026L);			// PGN_GEN_PHASE_A_AC_POWER
	msg.Priority = 3;
	msg.Add4ByteUInt(400);		// Real Power (Watts), signed 32-bit
	msg.Add4ByteUInt(500);		// Apparent Power (VA), signed 32-bit
	nmea2000.SendMsg(msg);
#endif

#if 1

	msg.SetPGN(65027L);			// PGN_GEN_PHASE_A_BASIC_AC
	msg.Priority = 3;
	msg.Add2ByteUDouble(120.0, 0.01);   	// Line-Line Voltage (V)
	msg.Add2ByteUDouble(120.0, 0.01);   	// Line-Neutral Voltage (V)
	msg.Add2ByteUDouble(60.0, 1.0/128.0); 	// Frequency (Hz)
	msg.Add2ByteUDouble(10.0, 0.1);     	// RMS Current (A)
	nmea2000.SendMsg(msg);
#endif

#if 1
	msg.SetPGN(65029L);			// PGN_TOTAL_AC_POWER
	msg.Priority = 3;
	msg.Add4ByteUInt(1200);		// Real Power (Watts), ie 1200, signed 32-bit with offset
	msg.Add4ByteUInt(1500);		// Apparent Power (VA), ie 1500, signed 32-bit with offset
	nmea2000.SendMsg(msg);
#endif

#if 1
	msg.SetPGN(65030L);			// PGN_AVERAGE_AC_QUANTITIES
	msg.Priority = 3;
	msg.Add2ByteUDouble(120.0, 0.01);   	// Line-Line Voltage
	msg.Add2ByteUDouble(120.0, 0.01);   	// Line-Neutral Voltage
	msg.Add2ByteUDouble(60.0, 1.0/128.0); 	// Frequency
	msg.Add2ByteUDouble(10.0, 0.1);     	// RMS Current
	nmea2000.SendMsg(msg);
#endif

#if 1
	msg.SetPGN(65288L);          // PGN_SEATALK_GEN_INFO Raymarine proprietary PGN
	msg.Priority = 3;
	msg.AddByte(0xFF);           		// SID (optional)
	msg.AddByte(1);              		// EngineInstance (1 = genset)
	msg.Add2ByteUDouble(1800.0, 0.25); 	// RPM (scaled by 0.25)
	msg.Add2ByteUDouble(120.0, 0.01);  	// Voltage (scaled by 0.01)
	msg.Add2ByteUDouble(5.0, 0.1);     	// Current (scaled by 0.1)
	msg.Add2ByteUDouble(2.5, 0.1);     	// Load (scaled by 0.1)
	msg.Add2ByteUDouble(1.2, 0.1);    	// Fuel Rate (scaled by 0.1)
	msg.AddByte(0);              		// Status byte (0 = OK)
	nmea2000.SendMsg(msg);

	msg.SetPGN(127503L);         // PGN_AC_INPUT_STATUS
	msg.Priority = 3;
	msg.AddByte(0xFF);           		// SID (Sequence ID, optional)
	msg.AddByte(0);              		// AC Instance (0 = first output channel)
	msg.Add2ByteUDouble(120.0, 0.01);   // Line Voltage in Volts (resolution 0.01 V)
	msg.Add2ByteUDouble(60.0, 0.001);   // Line Frequency in Hz (resolution 0.001 Hz)
	msg.Add2ByteUDouble(10.0, 0.1);     // Current in Amps (resolution 0.1 A)
	msg.Add2ByteUDouble(1.0, 0.001);    // Power Factor (1.0 = unity)
	msg.AddByte(0);              		// Reserved

	msg.SetPGN(127504L);		// PGN_AC_OUTPUT_STATUS
	msg.Priority = 3;
	msg.AddByte(0xFF);           		// SID (Sequence ID, optional)
	msg.AddByte(0);              		// AC Instance (0 = first output channel)
	msg.Add2ByteUDouble(120.0, 0.01);   // Line Voltage in Volts (resolution 0.01 V)
	msg.Add2ByteUDouble(60.0, 0.001);   // Line Frequency in Hz (resolution 0.001 Hz)
	msg.Add2ByteUDouble(10.0, 0.1);     // Current in Amps (resolution 0.1 A)
	msg.Add2ByteUDouble(1.0, 0.001);    // Power Factor (1.0 = unity)
	msg.AddByte(0);                  	// Reserved
	nmea2000.SendMsg(msg);
#endif

#if 1	// nothing worked

	int instance = 0x01;

	SetN2kPGN127488(			// PGN_ENGINE_RAPID
			msg,
			instance,			// EngineInstance
			boat.getGenRPM(),	// EngineSpeed
			N2kDoubleNA,		// EngineBoostPressure
			N2kUInt8NA);		// EngineTiltTrim
	nmea2000.SendMsg(msg);

	static tN2kEngineDiscreteStatus1 status1;		// filled with zeros
	static tN2kEngineDiscreteStatus2 status2;		// filled with zeros

	SetN2kPGN127489(msg,		// PGN_ENGINE_DYNAMIC
		instance,									// EngineInstance
		boat.getGenOilPressure() * PSI_TO_PASCAL,	// EngineOilPress      in Pascal
		FToKelvin(boat.getOilTemp()),				// EngineOilTemp       in Kelvin
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
