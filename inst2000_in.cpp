//-------------------------------------------
// inst2000_in.cpp
//-------------------------------------------
// NMEA2000 monitoring

#include "inst2000.h"
#include "instSimulator.h"
#include <N2kMessages.h>
#include <myDebug.h>


#define BUS_COLOR "\033[37m"
	// 37 = WHITE


static String showIfValid(double d, const char *format)
	// return a string of floating point number with the given format string
	// if it is valid,
	// otherwise return an empty string
{
	String s;
	if (d != N2kDoubleNA)
	{
		char buf[32];		// CAREFUL, 31 chars max!
		sprintf(buf,format,d);
		s = buf;
	}
	return s;
}


static String msgToString(const tN2kMsg &msg,  const char *prefix=0, bool with_data=true)
{
	String rslt;
	if (prefix) rslt += prefix;
	rslt += String(N2kMillis());
	rslt += " : ";
	rslt += "Pri: ";
	rslt += String(msg.Priority);
	rslt += " PGN: ";
	rslt += String(msg.PGN);
	rslt += " Source: ";
	rslt += String(msg.Source);
	rslt += " Dest: ";
	rslt += String(msg.Destination);
	rslt += " Len: ";
	rslt += String(msg.DataLen);
	if (with_data)
	{
		rslt += " Data:";
		for (int i=0; i<msg.DataLen; i++)
		{
			if (i) rslt += ",";
			rslt += String(msg.Data[i],HEX);
		}
  }
  return rslt;
}


//---------------------------------
// onBusMessage
//---------------------------------
// Handles sensor messages and can show others from the bus
// i.e. PGN=130316 temperatureC = -100000000000 ?!?!?!

// static
void inst2000::onBusMessage(const tN2kMsg &msg)
{
	bool msg_handled = false;
	static uint32_t msg_counter = 0;
	msg_counter++;

	// display_string(BUS_COLOR,0,msgToString(msg,"BUS: ").c_str());

	if (msg.Destination == 255 ||
		msg.Destination == INST2000_NMEA_ADDRESS)
	{
		uint8_t sid;
		double d1,d2,d3;

		if (msg.PGN == PGN_VESSEL_HEADING)
		{
			// heading is in radians
			tN2kHeadingReference ref;
			if (ParseN2kPGN127250(msg, sid, d1,d2,d3, ref))
			{
				msg_handled = true;
				if (m_MON_SENSORS)
					display(0,"%3d(%d) heading  : %0.3f degrees",msg.Source,msg_counter,RadToDeg(d1));
			}
			else
				my_error("Parsing PGN_HEADING(128267)",0);
		}
		else if (msg.PGN == PGN_ENGINE_RAPID)
		{
			unsigned char engine_num;
			double rpms;
			double boost;
			int8_t tilt;
			if (ParseN2kPGN127488(msg,engine_num,rpms,boost,tilt))
			{
				msg_handled = true;
				if (m_MON_SENSORS)
					display(0,"%3d(%d) rpms     : %d",msg.Source,msg_counter,((int) rpms) );
			}
			else
				my_error("Parsing PGN_ENGINE_RAPID",0);
		}
		else if (msg.PGN == PGN_ENGINE_DYNAMIC)
		{
			unsigned char engine_num;
			double oil_pressure;
			double oil_temp;
			double coolant_temp;
			double alt_voltage;
			double fuel_rate;
			double hours;
			double coolant_pressure;
			double fuel_pressure;
			int8_t load;
			int8_t torque;
			tN2kEngineDiscreteStatus1 status1;
			tN2kEngineDiscreteStatus2 status2;

			#define PSI_TO_PASCAL		6895.0
			#define GALLON_TO_LITRE		3.785

			if (ParseN2kPGN127489(msg,
					engine_num,			// EngineInstance
					oil_pressure,		// EngineOilPress      in Pascal
					oil_temp,			// EngineOilTemp       in Kelvin
					coolant_temp,		// EngineCoolantTemp   in Kelvin
					alt_voltage,		// AltenatorVoltage    in Voltage
					fuel_rate,			// FuelRate            in litres/hour
					hours,				// EngineHours         in seconds
					coolant_pressure,	// EngineCoolantPress  in Pascal
					fuel_pressure,		// EngineFuelPress     in Pascal
					load,				// EngineLoad          in %
					torque,				// EngineTorque        in %
					status1,			// Status1             Engine Discrete Status 1
					status2))			// Status2             Engine Discrete Status 2
			{
				msg_handled = true;
				if (m_MON_SENSORS)
					display(0,"%3d(%d) engine   : temp(%0.0f) pres(%0.0f) volts(%0.1f) rate(%0.1f)",
						msg.Source,
						msg_counter,
						KelvinToF(coolant_temp),
						oil_pressure/PSI_TO_PASCAL,
						alt_voltage,
						fuel_rate / GALLON_TO_LITRE);
			}
			else
				my_error("Parsing PGN_ENGINE_DYNAMIC",0);
		}
		else if (msg.PGN == PGN_FLUID_LEVEL)
		{
			unsigned char instance;
			tN2kFluidType fluid_type;
			double level;
			double capacity;
			if (ParseN2kPGN127505(msg, instance, fluid_type, level, capacity))
			{
				msg_handled = true;
				if (m_MON_SENSORS)
					display(0,"%3d(%d) fuel     : tank(%d) = %d%%",
						msg.Source,
						msg_counter,
						instance,
						(int) level);
			}
			else
				my_error("Parsing PGN_FLUID_LEVEL",0);
		}
		else if (msg.PGN == PGN_SPEED_WATER_REF)
		{
			// speed is in meters/second
			tN2kSpeedWaterReferenceType SWRT;
			if (ParseN2kPGN128259(msg, sid, d1, d2, SWRT))
			{
				msg_handled = true;
				if (m_MON_SENSORS)
					display(0,"%3d(%d) speed    : %0.3f kts",msg.Source,msg_counter,msToKnots(d1));
			}
			else
				my_error("Parsing PGN_SPEED(128267)",0);
		}
		else if (msg.PGN == PGN_WATER_DEPTH)
		{
			// depth is in meters
			if (ParseN2kPGN128267(msg,sid,d1,d2,d3))
			{
				msg_handled = true;
				if (m_MON_SENSORS)
					display(0,"%3d(%d) depth    : %0.3f meters",msg.Source,msg_counter,d1);
			}
			else
				my_error("Parsing PGN_DEPTH(128267)",0);
		}
		else if (msg.PGN == PGN_POSITION_RAPID_UPDATE)
		{
			double lat;
			double lon;
			if (ParseN2kPGN129025(msg,lat,lon))
			{
				msg_handled = true;
				if (m_MON_SENSORS)
					display(0,"%3d(%d) position : lat(%0.6f) lon(%0.6f)",msg.Source,msg_counter,lat,lon);
			}
			else
				my_error("Parsing PGN_TEMPERATURE(130316)",0);
		}
		else if (msg.PGN == PGN_GNSS_POSITION_DATA)
		{
			uint16_t days_since_1970;
			double   seconds_since_midnight,lat,lon,altitude;
			tN2kGNSStype gnss_type;
			tN2kGNSSmethod gnss_method;
			unsigned char num_sats;
			double hdop, pdop, geo_separation;
			unsigned char num_ref_stations;
			tN2kGNSStype ref_station_type;
			uint16_t ref_station_id;
			double age_of_correction;

			if (ParseN2kPGN129029(msg, sid, days_since_1970, seconds_since_midnight, lat, lon, altitude,
								  gnss_type, gnss_method, num_sats, hdop, pdop, geo_separation,
								  num_ref_stations, ref_station_type, ref_station_id, age_of_correction))
			{
				msg_handled = true;
				if (m_MON_GPS)
					display(0,"%3d(%d) gnss_data: lat(%0.3f) lon(%0.3f) num_sats(%d) hdop(%0.2f)",
						msg.Source,
						msg_counter,
						lat,
						lon,
						num_sats,
						hdop);
			}
			else
				my_error("Parsing PGN_GNSS_POSITION_DATA",0);
		}
		else if (msg.PGN == PGN_AIS_CLASS_B_POSITION)
		{
			uint8_t msg_id;
			tN2kAISRepeat repeat;
			uint32_t mmsi;
			double lat, lon;
			bool accuracy, raim;
			uint8_t seconds;
			double cog, sog;
			tN2kAISTransceiverInformation trans_info;
			double heading;
			tN2kAISUnit unit;
			bool disp, dsc, band, msg22;
			tN2kAISMode mode;
			bool state;

			if (ParseN2kPGN129039(msg, msg_id, repeat, mmsi, lat, lon, accuracy, raim, seconds, cog, sog,
								  trans_info, heading, unit, disp, dsc, band, msg22, mode, state, sid))
			{
				msg_handled = true;
				if (m_MON_SENSORS)
					display(0,"%3d(%d) ais_a    : mmsi(%d) lat(%0.3f) lon(%0.3f) %s sog(%0.1f) %s",
						msg.Source,
						msg_counter,
						mmsi,
						lat,
						lon,
						showIfValid(cog,"cog(%0.1f)").c_str(),
						sog,
						showIfValid(heading,"heading(%0.1f)").c_str());
			}
			else
				my_error("Parsing PGN_AIS_CLASS_B_POSITION",0);
		}
		else if (msg.PGN == PGN_GNSS_SATS_IN_VIEW)
		{
			uint8_t sat_index = 0;
			tSatelliteInfo sat_info;

			if (ParseN2kPGN129540(msg, sat_index, sat_info))
			{
				msg_handled = true;
				if (m_MON_GPS)
					display(0,"%3d(%d) sat_view : index(%d) PRN(%d) elev(%0.3f) azimuth(%0.3f) SNR(%0.1f)",
						msg.Source,
						msg_counter,
						sat_index,
						sat_info.PRN,
						sat_info.Elevation,
						sat_info.Azimuth,
						sat_info.SNR);
			}
			else
				my_error("Parsing PGN_GNSS_SATS_IN_VIEW",0);
		}
		else if (msg.PGN == PGN_AIS_STATIC_B_PART_A)
		{
			uint8_t msg_id;
			tN2kAISRepeat repeat;
			uint32_t mmsi;
			char name[21];	// maximum name 20 characters
			tN2kAISTransceiverInformation trans_info;

			memset(name,0,21);
			if (ParseN2kPGN129809(msg, msg_id, repeat, mmsi, name, 20, trans_info, sid))
			{
				msg_handled = true;
				if (m_MON_SENSORS)
					display(0,"%3d(%d) ais_b_a  : mmsi(%d) name: %s",
						msg.Source,
						msg_counter,
						mmsi,
						name);
			}
			else
				my_error("Parsing PGN_AIS_STATIC_B_PART_A",0);
		}
		else if (msg.PGN == PGN_AIS_STATIC_B_PART_B)
		{
			uint8_t msg_id;
			tN2kAISRepeat repeat;
			uint32_t mmsi;
			uint8_t vessel_type;
			char vendor[21];		// my maximum vendor unit number is 20 characters
			char call_sign[21];		// my maximum call_sign length is 20 chars
			double length;
			double beam;
			double pos_ref_stbd;
			double pos_ref_bow;
			uint32_t mothership_id;	// mmsi?
			tN2kAISTransceiverInformation trans_info;

			memset(vendor,0,21);
			memset(call_sign,0,21);

			if (ParseN2kPGN129810(msg, msg_id, repeat, mmsi, vessel_type, vendor, 20, call_sign, 20,
								  length, beam, pos_ref_stbd, pos_ref_bow, mothership_id, trans_info, sid))
			{
				msg_handled = true;
				if (m_MON_SENSORS)
				{
					int i_length = length * 3.28084;
					int i_beam = beam * 3.28084;

					display(0,"%3d(%d) ais_b_b  : mmsi(%d) call_sign(%s) length(%d) beam(%d)",
						msg.Source,
						msg_counter,
						mmsi,
						call_sign,
						i_length,
						i_beam);
				}
			}
			else
				my_error("Parsing PGN_AIS_STATIC_B_PART_B",0);
		}
		else if (msg.PGN == PGN_TEMPERATURE)
		{
			// temperature is in kelvin
			uint8_t instance;
			tN2kTempSource source;

			if (ParseN2kPGN130316(msg,sid,instance,source,d1,d2))
			{
				msg_handled = true;
				if (m_MON_SENSORS)
					display(0,"%3d(%d) temp     : %0.3fC",msg.Source,msg_counter,KelvinToC(d1));
			}
			else
				my_error("Parsing PGN_TEMPERATURE(130316)",0);
		}


	}	// 255 or MONITOR_NMEA_ADDRESS


	// show unhandled messages as BUS: in white

	if (!msg_handled && m_MON_BUS)
	{
		bool is_known_proprietary =
			msg.PGN == PGN_PROP_B_65311 ||
			msg.PGN == PGN_PROP_B_65362 ||
			msg.PGN == PGN_PROP_B_130846;

		if (!is_known_proprietary || m_MON_PROP)
			display_string(BUS_COLOR,0,msgToString(msg,"BUS: ").c_str());
	}

}	// onBusMessage()




// end of inst2000_in.cpp