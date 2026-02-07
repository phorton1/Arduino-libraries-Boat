//-------------------------------------------
// inst2000_in.cpp
//-------------------------------------------
// NMEA2000 monitoring

#include "inst2000.h"
#include "instSimulator.h"
#include <N2kMessages.h>
#include <myDebug.h>

#define WITH_NMEA2000_BINARY	0

#if WITH_NMEA2000_BINARY
#include "boatBinary.h"
#endif


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


String RadOrNAToDeg(double v)
{
	if (!N2kIsNA(v))
	{
		static char buf[32];
		snprintf(buf,sizeof(buf),"%0.1f",RadToDeg(v));
		return String(buf);
	}
	return String("NA");
}



//---------------------------------
// onBusMessage
//---------------------------------
// Handles sensor messages and can show others from the bus
// i.e. PGN=130316 temperatureC = -100000000000 ?!?!?!

#if WITH_NMEA2000_BINARY
	// initialt implementation is only a binary packet to teensyBoat.pm


	static void sendActisense(const tN2kMsg &msg)
		// The Actisense frame is simple:
		//		0xA0 start, length byte, 0x02, 0x93, then
		//		priority, PGN (3 bytes LE), source, destination, data length, data bytes, and
		//      finally an 8?bit checksum which is the sum of all bytes from the length field
		//			through the last data byte. Timo’s
	{
		#define MAX_NMEA2000_BINARY_BUF 	(255 + 5)
			// 255 is maximum actisense packet size, plus 5 for my binary wrapper

		static uint8_t buf[MAX_NMEA2000_BINARY_BUF + 5];
		int idx = 0;

		// start the binary packet
		idx = startBinary(buf, BINARY_TYPE_2000);

		// Actisense header
		buf[idx++] = 0xA0;        // Start delimiter
		int cs_start = idx;
		
		buf[idx++] = 0x00;        // Placeholder for length
		buf[idx++] = 0x02;        // Actisense "message" group
		buf[idx++] = 0x93;        // NMEA2000 message type

		// PGN metadata
		buf[idx++] = msg.Priority;
		buf[idx++] = msg.PGN & 0xFF;
		buf[idx++] = (msg.PGN >> 8) & 0xFF;
		buf[idx++] = (msg.PGN >> 16) & 0xFF;
		buf[idx++] = msg.Source;
		buf[idx++] = msg.Destination;
		buf[idx++] = msg.DataLen;

		// PGN data bytes
		for (int i = 0; i < msg.DataLen; i++)
		{
			buf[idx++] = msg.Data[i];
		}

		// Compute length (everything after 0xA0 and before checksum)
		int payload_len = idx - 2;   // exclude 0xA0 and length byte
		buf[1] = payload_len;

		// Compute checksum (sum of bytes from length through last data byte)
		uint8_t csum = 0;
		for (int i = cs_start; i < idx; i++)
		{
			csum += (uint8_t)buf[i];
		}

		buf[idx++] = csum;
		endBinary(buf,idx);
		Serial.write(buf,idx);
	}
#endif






// static
void inst2000::onBusMessage(const tN2kMsg &msg)
{
	bool msg_handled = false;
	static uint32_t msg_counter = 0;
	msg_counter++;

	// g_MON[PORT_2000] is bitwise
	//	MON2000_SENSORS
	//	MON2000_AIS_GPS
	//	MON2000_PROPRIETARY
	//	MON2000_UNKNOWN
	//	MON2000_BUS_IN
	//	MON2000_BUS_OUT	

	int mon = inst_sim.g_MON[PORT_2000];
	bool b_mon_sensors 	= mon & MON2000_SENSORS;
	bool b_mon_ais_gps  = mon & MON2000_AIS_GPS;

	// display_string(BUS_COLOR,0,msgToString(msg,"BUS: ").c_str());

	if (msg.Destination == 255 ||
		msg.Destination == nmea2000.m_source_address)
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
				if (b_mon_sensors)
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
				if (b_mon_sensors)
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
				if (b_mon_sensors)
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
				if (b_mon_sensors)
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
				if (b_mon_sensors)
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
				if (b_mon_sensors)
					display(0,"%3d(%d) depth    : %0.3f meters",msg.Source,msg_counter,d1);
			}
			else
				my_error("Parsing PGN_DEPTH(128267)",0);
		}
		else if (msg.PGN == PGN_DISTANCE_LOG)
		{
			uint16_t days;
			double   secs;
			uint32_t logv;
			uint32_t tripv;

			if (ParseN2kPGN128275(msg,days,secs,logv,tripv))
			{
				msg_handled = true;

				if (b_mon_sensors)
				{
					display(0,"%3d(%d) distlog  : log(%0.1f) trip(%0.1f) NM",
						msg.Source,msg_counter,
						((float)logv)/NM_TO_METERS,((float)tripv)/NM_TO_METERS);
				}
			}
			else
				my_error("Parsing PGN_DISTANCE_LOG(128275)",0);
		}
		else if (msg.PGN == PGN_POSITION_RAPID_UPDATE)
		{
			double lat;
			double lon;
			if (ParseN2kPGN129025(msg,lat,lon))
			{
				msg_handled = true;
				if (b_mon_sensors)
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
				if (b_mon_sensors || b_mon_ais_gps)
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
				if (b_mon_sensors || b_mon_ais_gps)
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
		else if (msg.PGN == PGN_CROSS_TRACK_ERROR)
		{
			uint8_t msg_id;
			tN2kXTEMode xteMode;
			bool terminated;
			double xte;
			if (ParseN2kPGN129283(msg,msg_id,xteMode,terminated,xte))
			{
				msg_handled = true;
				if (b_mon_sensors)
					display(0,"%3d(%d) xte_err  : terminated(%d) xte_nm(%0.3f)",
						msg.Source,
						msg_counter,
						terminated,
						xte == N2kDoubleNA ? -1 : xte * NM_TO_METERS);
			}
			else
				my_error("Parsing PGN_CROSS_TRACK_ERROR",0);
;
		}
		else if (msg.PGN == PGN_GNSS_SATS_IN_VIEW)
		{
			uint8_t sat_index = 0;
			tSatelliteInfo sat_info;

			while (ParseN2kPGN129540(msg, sat_index, sat_info))
			{
				msg_handled = true;
				if (b_mon_sensors || b_mon_ais_gps)
				{
					display(0,"%3d(%d) sat_view : index(%d) PRN(%3d) elev(%2.0f) azimuth(%3.0f) SNR(%2.0f) used(%d)",
						msg.Source,
						msg_counter,
						sat_index,
						sat_info.PRN,
						RadToDeg(sat_info.Elevation),
						RadToDeg(sat_info.Azimuth),
						sat_info.SNR,
						sat_info.UsageStatus);
				}
				sat_index++;
			}
			// else
			//	my_error("Parsing PGN_GNSS_SATS_IN_VIEW",0);
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
				if (b_mon_sensors || b_mon_ais_gps)
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
				if (b_mon_sensors || b_mon_ais_gps)
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
		else if (msg.PGN == PGN_WIND_DATA)
		{
			uint8_t sid;
			double  wspd;
			double  wang;
			tN2kWindReference wref;
			if (ParseN2kPGN130306(msg,sid,wspd,wang,wref))
			{
				msg_handled = true;
				if (b_mon_sensors)
				{
					// it is a known flaw that the E80 will transmit speed(0) from Seatalk inputs.
					
					display(0,"%3d(%d) wind     : speed(%0.1f) angle(%0.1f)",
						msg.Source,msg_counter,
						wspd/NM_TO_METERS, RadToDeg(wang));
				}
			}
			else
				my_error("Parsing PGN_WIND_DATA(130306)",0);
		}
		else if (msg.PGN == PGN_TEMPERATURE)
		{
			// temperature is in kelvin
			uint8_t instance;
			tN2kTempSource source;

			if (ParseN2kPGN130316(msg,sid,instance,source,d1,d2))
			{
				msg_handled = true;
				if (b_mon_sensors)
					display(0,"%3d(%d) temp     : %0.3fC",msg.Source,msg_counter,KelvinToC(d1));
			}
			else
				my_error("Parsing PGN_TEMPERATURE(130316)",0);
		}
		else if (msg.PGN == PGN_DIRECTION_DATA)	// prh
		{
			tN2kDataMode dataMode;
			tN2kHeadingReference cogRef;
			uint8_t sid;
			double  cog;			// usually NA from E80
			double  headMag;		// usually NA from E80
			double  headTrue;		// Raymarine puts heading here
			double  variation;
			double  deviation;
			double  reserved;

			if (ParseN2kPGN130577(msg,
					dataMode,
					cogRef,
					sid,
					cog,
					headMag,
					headTrue,
					variation,
					deviation,
					reserved))
			{
				msg_handled = true;
				if (b_mon_sensors)
				{
					display(0,"%3d(%d) dirdata  : cog(%s) headMag(%s) headTrue(%s)",
						msg.Source,msg_counter,
						RadOrNAToDeg(cog).c_str(),
						RadOrNAToDeg(headMag).c_str(),
						RadOrNAToDeg(headTrue).c_str() );
				}
			}
			else
				my_error("Parsing PGN_DIRECTION_DATA(130577)",0);
		}


	}	// 255 or MONITOR_NMEA_ADDRESS

	// send "handled" messages via binary protocol
	// before we "handle" proprietary and system messzges
	
	#if WITH_NMEA2000_BINARY
		if (msg_handled && (g_BINARY & BINARY_TYPE_2000))
			sendActisense(msg);
	#endif

	// I don't think we get bus messages here;
	// if so, we want to filter them
	// show unhandled messages as UNKNOWN: in white

	if (!msg_handled && (
		msg.PGN == PGN_ACK				||
		msg.PGN == PGN_REQUEST			||
		msg.PGN == PGN_ADDRESS_CLAIM	||
		msg.PGN == PGN_PGN_LIST			||
		msg.PGN == PGN_HEARTBEAT		||
		msg.PGN == PGN_PRODUCT_INFO		||
		msg.PGN == PGN_DEVICE_CONFIG ))
	{
		if (mon & MON2000_BUS_IN)
		{
			const char *name =
				msg.PGN == PGN_ACK				? "BUS_ACK:" :
				msg.PGN == PGN_REQUEST			? "BUS_REQUEST: " :
				msg.PGN == PGN_ADDRESS_CLAIM	? "BUS_ADDRESS_CLAIM: " :
				msg.PGN == PGN_PGN_LIST			? "BUS_PGN_LIST: " :
				msg.PGN == PGN_HEARTBEAT		? "BUS_HEARTBEAT: " :
				msg.PGN == PGN_PRODUCT_INFO		? "BUS_PRODUCT_INFO: " :
				msg.PGN == PGN_DEVICE_CONFIG	? "BUS_DEVICE_CONFIG: " : "";
			display_string(BUS_COLOR,0,msgToString(msg,name).c_str());
		}
		msg_handled = true;
	}

	if (!msg_handled)
	{
		bool b_mon_prop 	= mon & MON2000_PROPRIETARY;
		bool b_mon_unknown 	= mon & MON2000_UNKNOWN;
		bool is_known_proprietary =
			msg.PGN == PGN_PROP_B_65311 ||
			msg.PGN == PGN_PROP_B_65362 ||
			msg.PGN == PGN_PROP_B_65364 ||
			msg.PGN == PGN_PROP_B_129044 ||
			msg.PGN == PGN_PROP_B_130846;
		const char *name =
			is_known_proprietary ? "PROPRIETARY: " : "UNKNOWN: ";

		if ((is_known_proprietary && b_mon_prop) ||
			(!is_known_proprietary && b_mon_unknown))
		{
			display_string(BUS_COLOR,0,msgToString(msg,name).c_str());
		}
	}

}	// onBusMessage()




// end of inst2000_in.cpp