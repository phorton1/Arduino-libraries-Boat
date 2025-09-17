//---------------------------------------------
// boatSimulator.cpp
//---------------------------------------------

#include "boatSimulator.h"
#include <myDebug.h>
#include <math.h>
#include "boatUtils.h"
#include "boatBinary.h"
#include <TimeLib.h>

#define dbg_sim	 (1 - boat.g_MON_SIM)

#define SIMULATION_INTERVAL		1000	// ms
#define CLOSEST_NONE			65535	// exact


boatSimulator boat;
int boatSimulator::g_MON_SIM = 0;
	// global instance



//----------------------------------------
// initialization
//----------------------------------------


void boatSimulator::start()
{
	display(0,"STARTING SIMULATOR",0);
	m_running = true;
	sendBinaryBoatState(1);
}

void boatSimulator::stop()
{
	display(0,"STOPPING SIMULATOR",0);
	m_running = false;
	sendBinaryBoatState(1);
}


void boatSimulator::init()
{
	// initial setting of m_route, m_waypoints, and m_num_waypoints
	// also sets m_waypoint num to 1,

	display(0,"boatSimulator::init() started",0);
	proc_entry();

	if (!m_inited)
		setRoute(simulator_routes[0].name);

	m_start_wp_num = 0;
	m_target_wp_num = 1;
	m_latitude = m_waypoints[0].lat;
	m_longitude = m_waypoints[0].lon;
	
	// state vars

	m_inited 	= true;
	// m_running 	= false;
	m_autopilot = false;
	m_routing 	= false;
	m_arrived 	= false;

	// simulation vars

	m_depth 	 	= 10;
	m_sog 		 	= 0;
	m_cog 		 	= 180;
	m_wind_angle 	= 90;	// true east
	m_wind_speed 	= 12;
	m_rpm 		 	= 0;

	m_oil_pressure 	= 0;
	m_oil_temp  	= 0;
	m_coolant_temp 	= 0;
	m_alt_voltage 	= 0;
	m_fuel_rate 	= 0;
	m_fuel_level1 	= 0;
	m_fuel_level2 	= 0;

	m_genset 			= false;
	m_gen_rpm 			= 0;
	m_gen_oil_pressure	= 0;
	m_gen_cool_temp		= 0;
	m_gen_voltage 		= 0;
	m_gen_freq 			= 0;

	// update vars

	m_update_num = 0;
	m_last_update_ms = 0;
	m_closest = CLOSEST_NONE;

	calculateApparentWind(false);
		// sets m_app_wind_angle and m_app_wind_speed;
		// with debugging

	sendBinaryBoatState(1);

	proc_leave();
	display(0,"boatSimulator::init() finished",0);
}


//-------------------------------------------
// setters
//-------------------------------------------

void boatSimulator::setDepth(float depth)		{ m_depth = depth; 	sendBinaryBoatState(!m_running); }
void boatSimulator::setSOG(float sog)			{ m_sog = sog; calculateApparentWind(); m_rpm=sog?1800:0; sendBinaryBoatState(!m_running);}
void boatSimulator::setCOG(float cog)			{ m_cog = cog; calculateApparentWind(); sendBinaryBoatState(!m_running);}
void boatSimulator::setWindAngle(float angle) 	{ m_wind_angle = angle; calculateApparentWind(); sendBinaryBoatState(!m_running);}
void boatSimulator::setWindSpeed(float speed)	{ m_wind_speed = speed; calculateApparentWind(); sendBinaryBoatState(!m_running);}
void boatSimulator::setRPM(uint16_t rpm)		{ m_rpm = rpm; sendBinaryBoatState(!m_running);}
void boatSimulator::setGenset(bool on)			{ m_genset = on; display(0,"GENSET %s",m_genset?"ON":"OFF"); sendBinaryBoatState(!m_running);}


const route_t *boatSimulator::getRoute(const char *name)
{
	const route_t *found = 0;
	for (int i=0; i<simulator_num_routes; i++)
	{
		const route_t *rte = &simulator_routes[i];
		if (String(name).equalsIgnoreCase(String(rte->name)))
		{
			found = rte;
			i = simulator_num_routes;
		}
	}

	if (!found)
	{
		my_error("Could not find route(%s)",name);
		return 0;
	}
	return found;
}

void boatSimulator::setRoute(const char *name)
{
	m_sog = 0;
	m_autopilot = false;
	m_routing = false;
	m_arrived = false;
	m_closest = CLOSEST_NONE;

	const route_t *found = getRoute(name);
	if (!found) return;

	m_start_wp_num = 0;
	m_target_wp_num = 1;
	m_waypoints = found->wpts;
	m_num_waypoints = found->num_wpts;
	display(0,"ROUTE(%s) num_waypoints=%d",found->name,m_num_waypoints);

	m_latitude = m_waypoints[0].lat;
	m_longitude = m_waypoints[0].lon;

	sendBinaryBoatState(m_inited && !m_running);
}



void boatSimulator::setTargetWPNum(uint8_t wp_num)
{
	if (wp_num < 0 || wp_num >= m_num_waypoints)
	{
		my_error("ILLEGAL WP_NUM(%d)",wp_num);
		return;
	}

	m_target_wp_num = wp_num;
	m_cog = headingToWaypoint();

	m_arrived = false;
	m_closest = CLOSEST_NONE;
	calculateApparentWind();

	const waypoint_t *wp = &m_waypoints[wp_num];

	display(0,"SET TARGET WP[%d] %s %s %s cog(%d)",
		wp_num,
		wp->name,
		strDegreeMinutes(wp->lat).c_str(),
		strDegreeMinutes(wp->lon).c_str(),
		(int) m_cog);

	sendBinaryBoatState(!m_running);
}


void boatSimulator::setStartWPNum(uint8_t wp_num)
{
	if (wp_num >= m_num_waypoints)
	{
		my_error("ILLEGAL JUMP WP_NUM(%d)",wp_num);
		return;
	}

	m_start_wp_num = wp_num;
	const waypoint_t *wp = &m_waypoints[wp_num];
	m_latitude = wp->lat;
	m_longitude = wp->lon;
	if (!m_arrived)
		m_closest = CLOSEST_NONE;

	display(0,"SET START WP[%d] %s %s %s",
		wp_num,
		wp->name,
		strDegreeMinutes(m_latitude).c_str(),
		strDegreeMinutes(m_longitude).c_str());

	sendBinaryBoatState(!m_running);
}




void boatSimulator::setAutopilot(bool on)
{
	if (m_autopilot != on)
	{
		m_arrived = false;
		m_closest = CLOSEST_NONE;

		display(0,"AUTOPILOT %s",(on?"ON":"OFF"));
		m_autopilot = on;

		if (on)
			m_cog = headingToWaypoint();
		else if (m_routing)
			setRouting(false);

		sendBinaryBoatState(!m_running);
	}
	else
		warning(0,"AUTOPILOT ALREADY %s",(on?"ON":"OFF"));
}


void boatSimulator::setRouting(bool on)
{
	if (m_routing != on)
	{
		display(0,"ROUTING %s",(on?"ON":"OFF"));
		m_routing = on;

		if (on && !m_autopilot)
			setAutopilot(true);
		else if (!on && m_autopilot)
			setAutopilot(false);

		sendBinaryBoatState(!m_running);
	}
	else
		warning(0,"ROUTING ALREADY %s",(on?"ON":"OFF"));
}




//-----------------------------------------------
// implementation
//-----------------------------------------------

#define rad2deg(degrees)	((degrees) * M_PI / 180.0)
#define deg2rad(radians)	((radians) * (180.0 / M_PI))


void boatSimulator::run()
	// Calculate and set new latitude and longitude
	// based on cog, sog, and millis() since last call
{
	uint32_t now = millis();
	double elapsed_secs = (now - m_last_update_ms) / 1000.0;
	m_last_update_ms = now;

	if (!m_running)
		return;

	m_update_num++;
	
	display(dbg_sim,"boatSimulator::run lat(%s) lon(%s) cog(%d) sog(%d)",
		strDegreeMinutes(m_latitude).c_str(),
		strDegreeMinutes(m_longitude).c_str(),
		(int) m_cog,
		(int) m_sog);
	proc_entry();
	display(dbg_sim,"year(%d) month(%d) day(%d) hour(%d) minute(%d) second(%d)",
		getYear(),
		getMonth(),
		getDay(),
		getHour(),
		getMinute(),
		getSecond());

	// set our new position

	if (m_sog != 0)
	{
		const double EARTH_RADIUS = 6371000; 	// in meters
		const double KNOTS_TO_MPS = 0.514444;
		double distance_m = m_sog * KNOTS_TO_MPS * elapsed_secs;
		double cog_rad = rad2deg(m_cog);
		double delta_lat = (distance_m * cos(cog_rad)) / EARTH_RADIUS;
		double delta_lon = (distance_m * sin(cog_rad)) / (EARTH_RADIUS * cos(rad2deg(m_latitude)));
		double new_lat = m_latitude + deg2rad(delta_lat);
		double new_lon = m_longitude + deg2rad(delta_lon);

		if (m_latitude != new_lat ||
			m_longitude != new_lon)
		{
			display(dbg_sim+1,"new lat(%s) lon(%s)",
				strDegreeMinutes(new_lat).c_str(),
				strDegreeMinutes(new_lon).c_str());
		}
		m_latitude = new_lat;
		m_longitude = new_lon;
	}

	// adjust our heading if we're routing to a waypoint

	if (m_routing && !m_arrived)
	{
		m_cog = headingToWaypoint();
		static int last_cog = 0;
		if (last_cog != (int) m_cog)
		{
			last_cog = (int) m_cog;
			display(dbg_sim+1,"AP new cog(%d)",(int) m_cog);
			calculateApparentWind();
		}
	}

	// handle the autopilot

	if (m_autopilot)
	{
		const float NM_TO_FEET = 6076.12;
		uint32_t feet_to_wp = distanceToWaypoint() * NM_TO_FEET;
		bool arr = feet_to_wp < 400 ? 1 : 0;
		if (feet_to_wp < ((uint32_t) m_closest))
			m_closest = feet_to_wp;

		display(dbg_sim+1,"AP feet_to_wp(%d) arr(%d) m_arrived(%d) m_closest(%d)",
			feet_to_wp,
			arr,
			m_arrived,
			m_closest);
	
		// check for initial arrival

		if (arr && !m_arrived)
		{
			display(dbg_sim,"INITIAL ARRIVAL",0);
			// m_closest = feet_to_wp;
			m_arrived = true;
		}

		// check for end of arrival

		else if (m_arrived)
		{
			if (feet_to_wp <= m_closest)			// getting closeer
				m_closest = feet_to_wp;
			else  									// arrival finished
			{
				display(dbg_sim,"ARRIVAL COMPLETE",0);
				if (m_routing)						// goto next waypoint or stop
				{
					if (m_target_wp_num < m_num_waypoints - 1)	// goto next waypoint
					{
						m_start_wp_num = m_target_wp_num;
						// I don't call setTargetWPNum() because it
						// is a command and I don't want the output
						// setTargetWPNum(m_target_wp_num+1);
						m_target_wp_num++;
						m_arrived = false;
						m_closest = CLOSEST_NONE;
					}
					else
					{
						display(dbg_sim,"ROUTE COMPLETE",0);
						m_sog = 0;					    // stop the boat
						// setRouting(false);			// turn off routing
						m_routing = false;
						m_autopilot = false;
						m_arrived = false;
						calculateApparentWind();
					}
				}
				else
				{
					// setAutopilot(false);			// turn off autopilot after arrival
					m_autopilot = false;
				}
			}
		}	// already arrived
	}	// m_autpilot

	// set psudeo random engine and genset values

	m_oil_pressure = 	m_rpm == 0 ? 0 : 50 + random(-30,30); 		// psi
	m_oil_temp = 		m_rpm == 0 ? 0 : 180 + random(-40,40);   	// farenheight
	m_coolant_temp =	m_rpm == 0 ? 0 : 180 + random(-40,40);   	// farenheight
	m_alt_voltage = 	m_rpm == 0 ? 0 : 12.0 + (((float)random(-300,300)) / 100.0);
	m_fuel_rate = 		m_rpm == 0 ? 0 : 1.5 + (((float) random(-100,100)) / 100.0);  // gph
	m_fuel_level1 = 	(500.0 + ((double) random(-100,100)))/10.0; 	// 0..1
	m_fuel_level2 = 	(500.0 + ((double) random(-100,100)))/10.0; 	// 0..1

	m_gen_rpm = 		m_genset ? 3600 + random(-100,100) : 0;
	m_gen_oil_pressure=	m_genset ? 50 + random(-30,30) : 0; 	// psi
	m_gen_cool_temp	=	m_genset ? 180 + random(-40,40) : 0; 	// farenheight
	m_gen_voltage = 	m_genset ? 120 + random(-10,10) : 0;
	m_gen_freq =		m_genset ? 60 + random(-5,5) : 0;

	sendBinaryBoatState(1);

	proc_leave();
	

}	// run()



float boatSimulator::headingToWaypoint()
	// Returns heading in true degrees to given waypoint
	// from current latitute and longitude
{
	const waypoint_t *wp = &m_waypoints[m_target_wp_num];

	double delta_lon = wp->lon - m_longitude;
	double y = sin(rad2deg(delta_lon)) * cos(rad2deg(wp->lat));
	double x = cos(rad2deg(m_latitude)) * sin(rad2deg(wp->lat)) -
		sin(rad2deg(m_latitude)) * cos(rad2deg(wp->lat)) * cos(rad2deg(delta_lon));
	double heading_rad = atan2(y, x);
	double heading_deg = deg2rad(heading_rad);
	heading_deg = fmod((heading_deg + 360.0), 360.0);
	return (float) heading_deg;
}


float boatSimulator::distanceToWaypoint()
	// Returns distance in NM to given waypoint
	// from current latitute and longitude
{
	const waypoint_t *wp = &m_waypoints[m_target_wp_num];

	const double EARTH_RADIUS_NM = 3440.065; 	// in nautical miles
    double d_lat = rad2deg(wp->lat - m_latitude);
    double d_lon = rad2deg(wp->lon - m_longitude);
    double a = sin(d_lat / 2) * sin(d_lat / 2) +
        cos(rad2deg(m_latitude)) * cos(rad2deg(wp->lat)) *
        sin(d_lon / 2) * sin(d_lon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    return (float) (EARTH_RADIUS_NM * c);
}


void boatSimulator::calculateApparentWind(bool quiet /*=true*/)
	// Calculates and sets app_wind_angle and app_wind_speed
	// based on cog, sog, wind_angle, and wind_epeed
	// as degrees relative to the bow of the boat
{
	#define dbg_wind 	(dbg_sim + (quiet?2:0))

	display(dbg_wind,"calculateApparentWind speed/angle boat(%0.3f,%0.3f) wind(%0.3f,%0.3f)",
		m_sog,m_cog,m_wind_speed,m_wind_angle);

	double wx = -m_wind_speed * cos(rad2deg(m_wind_angle));
	double wy = -m_wind_speed * sin(rad2deg(m_wind_angle));
	double bx = m_sog * cos(rad2deg(m_cog));
	double by = m_sog * sin(rad2deg(m_cog));
	double ax = wx - bx;
	double ay = wy - by;

	proc_entry();
	display(dbg_wind+1,"wx(%0.3f) wy(%0.3f) bx(%0.3f) by(%0.3f) ax(%0.3f) ay(%0.3f)", wx,wy,bx,by,ax,ay);

	m_app_wind_speed = sqrt(ax * ax + ay * ay);
	m_app_wind_angle = deg2rad(atan2(ay, ax));
	m_app_wind_angle += 180.0;
	if (m_app_wind_angle >= 360.0)
		m_app_wind_angle -= 360.0;
	display(dbg_wind+1,"app_wind_speed(%0.3f) absolute angle(%0.3f)", m_app_wind_speed,m_app_wind_angle);

	m_app_wind_angle -= m_cog;
	display(dbg_wind+1,"relative angle(%0.3f)", m_app_wind_angle);

	if (m_app_wind_angle < 0)
	{
		m_app_wind_angle += 360.0;
		display(dbg_wind,"normalized angle(%0.3f)", m_app_wind_angle);
	}

	display(dbg_wind,"app_wind_speed(%0.3f) app_wind_angle(%0.3f)", m_app_wind_speed, m_app_wind_angle);
	proc_leave();
}



//-------------------------------------------
// time
//-------------------------------------------
// from teensy RTC - installed Time library - TimeLib.h

int boatSimulator::getYear()		{ return year(); }
int boatSimulator::getMonth()		{ return month(); }
int boatSimulator::getDay()			{ return day(); }
int boatSimulator::getHour()		{ return hour(); }
int boatSimulator::getMinute()		{ return minute(); }
int boatSimulator::getSecond()		{ return second(); }


void boatSimulator::setDateTime(int year, int month, int day, int hour, int minute, int second)
{
	display(0,"setDateTime(%d-%d-%d %d:%d:%d)",year,month,day,hour,minute,second);
	setTime(hour,minute,second,day,month,year);
}


//-------------------------------------------
// binary
//-------------------------------------------


#define MAX_WP_NAME		8
#define DATE_SIZE		20

void boatSimulator::sendBinaryBoatState(bool doit /*=1*/)
{
	if (!(g_BINARY & BINARY_TYPE_BOAT)) return;
	if (!doit) return;

	// display(0,"sendBinaryBoatState(%d)",doit);

	uint8_t buf[sizeof(boatSimulator) + BINARY_HEADER_LEN + 2 * (MAX_WP_NAME + 1) + DATE_SIZE + 2];		// guaranteed to be big enough

	const waypoint_t *start_wp = getWaypoint(m_start_wp_num);
	const waypoint_t *target_wp = getWaypoint(m_target_wp_num);

	int offset = startBinary(buf,BINARY_TYPE_BOAT);

	offset = binaryBool		(buf,offset,m_running);
	offset = binaryBool		(buf,offset,m_autopilot);
	offset = binaryBool		(buf,offset,m_routing);
	offset = binaryBool		(buf,offset,m_arrived);

	offset = binaryUint8	(buf,offset,m_start_wp_num);
	offset = binaryFixStr	(buf,offset,start_wp->name, MAX_WP_NAME);
	offset = binaryUint8	(buf,offset,m_target_wp_num);
	offset = binaryFixStr	(buf,offset,target_wp->name, MAX_WP_NAME);

	offset = binaryFloat	(buf,offset,m_depth);
	offset = binaryFloat	(buf,offset,m_sog);
	offset = binaryFloat	(buf,offset,m_cog);
	offset = binaryFloat	(buf,offset,m_wind_angle);
	offset = binaryFloat	(buf,offset,m_wind_speed);
	offset = binaryDouble	(buf,offset,m_latitude);
	offset = binaryDouble	(buf,offset,m_longitude);
	offset = binaryFloat	(buf,offset,m_app_wind_angle);
	offset = binaryFloat	(buf,offset,m_app_wind_speed);

	offset = binaryUint16	(buf,offset,m_rpm);
	offset = binaryUint16	(buf,offset,m_oil_pressure);
	offset = binaryUint16	(buf,offset,m_oil_temp);
	offset = binaryUint16	(buf,offset,m_coolant_temp);
	offset = binaryFloat	(buf,offset,m_alt_voltage);
	offset = binaryFloat	(buf,offset,m_fuel_rate);
	offset = binaryFloat	(buf,offset,m_fuel_level1);
	offset = binaryFloat	(buf,offset,m_fuel_level2);

	offset = binaryBool		(buf,offset,m_genset);
	offset = binaryUint16	(buf,offset,m_gen_rpm);
	offset = binaryUint16	(buf,offset,m_gen_oil_pressure);
	offset = binaryUint16	(buf,offset,m_gen_cool_temp);
	offset = binaryFloat	(buf,offset,m_gen_voltage);
	offset = binaryUint8	(buf,offset,m_gen_freq);

	offset = binaryUint32	(buf,offset,m_update_num);
	offset = binaryUint16	(buf,offset,m_closest);
	
	offset = binaryFloat	(buf,offset,headingToWaypoint());
	offset = binaryFloat  	(buf,offset,distanceToWaypoint());

	char timebuf[20];
	// 2025-09-14 12:13:14
	sprintf(timebuf,"%04d-%02d-%02d %02d:%02d:%02d",
		getYear(),
		getMonth(),
		getDay(),
		getHour(),
		getMinute(),
		getSecond());
	offset = binaryFixStr	(buf,offset,timebuf, 20);
	endBinary(buf,offset);

	// display(0,"sendBinaryBoatState(%d) finished",doit);

	Serial.write(buf,offset);
}


// end of boatSimulator.cpp
