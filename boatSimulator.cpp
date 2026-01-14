//---------------------------------------------
// boatSimulator.cpp
//---------------------------------------------

#include "boatSimulator.h"
#include "instSimulator.h"
#include <myDebug.h>
#include <math.h>
#include "boatUtils.h"
#include "boatBinary.h"
#include <TimeLib.h>

#define dbg_sim	 (1 - boat_sim.g_MON_SIM)
#define dbg_ap  0

#define SIMULATION_INTERVAL		1000	// ms
#define CLOSEST_NONE			65535	// exact


boatSimulator boat_sim;
int boatSimulator::g_MON_SIM = 0;
	// global instance


#define deg2rad(degrees)	((degrees) * M_PI / 180.0)
#define rad2deg(radians)	((radians) * (180.0 / M_PI))



//----------------------------------------
// initialization
//----------------------------------------


void boatSimulator::start()
{
	display(0,"STARTING SIMULATOR",0);
	m_running = true;
	sendBinarySimState(1);
}

void boatSimulator::stop()
{
	display(0,"STOPPING SIMULATOR",0);
	m_running = false;
	sendBinarySimState(1);
}


void boatSimulator::init()
{
	// initial setting of m_route, m_waypoints, and m_num_waypoints
	// also sets m_waypoint num to 1,

	display(0,"boatSimulator::init() started",0);
	proc_entry();

	if (!m_inited)
	{
		m_route_name = "";
		setRoute(simulator_routes[0].name);
		m_trip_on  = 1;
		m_trip_distance = 0;
	}


	// state vars

	m_inited 	= true;
	// m_running 	= false;
	m_autopilot = AP_MODE_OFF;
	m_routing 	= false;
	m_arrived 	= false;

	// simulation inputs

	m_start_wp_num = 0;
	m_target_wp_num = 1;
	m_latitude = m_waypoints[0].lat;
	m_longitude = m_waypoints[0].lon;
	m_desired_heading = 90;	// 0;
	m_rudder = 0;
	
	m_depth 	 	= 10;
	m_heading		= 180;
	m_water_speed   = 0;
	m_wind_angle 	= 90;			// from true east
	m_wind_speed 	= 12;
	m_current_set 	= 45;			// to north east
	m_current_drift = 0;

	// calculated variabls

	m_cog	= 0;
	m_sog	= 0;
	m_app_wind_angle = 0;
	m_app_wind_speed = 0;

	// autopilot

	m_estimated_set 	= 0;
	m_estimated_drift 	= 0;
	m_track_error 		= 0;

	// artificial

	m_rpm 		 		= 0;
	m_boost_pressure	= 0;
	m_oil_pressure 		= 0;
	m_oil_temp  		= 0;
	m_coolant_temp 		= 0;
	m_alt_voltage 		= 0;
	m_fuel_rate 		= 0;
	m_fuel_level1 		= 0.48;
	m_fuel_level2 		= 0.52;

	m_genset 			= 1;	// false;
	m_gen_rpm 			= 0;
	m_gen_oil_pressure	= 0;
	m_gen_cool_temp		= 0;
	m_gen_voltage 		= 0;
	m_gen_freq 			= 0;

	// update vars

	m_update_num = 0;
	m_last_update_ms = 0;
	m_closest = CLOSEST_NONE;

	calculate(false);
	sendBinarySimState(1);

	proc_leave();
	display(0,"boatSimulator::init() finished",0);
}


//-------------------------------------------
// setters
//-------------------------------------------

void boatSimulator::setDepth(float depth)
{
	m_depth = depth;
	sendBinarySimState(!m_running);
	if (depth<0)
	{
		my_error("Depth(%0.1f) must be >= zero",depth);
		return;
	}
}

void boatSimulator::setHeading(float heading)
{
	if (heading<0 || heading>360)
	{
		my_error("Heading(%0.1f) must be between 0 and 360",heading);
		return;
	}
	m_heading = heading;
	calculate();
	sendBinarySimState(!m_running);
}

void boatSimulator::setWaterSpeed(float speed)
{
	if (speed<0)
	{
		my_error("WaterSpeed(%0.1f) must be >= zero",speed);
		return;
	}
	m_water_speed = speed;
	calculate();
	if (speed == 0)
	{
		m_rpm = 0;
	}
	else
	{
		float new_rpm = 100 + (m_water_speed / 10.0) * 900;  // approx 900 rpm per 10 knots
		m_rpm = new_rpm;
	}
	sendBinarySimState(!m_running);
}

void boatSimulator::setCurrentSet(float angle)
{
	if (angle<0 || angle>360)
	{
		my_error("Set(%0.1f) must be between 0 and 360",angle);
		return;
	}
	m_current_set = angle;
	calculate();
	sendBinarySimState(!m_running);
}

void boatSimulator::setCurrentDrift(float speed)
{
	if (speed<0)
	{
		my_error("Drift(%0.1f) must be >= zero",speed);
		return;
	}
	m_current_drift = speed;
	calculate();
	sendBinarySimState(!m_running);
}

void boatSimulator::setWindAngle(float angle)
{
	if (angle<0 || angle>360)
	{
		my_error("WindAngle(%0.1f) must be between 0 and 360",angle);
		return;
	}
	m_wind_angle = angle;
	calculateApparentWind();
	sendBinarySimState(!m_running);
}

void boatSimulator::setWindSpeed(float speed)
{
	if (speed<0)
	{
		my_error("WindSpeed(%0.1f) must be >= zero",speed);
		return;
	}
	m_wind_speed = speed;
	calculateApparentWind();
	sendBinarySimState(!m_running);
}

void boatSimulator::setRPM(uint16_t rpm)
{
	m_rpm = rpm;
	sendBinarySimState(!m_running);
}

void boatSimulator::setGenset(bool on)
{
	m_genset = on;
	display(0,"GENSET %s",m_genset?"ON":"OFF");
	sendBinarySimState(!m_running);
}

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
	m_autopilot = AP_MODE_OFF;
	m_routing = false;
	m_arrived = false;
	m_closest = CLOSEST_NONE;

	const route_t *found = getRoute(name);
	if (!found) return;
	m_route_name = name;

	m_start_wp_num = 0;
	m_target_wp_num = 1;
	m_waypoints = found->wpts;
	m_num_waypoints = found->num_wpts;
	display(0,"ROUTE(%s) num_waypoints=%d",found->name,m_num_waypoints);

	m_latitude = m_waypoints[0].lat;
	m_longitude = m_waypoints[0].lon;

	sendBinarySimState(m_inited && !m_running);
}

void boatSimulator::setTargetWPNum(uint8_t wp_num)
{
	if (wp_num < 0 || wp_num >= m_num_waypoints)
	{
		my_error("ILLEGAL WP_NUM(%d)",wp_num);
		return;
	}

	m_target_wp_num = wp_num;
	m_heading = headingToWaypoint();
	m_arrived = false;
	m_closest = CLOSEST_NONE;
	calculate();

	const waypoint_t *wp = &m_waypoints[wp_num];

	display(0,"SET TARGET WP[%d] %s %s %s heading(%d)",
		wp_num,
		wp->name,
		strDegreeMinutes(wp->lat).c_str(),
		strDegreeMinutes(wp->lon).c_str(),
		(int) m_heading);

	sendBinarySimState(!m_running);
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

	sendBinarySimState(!m_running);
}

void boatSimulator::setAutopilot(uint8_t mode)
{
	const char *ap_mode =
		mode == 2 ? "VANE" :
		mode == 1 ? "AUTO" :
		"OFF";

	if (m_autopilot != mode)
	{
		m_arrived = false;
		m_closest = CLOSEST_NONE;
		display(0,"AUTOPILOT %s",ap_mode);
		m_autopilot = mode;

		// temoporarily implementaiton: VANE mode == AUTO mode

		if (mode)
			m_heading = headingToWaypoint();
		else if (m_routing)
			setRouting(false);

		sendBinarySimState(!m_running);
	}
	else
		warning(0,"AUTOPILOT ALREADY %s",ap_mode);
}


void boatSimulator::setRouting(bool on)
{
	if (m_routing != on)
	{
		display(0,"ROUTING %s",(on?"ON":"OFF"));
		m_routing = on;

		if (on && m_autopilot != AP_MODE_AUTO)
			setAutopilot(AP_MODE_AUTO);	// AUTO

		sendBinarySimState(!m_running);
	}
	else
		warning(0,"ROUTING ALREADY %s",(on?"ON":"OFF"));
}


//-----------------------------------------------
// implementation
//-----------------------------------------------

static void useFuel(float *used, float *level, float capacity)
{
	float cur = *level * capacity;
	if (*used > cur)
	{
		(*used) -= cur;
		*level = 0;
	}
	else
	{
		cur -= *used;
		*used = 0;
		*level = cur / capacity;
	}
}



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
	
	display(dbg_sim,"boatSimulator::run lat(%s) lon(%s) heading(%d) s_water(%d)",
		strDegreeMinutes(m_latitude).c_str(),
		strDegreeMinutes(m_longitude).c_str(),
		(int) m_heading,
		(int) m_water_speed);
	proc_entry();
	display(dbg_sim+1,"year(%d) month(%d) day(%d) hour(%d) minute(%d) second(%d)",
		getYear(),
		getMonth(),
		getDay(),
		getHour(),
		getMinute(),
		getSecond());

	calculate(1);
	
	// integrate the trip distance

	if (m_trip_on)
	{
		double delta_nm = m_sog * (elapsed_secs / 3600.0);  // NM
		m_trip_distance += delta_nm;
	}

	// set our new position

	if (m_sog != 0)
	{
		const double EARTH_RADIUS = 6371000; 	// in meters
		const double KNOTS_TO_MPS = 0.514444;
		double distance_m = m_sog * KNOTS_TO_MPS * elapsed_secs;
		double cog_rad = deg2rad(m_cog);
		double delta_lat = (distance_m * cos(cog_rad)) / EARTH_RADIUS;
		double delta_lon = (distance_m * sin(cog_rad)) / (EARTH_RADIUS * cos(deg2rad(m_latitude)));
		double new_lat = m_latitude + rad2deg(delta_lat);
		double new_lon = m_longitude + rad2deg(delta_lon);

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

	// display m_heading and calculate() if it has changed

	static int last_heading = 0;
	if (last_heading != (int) m_heading)
	{
		last_heading = (int) m_heading;
		display(dbg_sim+1,"new heading(%d)",(int) m_heading);
		calculate(1);
	}

	// handle the autopilot

	if (m_autopilot)
		doAutopilot();

	// set psudeo random engine and genset values

	if (1)
	{
		#define ACCELERATED_USE   10

		m_oil_pressure = 	m_rpm == 0 ? 0 : 5    + (((float)m_rpm) / 900) * 20.0; 	// psi
		m_boost_pressure = 	m_rpm == 0 ? 0 : 0    + (((float)m_rpm) / 900) * 10.0; 	// psi
		m_oil_temp = 		m_rpm == 0 ? 0 : 120  + (((float)m_rpm) / 900) * 20.0;   // farenheight
		m_coolant_temp =	m_rpm == 0 ? 0 : 90   + (((float)m_rpm) / 900) * 20.0;  	// farenheight
		m_alt_voltage = 	m_rpm == 0 ? 0 : 10.0 + (((float)m_rpm) / 900) * 1.0;
		m_fuel_rate = 		m_rpm == 0 ? 0 : 0.1  + (((float)m_rpm) / 900) * 1.0;  	// gph

		if (m_rpm && ACCELERATED_USE)
			m_fuel_rate *= ACCELERATED_USE;

		if (m_rpm)	// use tank1 first
		{
			float used = (elapsed_secs / 3600) * m_fuel_rate;
			if (m_fuel_level1 > 0)
				useFuel(&used, &m_fuel_level1, TANK1_CAPACITY);
			if (used > 0 && m_fuel_level2 > 0)
				useFuel(&used, &m_fuel_level2, TANK2_CAPACITY);
		}

		m_gen_rpm = 		m_genset ? 3600 + random(-100,100) : 0;
		m_gen_oil_pressure=	m_genset ? 50 + random(-30,30) : 0; 	// psi
		m_gen_cool_temp	=	m_genset ? 180 + random(-40,40) : 0; 	// farenheight
		m_gen_voltage = 	m_genset ? 120 + random(-10,10) : 0;
		m_gen_freq =		m_genset ? 60 + random(-5,5) : 0;
	}
	else	// fixed values to help in debugging
	{
		static int counter;
		counter++;

		int level1 = (counter/1) % 100;
		int level2 = (counter/2) % 100;

		m_oil_pressure = 	m_rpm == 0 ? 0 : 50.7; 		// psi
		m_oil_temp = 		m_rpm == 0 ? 0 : 180.3;   	// farenheight
		m_coolant_temp =	m_rpm == 0 ? 0 : 165.2;   	// farenheight
		m_alt_voltage = 	m_rpm == 0 ? 0 : 12.34;
		m_fuel_rate = 		m_rpm == 0 ? 0 : 1.123;  	// gph
		m_fuel_level1 = 	m_rpm == 0 ? ((float) level1) / 100.0 : 0.33; 	// 0..1
		m_fuel_level2 = 	m_rpm == 0 ? ((float) level2) / 100.0 : 0.66; 	// 0..1
		m_gen_rpm = 		m_genset ? 3600 : 0;
		m_gen_oil_pressure=	m_genset ? 50 : 0; 			// psi
		m_gen_cool_temp	=	m_genset ? 182 : 0; 		// farenheight
		m_gen_voltage = 	m_genset ? 13.45 : 0;
		m_gen_freq =		m_genset ? 60: 0;
	}
	
	sendBinarySimState(1);

	proc_leave();
	

}	// run()



//------------------------------------------------------------------------------------
// autoPilot
//------------------------------------------------------------------------------------


void boatSimulator::doAutopilot()
{
	if (m_routing)
		m_desired_heading = headingToWaypoint();
		
	display(dbg_ap+1,"AP routing(%d) desired_heading(%0.1f)",
		m_routing, m_desired_heading);
	proc_entry();

	// check arrival state, possibly change to new m_desired_heading

	if (m_routing)
	{
		const float NM_TO_FEET = 6076.12;
		uint32_t feet_to_wp = distanceToWaypoint() * NM_TO_FEET;
		bool arr = feet_to_wp < 400 ? 1 : 0;
		if (feet_to_wp < ((uint32_t) m_closest))
			m_closest = feet_to_wp;

		display(dbg_ap+1,"ROUTING feet_to_wp(%d) arr(%d) m_arrived(%d) m_closest(%d)",
			feet_to_wp,
			arr,
			m_arrived,
			m_closest);
		proc_entry();

		// check for initial arrival

		if (arr && !m_arrived)
		{
			display(dbg_ap,"INITIAL ARRIVAL",0);
			m_arrived = true;
		}

		// check for end of arrival

		else if (m_arrived)
		{
			if (feet_to_wp <= m_closest)							// getting closer
				m_closest = feet_to_wp;
			else                                                    // arrival finished
			{
				display(dbg_ap,"ARRIVAL COMPLETE",0);
				if (m_target_wp_num < m_num_waypoints - 1)		// goto next waypoint
				{
					m_start_wp_num = m_target_wp_num;
					m_target_wp_num++;
					m_arrived = false;
					m_closest = CLOSEST_NONE;
					m_desired_heading = headingToWaypoint();
					display(dbg_ap+1,"new target_wp_num($%d) desired_heading(%0.1f)",
						m_target_wp_num,m_desired_heading);
				}
				else
				{
					display(dbg_ap,"ROUTE COMPLETE",0);
					m_water_speed = 0;                          // stop the boat
					m_rpm = 0;
					m_routing = false;                          // turn off routing
					m_autopilot = AP_MODE_OFF;					// turn off autopilot
					m_arrived = false;
					m_desired_heading = 0;
					calculate(1);
				}
			}
		}

		// calculate XTE

		calcuateCrossTrackError();
		proc_leave();
	}


	//--------------------------------------------------------
	// estimate current set and drift from boat motion
	//--------------------------------------------------------

	double heading_rad = deg2rad(90.0 - m_heading);
	double cog_rad     = deg2rad(90.0 - m_cog);

	double boat_vx = m_water_speed * cos(heading_rad);
	double boat_vy = m_water_speed * sin(heading_rad);
	double sog_vx = m_sog * cos(cog_rad);
	double sog_vy = m_sog * sin(cog_rad);

	double current_vx = sog_vx - boat_vx;
	double current_vy = sog_vy - boat_vy;

	// double estimated_set = fmod(rad2deg(atan2(current_vy, current_vx)) - 90, 360);
	double estimated_set = fmod(450 - rad2deg(atan2(current_vy, current_vx)), 360);
	double estimated_drift = sqrt(current_vx * current_vx + current_vy * current_vy);

	display(dbg_ap+2,"m_heading(%0.1f) rad(%0.3f)  cog(%0.1f) rad(%0.3f)  water_speed(%0.3f)",
		m_heading,
		heading_rad,
		m_cog,
		m_cog,
		m_water_speed);
	proc_entry();

	display(dbg_ap+2,"boat_vx(%0.3f) vy(%0.3f)   sog_vx(%0.3f) vy(%0.3f)   current_vx(%0.3f) vy(%0.3f)",
		boat_vx,
		boat_vy,
		sog_vx,
		sog_vy,
		current_vx,
		current_vy);

	display(dbg_ap+2,"local (relative?) estimated_set(%0.3f) estimated_drift(%0.3f)",
		estimated_set,
		estimated_drift);


	// low-pass filter to smooth it

	double learn_rate = 0.1;
		// will 'learn' current changes in learn_rate/1 of the current it sees
		// or will learn the current in 10 seconds

	m_estimated_set = (1.0 - learn_rate) * m_estimated_set + learn_rate * estimated_set;
	m_estimated_drift = (1.0 - learn_rate) * m_estimated_drift + learn_rate * estimated_drift;

	display(dbg_ap+2,"learn_rate(%0.3f) m_estimated_set(%0.3f) m_estimated_drift(%0.3f)",
		learn_rate,
		m_estimated_set,
		m_estimated_drift);


	//--------------------------------------------------------
	// autopilot heading calculations
	//--------------------------------------------------------
	// with pid-like easing to the new heading

	double turn_rate = 0.2;
		// turn_rate is how fast the autopilot moves the rudder.
		// In this case 20% of the difference between the current_heading
		// and the compensated heading every second.  It moves
		// the rudder over about a 5-7 second period.

	double heading_error = m_desired_heading - m_cog;
	if (heading_error > 180) heading_error -= 360;
	if (heading_error < -180) heading_error += 360;

	double adjustment = turn_rate * heading_error;
	if (adjustment > 10.0) adjustment = 10.0;
	if (adjustment < -10.0) adjustment = -10.0;

	display(dbg_ap+2,"heading_error(%0.3f) = m_desired_heading(%0.3f) - m_cog(%0.3f)",
		heading_error,
		m_desired_heading,
		m_cog);

	display(dbg_ap+2,"adjustment(%0.3f) = turn_rate(%0.3f) * heading_error(%0.3f)",
		adjustment,
		turn_rate,
		heading_error);

	m_heading += adjustment;
	if (m_heading < 0) m_heading += 360;
	if (m_heading >= 360) m_heading -= 360;

	display(dbg_ap+2,"FINAL m_heading(%0.3f) = previous m_heading + adjustment",
		m_heading);
	proc_leave();

	// static int last_heading = -1;
	// if ((int)m_heading != last_heading)
	// {
	// 	last_heading = (int)m_heading;
	// 	display(dbg_ap,"AP adjusted heading(%d)", (int)m_heading);
	// 	calculate(1);
	// }
	proc_leave();
}



void boatSimulator::calcuateCrossTrackError()
{

	if (0)
	{
		static float xte = 3.0;
		xte -= 0.01;
		if (xte <= 0) xte = 3.0;
		display(0,"xte=%0.3f",xte);
		m_track_error = xte;
		return;
	}

	// calculate track error from current position to line between start and target waypoint

	const waypoint_t *start_wp = &m_waypoints[m_start_wp_num];
	const waypoint_t *target_wp = &m_waypoints[m_target_wp_num];

	double lat1 = start_wp->lat;
	double lon1 = start_wp->lon;
	double lat2 = target_wp->lat;
	double lon2 = target_wp->lon;
	double lat3 = m_latitude;
	double lon3 = m_longitude;

	double x13 = (lon3 - lon1) * cos(deg2rad((lat1 + lat3) / 2.0));
	double y13 = lat3 - lat1;
	double x12 = (lon2 - lon1) * cos(deg2rad((lat1 + lat2) / 2.0));
	double y12 = lat2 - lat1;

	double dot = x13 * x12 + y13 * y12;
	double len_sq = x12 * x12 + y12 * y12;
	double proj = dot / len_sq;

	double x_proj = proj * x12;
	double y_proj = proj * y12;

	double x_err = x13 - x_proj;
	double y_err = y13 - y_proj;

	double err_nm = sqrt(x_err * x_err + y_err * y_err) * 60.0;
	m_track_error = err_nm;
}



//---------------------------------------------------
// lower level calculations
//---------------------------------------------------


float boatSimulator::headingTo(float lat, float lon, const waypoint_t *wp)
{
	double delta_lon = wp->lon - lon;
	double y = sin(deg2rad(delta_lon)) * cos(deg2rad(wp->lat));
	double x = cos(deg2rad(lat)) * sin(deg2rad(wp->lat)) -
		sin(deg2rad(lat)) * cos(deg2rad(wp->lat)) * cos(deg2rad(delta_lon));
	double heading_rad = atan2(y, x);
	double heading_deg = rad2deg(heading_rad);
	heading_deg = fmod((heading_deg + 360.0), 360.0);
	return (float) heading_deg;
}

	
float boatSimulator::distanceTo(float lat, float lon, const waypoint_t *wp)
{
	const double EARTH_RADIUS_NM = 3440.065; 	// in nautical miles
    double d_lat = deg2rad(wp->lat - lat);
    double d_lon = deg2rad(wp->lon - lon);
    double a = sin(d_lat / 2) * sin(d_lat / 2) +
        cos(deg2rad(lat)) * cos(deg2rad(wp->lat)) *
        sin(d_lon / 2) * sin(d_lon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    return (float) (EARTH_RADIUS_NM * c);
}


float boatSimulator::headingToWaypoint()
	// Returns heading in true degrees to given waypoint
	// from current latitute and longitude
{
	const waypoint_t *wp = &m_waypoints[m_target_wp_num];
	return headingTo(m_latitude,m_longitude,wp);
}


float boatSimulator::distanceToWaypoint()
	// Returns distance in NM to given waypoint
	// from current latitute and longitude
{
	const waypoint_t *wp = &m_waypoints[m_target_wp_num];
	return distanceTo(m_latitude,m_longitude,wp);
}


void boatSimulator::calculateApparentWind(bool quiet /*=true*/)
	// Calculates and sets app_wind_angle and app_wind_speed
	// based on cog, sog, wind_angle, and wind_epeed
	// as degrees relative to the bow of the boat
{
	#define dbg_wind (dbg_sim + (quiet?2:0))

	display(dbg_wind,"calculateApparentWind speed/angle boat(%0.3f,%0.3f) wind(%0.3f,%0.3f)",
		m_sog,m_cog,m_wind_speed,m_wind_angle);

	double wx = m_wind_speed * sin(deg2rad(m_wind_angle));
	double wy = m_wind_speed * cos(deg2rad(m_wind_angle));
	double bx = m_sog * sin(deg2rad(m_cog));
	double by = m_sog * cos(deg2rad(m_cog));
	double ax = wx + bx;
	double ay = wy + by;

	proc_entry();
	display(dbg_wind+1,"wx(%0.3f) wy(%0.3f) bx(%0.3f) by(%0.3f) ax(%0.3f) ay(%0.3f)", wx,wy,bx,by,ax,ay);

	m_app_wind_speed = sqrt(ax * ax + ay * ay);
	double awa_global = fmod(rad2deg(atan2(ax, ay)) + 360.0, 360.0);
	m_app_wind_angle = fmod(awa_global - m_heading + 360.0, 360.0);

	display(dbg_wind + 1, "app_wind_speed(%0.3f) absolute angle(%0.3f)", m_app_wind_speed, awa_global);
	display(dbg_wind + 1, "relative angle(%0.3f)", m_app_wind_angle);
	display(dbg_wind, "app_wind_speed(%0.3f) app_wind_angle(%0.3f)", m_app_wind_speed, m_app_wind_angle);

	proc_leave();
}




void boatSimulator::calculateOverGround(bool quiet /*=true*/)
	// Calculates and sets app_wind_angle and app_wind_speed
	// based on cog, sog, wind_angle, and wind_epeed
	// as degrees relative to the bow of the boat
{
	#define dbg_ground (dbg_sim + (quiet?2:0))

	display(dbg_ground,"calculateOverGround heading(%0.3f) water_speed(%0.3f) current(%0.3f,%0.3f)",
		m_heading,m_water_speed,m_current_drift,m_current_set);
	proc_entry();

	double use_current_angle = fmod(m_current_set + 180, 360);
		// this routine, mostly written by coPilot, expects the
		// direction the water is coming FROM, but Set in mariner's
		// terms is the direction it's going TO, so we simply change it
		// from Set(m_current_set) to From(use_current_angle),
		// cuz I don't want to mess with the calcs below.

	// Boat vector
	double bx = m_water_speed * sin(deg2rad(m_heading));
	double by = m_water_speed * cos(deg2rad(m_heading));

	// Current vector (reverse direction)
	double cx = m_current_drift * sin(deg2rad(use_current_angle + 180));
	double cy = m_current_drift * cos(deg2rad(use_current_angle + 180));

	// Sum vectors
	double vx = bx + cx;
	double vy = by + cy;

	// Resultant vector
	m_sog = sqrt(vx*vx + vy*vy);
	m_cog = fmod(rad2deg(atan2(vx, vy)) + 360.0, 360.0);


	display(dbg_ground+1,"bx(%0.3f) by(%0.3f) cx(%0.3f) cy(%0.3f) vx(%0.3f) vy(%0.3f)", bx,by,cx,cy,vx,vy);
	display(dbg_ground,"returning sog(%0.3f) cog(%0.3f)",m_sog,m_cog);
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

void boatSimulator::sendBinarySimState(bool doit /*=1*/)
{
	if (!(g_BINARY & BINARY_TYPE_SIM)) return;
	if (!doit) return;

	// display(0,"sendBinarySimState(%d)",doit);

	uint8_t buf[sizeof(boatSimulator) + BINARY_HEADER_LEN + 16+1 + 2 * (MAX_WP_NAME + 1) + DATE_SIZE + 2];		// guaranteed to be big enough

	const waypoint_t *start_wp = getWaypoint(m_start_wp_num);
	const waypoint_t *target_wp = getWaypoint(m_target_wp_num);

	int offset = startBinary(buf,BINARY_TYPE_SIM);

	offset = binaryBool		(buf,offset,m_running);
	offset = binaryUint8	(buf,offset,m_autopilot);
	offset = binaryBool		(buf,offset,m_routing);
	offset = binaryBool		(buf,offset,m_arrived);
	offset = binaryFixStr	(buf,offset,m_route_name, 16);

	offset = binaryBool		(buf,offset,m_trip_on);
	offset = binaryFloat	(buf,offset,m_trip_distance);
	offset = binaryFloat	(buf,offset,getLogTotal());

	offset = binaryUint8	(buf,offset,m_start_wp_num);
	offset = binaryFixStr	(buf,offset,start_wp->name, MAX_WP_NAME);
	offset = binaryUint8	(buf,offset,m_target_wp_num);
	offset = binaryFixStr	(buf,offset,target_wp->name, MAX_WP_NAME);
	offset = binaryFloat	(buf,offset,headingToWaypoint());
	offset = binaryFloat  	(buf,offset,distanceToWaypoint());

	offset = binaryFloat	(buf,offset,m_desired_heading);
	offset = binaryFloat	(buf,offset,m_rudder);
	
	offset = binaryFloat	(buf,offset,m_depth);
	offset = binaryFloat	(buf,offset,m_heading);
	offset = binaryFloat	(buf,offset,m_water_speed);
	offset = binaryFloat	(buf,offset,m_current_set);
	offset = binaryFloat	(buf,offset,m_current_drift);
	offset = binaryFloat	(buf,offset,m_wind_angle);
	offset = binaryFloat	(buf,offset,m_wind_speed);
	offset = binaryDouble	(buf,offset,m_latitude);
	offset = binaryDouble	(buf,offset,m_longitude);

	offset = binaryFloat	(buf,offset,m_sog);
	offset = binaryFloat	(buf,offset,m_cog);
	offset = binaryFloat	(buf,offset,m_app_wind_angle);
	offset = binaryFloat	(buf,offset,m_app_wind_speed);
	offset = binaryFloat	(buf,offset,m_estimated_set);
	offset = binaryFloat	(buf,offset,m_estimated_drift);
	offset = binaryFloat	(buf,offset,m_track_error);
	offset = binaryUint16	(buf,offset,m_closest);
	
	offset = binaryUint16	(buf,offset,m_rpm);
	offset = binaryFloat    (buf,offset,m_boost_pressure);
	offset = binaryFloat    (buf,offset,m_oil_pressure);
	offset = binaryFloat    (buf,offset,m_oil_temp);
	offset = binaryFloat    (buf,offset,m_coolant_temp);
	offset = binaryFloat	(buf,offset,m_alt_voltage);
	offset = binaryFloat	(buf,offset,m_fuel_rate);
	offset = binaryFloat	(buf,offset,m_fuel_level1);
	offset = binaryFloat	(buf,offset,m_fuel_level2);

	offset = binaryBool		(buf,offset,m_genset);
	offset = binaryFloat	(buf,offset,m_gen_rpm);
	offset = binaryFloat	(buf,offset,m_gen_oil_pressure);
	offset = binaryFloat	(buf,offset,m_gen_cool_temp);
	offset = binaryFloat	(buf,offset,m_gen_voltage);
	offset = binaryUint8	(buf,offset,m_gen_freq);

	offset = binaryUint32	(buf,offset,m_update_num);
	
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

	Serial.write(buf,offset);
	#ifdef SERIAL_ESP32
		if (udp_enabled)
			SERIAL_ESP32.write(buf,offset);
	#endif
}


// end of boatSimulator.cpp
