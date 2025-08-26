//---------------------------------------------
// boatSimulator.cpp
//---------------------------------------------

#include "boatSimulator.h"
#include <myDebug.h>
#include <math.h>
#include "ge_routes.h"

#define dbg_sim	 -1

#define SIMULATION_INTERVAL		1000	// ms
#define CLOSEST_NONE			65535	// exact


boatSimulator boat;
	// global instance


static String showDeg(double coord)
{
	const char DEG_CHAR = 0xB0;
	
	double deg = round(coord);
	double mins = abs(coord - deg) * 60;
	char buf[20];
	sprintf(buf,"%d%c%0.3f",(int) deg, DEG_CHAR,mins);
	String rslt(buf);
	return rslt;
}


//----------------------------------------
// initialization
//----------------------------------------


void boatSimulator::start()
{
	display(0,"STARTING SIMULATOR",0);
	m_running = true;
}

void boatSimulator::stop()
{
	display(0,"STOPPING SIMULATOR",0);
	m_running = false;
}


void boatSimulator::init()
{
	// initial setting of m_route, m_waypoints, and m_num_waypoints
	// also sets m_waypoint num to 1,

	display(0,"INIT SIMULATOR",0);

	if (!m_inited)
		setRoute(routes[0].name);

	m_waypoint_num = 1;

	// state vars

	m_inited = true;
	m_running = false;
	m_autopilot = false;
	m_routing = false;
	m_arrived = false;

	// simulation vars

	m_depth = 10;
	m_sog = 0;
	m_cog = 180;
	m_wind_angle = 90;	// true east
	m_wind_speed = 12;
	m_rpms = 1000;

	// update vars

	m_update_num = 0;
	m_last_update_ms = 0;
	m_closest = CLOSEST_NONE;

	calculateApparentWind(false);
		// sets m_app_wind_angle and m_app_wind_speed;
		// with debugging
}


//-------------------------------------------
// setters
//-------------------------------------------

void boatSimulator::setRoute(const char *name)
{
	m_sog = 0;
	m_autopilot = false;
	m_routing = false;
	m_arrived = false;
	m_closest = CLOSEST_NONE;

	const route_t *found = 0;
	for (int i=0; i<NUM_ROUTES; i++)
	{
		const route_t *rte = &routes[i];
		if (String(name).equalsIgnoreCase(String(rte->name)))
		{
			found = rte;
			i = NUM_ROUTES;
		}
	}

	if (!found)
	{
		my_error("Could not find route(%s)",name);
		return;
	}

	m_waypoint_num = 1;
	m_waypoints = found->wpts;
	m_num_waypoints = found->num_wpts;
	display(0,"ROUTE(%s) num_waypoints=%d",found->name,m_num_waypoints);

	m_latitude = m_waypoints[0].lat;
	m_longitude = m_waypoints[0].lon;
}



void boatSimulator::setWaypointNum(int wp_num)
{
	if (wp_num < 0 || wp_num >= m_num_waypoints)
	{
		my_error("ILLEGAL WP_NUM(%d)",wp_num);
		return;
	}

	m_waypoint_num = wp_num;
	m_cog = headingToWaypoint();

	m_arrived = false;
	m_closest = CLOSEST_NONE;
	calculateApparentWind();

	const waypoint_t *wp = &m_waypoints[wp_num];

	display(0,"SET WAYPOINT[%d] %s %s %s cog(%d)",
		wp_num,
		wp->name,
		showDeg(wp->lat).c_str(),
		showDeg(wp->lon).c_str(),
		(int) m_cog);
}


void boatSimulator::jumpToWaypoint(int wp_num)
{
	if (wp_num < 0 || wp_num >= m_num_waypoints)
	{
		my_error("ILLEGAL JUMP WP_NUM(%d)",wp_num);
		return;
	}

	const waypoint_t *wp = &m_waypoints[wp_num];
	m_latitude = wp->lat;
	m_longitude = wp->lon;

	display(0,"JUMP TO WAYPOINT[%d] %s %s %s",
		wp_num,
		wp->name,
		showDeg(m_latitude).c_str(),
		showDeg(m_longitude).c_str());
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
	}
	else
		warning(0,"ROUTING ALREADY %s",(on?"ON":"OFF"));
}




//-----------------------------------------------
// implementation
//-----------------------------------------------

#define rad2deg(degrees)	((degrees) * M_PI / 180.0)
#define deg2rad(radians)	((radians) * (180.0 / M_PI))


bool boatSimulator::run()
	// Calculate and set new latitude and longitude
	// based on cog, sog, and millis() since last call
{
	if (!m_running)
		return false;
	
	proc_entry();

	if (!m_inited)
		init();

	uint32_t now = millis();
	if (now - m_last_update_ms < SIMULATION_INTERVAL)
	{
		proc_leave();
		return false;
	}

	display(dbg_sim,"boatSimulator::run lat(%s) lon(%s) cog(%d) sog(%d)",
		showDeg(m_latitude).c_str(),
		showDeg(m_longitude).c_str(),
		(int) m_cog,
		(int) m_sog);

	double elapsed_secs = (now - m_last_update_ms) / 1000.0;
	m_last_update_ms = now;
	if (m_sog == 0)
	{
		proc_leave();
		return true;
			// client should send out sensors
			// even though the boat has not moved
	}

	// set our new position

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
			showDeg(new_lat).c_str(),
			showDeg(new_lon).c_str());
	}
	m_latitude = new_lat;
	m_longitude = new_lon;


	// adjust our heading if we're routing to a waypoint

	if (m_routing && !m_arrived)
	{
		m_cog = headingToWaypoint();
		display(dbg_sim+1,"AP cog(%d)",(int) m_cog);
		calculateApparentWind(true);
	}

	// handle the autopilot

	if (m_autopilot)
	{
		const double NM_TO_FEET = 6076.12;
		uint32_t feet_to_wp = distanceToWaypoint() * NM_TO_FEET;
		bool arr = feet_to_wp < 400 ? 1 : 0;

		display(dbg_sim+1,"AP feet_to_wp(%d) arr(%d) m_arrived(%d) m_closest(%d)",
			feet_to_wp,
			arr,
			m_arrived,
			m_closest);
	
		// check for initial arrival

		if (arr && !m_arrived)
		{
			display(0,"INITIAL ARRIVAL",0);
			m_closest == feet_to_wp;
			m_arrived = true;
		}

		// check for end of arrival

		else if (m_arrived)
		{
			if (feet_to_wp <= m_closest)			// getting closeer
				m_closest = feet_to_wp;
			else  									// arrival finished
			{
				display(0,"ARRIVAL COMPLETE",0);
				if (m_routing)						// goto next waypoint or stop
				{
					if (m_waypoint_num < m_num_waypoints - 1)	// goto next waypoint
						setWaypointNum(m_waypoint_num+1);
					else
					{
						display(0,"ROUTE COMPLETE",0);
						setRouting(false);			// turn off routing
						m_sog = 0;					// stop the boat
					}
				}
				else
					setAutopilot(false);			// turn off autopilot after arrival
			}
		}	// already arrived
	}	// m_autpilot

	proc_leave();
	return true;

}	// run()



double boatSimulator::headingToWaypoint()
	// Returns heading in true degrees to given waypoint
	// from current latitute and longitude
{
	const waypoint_t *wp = &m_waypoints[m_waypoint_num];

	double delta_lon = wp->lon - m_longitude;
	double y = sin(rad2deg(delta_lon)) * cos(rad2deg(wp->lat));
	double x = cos(rad2deg(m_latitude)) * sin(rad2deg(wp->lat)) -
		sin(rad2deg(m_latitude)) * cos(rad2deg(wp->lat)) * cos(rad2deg(delta_lon));
	double heading_rad = atan2(y, x);
	double heading_deg = deg2rad(heading_rad);
	heading_deg = fmod((heading_deg + 360.0), 360.0);
	return heading_deg;
}


double boatSimulator::distanceToWaypoint()
	// Returns distance in NM to given waypoint
	// from current latitute and longitude
{
	const waypoint_t *wp = &m_waypoints[m_waypoint_num];

	const double EARTH_RADIUS_NM = 3440.065; 	// in nautical miles
    double d_lat = rad2deg(wp->lat - m_latitude);
    double d_lon = rad2deg(wp->lon - m_longitude);
    double a = sin(d_lat / 2) * sin(d_lat / 2) +
        cos(rad2deg(m_latitude)) * cos(rad2deg(wp->lat)) *
        sin(d_lon / 2) * sin(d_lon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    return EARTH_RADIUS_NM * c;
}


void boatSimulator::calculateApparentWind(bool quiet)
	// Calculates and sets app_wind_angle and app_wind_speed
	// based on cog, sog, wind_angle, and wind_epeed
	// as degrees relative to the bow of the boat
{
	#define dbg_wind 	(quiet?1:0)



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



// end of boatSimulator.cpp
