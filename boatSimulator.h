//---------------------------------------------
// boatSimulator.h
//---------------------------------------------
// A generic platform independent Arduino library that provides
// a simple simulation of a sailing boat.  Used to generate data for
// testing various hardware interfaces including my Seatalk, NMEA1083
// and NMEA2000 hardware projects.

#pragma once
#include <Arduino.h>


typedef struct
{
	const char *name;
	float lat;
	float lon;
} waypoint_t;


typedef struct
{
	const char *name;
	const waypoint_t *wpts;
	int	num_wpts;
} route_t;




class boatSimulator {

public:

	// functional api

	void init();		// stop the simulator and re-initialize
	void start();		// start the simulator
	void stop();		// stop the simulator
	bool run();			// run one time slice of the simulator (approx 1 second)
						// has it's own 1 second timer; returns true if it happened
	// getters

	bool   running()		{ return m_running; }

	double getDepth()		{ return m_depth; }			// feet below surface
	double getSOG()			{ return m_sog; }			// knots
	double getCOG()			{ return m_cog; }			// true
	double getWindAngle() 	{ return m_wind_angle; }	// true
	double getWindSpeed() 	{ return m_wind_speed; }	// knots
	double getLat()			{ return m_latitude; }
	double getLon()			{ return m_longitude; }

	double apparentWindAngle()	{ return m_app_wind_angle; }	// degrees relative to the bow of the boat
	double apparentWindSpeed()	{ return m_app_wind_speed; }	// knots

	double getRPMS()			{ return m_rpms; }
	double getOilPressure()		{ return m_rpms == 0 ? 0 : 50 + random(-30,30); }	// psi
	double getAltVoltage()		{ return m_rpms == 0 ? 0 : 12.0 + (((float)random(-300,300)) / 100.0); }
	double getCoolantTemp()		{ return m_rpms == 0 ? 0 : 180 + random(-40,40); }  // farenheight
	double getFuelRate()		{ return m_rpms == 0 ? 0 : 1.5 + (((float) random(-100,100)) / 100.0); } // gph

	int getNumWaypoints()	{ return m_num_waypoints; }		// in current route
	int getWaypointNum() 	{ return m_waypoint_num; }		// return the current waypoint to go to
	const waypoint_t *getWaypoint(int wp_num)
	{
		if (wp_num > 0 && wp_num < m_num_waypoints)
			return &m_waypoints[wp_num];
	}

	bool getAutopilot()		{ return m_autopilot; }
	bool getRouting()		{ return m_routing; }
	bool getArrived()		{ return m_arrived; }
		// get the autopilot, routing, and arrival status

	double headingToWaypoint();		// true
	double distanceToWaypoint();	// NM

	// setters

	void setDepth			(double depth)		{ m_depth = depth; }
	void setSOG				(double sog)		{ m_sog = sog; calculateApparentWind(); }
	void setCOG				(double cog)		{ m_cog = cog; calculateApparentWind(); }
	double setWindAngle		(double angle) 		{ m_wind_angle = angle; calculateApparentWind(); }
	double setWindSpeed 	(double speed)		{ m_wind_speed = speed; calculateApparentWind(); }
	double setRPMS			(double rpms)		{ m_rpms = rpms; }

	void setRoute(const char *route_name);
		// see ge_routes.h for names
		// stops the boat, turns off autopilot and routing
		// 		but does not stop the simulator
		// puts the boat at the 0th waypoint and sets the
		// 		current waypoint number to 1
	void setWaypointNum(int wp_num);
		// sets the waypoint to navigate to
		// constrained 0 to getNumWaypoints()-1
		// sets a heading to the waypoint and resets m_arrived and m_closest
	void jumpToWaypoint(int wp_num);
		// moves the boat to the given waypoint
		// without otherwise altering the simulator
		// there *really* should be a "last" and "next" waypoint


	void setAutopilot(bool on);
		// automatically sets heading to the next waypoint
		// on each time slice. Starts watching for arrival,
	void setRouting(bool on);
		// will automatically advance to the next waypoint
		// after an arrival and the boat starts getting
		// farther away from a waypoint.  Will stop the
		// boat upon arrival at the final waypoint.

private:

	bool m_inited;
	bool m_running;
	bool m_autopilot;
	bool m_routing;
	bool m_arrived;

	double m_depth;
	double m_sog;
	double m_cog;
	double m_wind_angle;
	double m_wind_speed;
	double m_latitude;
	double m_longitude;
	double m_app_wind_angle;
	double m_app_wind_speed;
	double m_rpms;

	int m_waypoint_num;
	int m_num_waypoints;
	const waypoint_t *m_waypoints;

	// implementation

	uint32_t m_update_num;
	uint32_t m_last_update_ms;		// ms since last update
	uint16_t m_closest;				// integer feet

	void calculateApparentWind(bool quiet=true);

};	// class Simulator


extern boatSimulator boat;
	// static instance in simulator.cpp



// end of simulator.h
