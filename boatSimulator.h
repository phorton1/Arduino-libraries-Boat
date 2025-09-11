//---------------------------------------------
// boatSimulator.h
//---------------------------------------------
// A generic platform independent Arduino library that provides
// a simple simulation of a sailing boat.  Used to generate data for
// testing various hardware interfaces including my Seatalk, NMEA1083
// and NMEA2000 hardware projects.

#pragma once
#include <Arduino.h>
#include <myDebug.h>


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


extern const route_t *simulator_routes __attribute__((weak));
extern const int simulator_num_routes __attribute__((weak));
	// Note that these are declared as weakly linked so that
	// they can be overriden by your application as desired.



	
class boatSimulator {

public:

	void init();			// stop the simulator and re-initialize
	void start();			// start the simulator (sets m_running=1)
	void stop();			// stop the simulator (sets m_running=0)
	void run();				// run one time slice of the simulator
		// call once per second or so

	// getters

	bool   running()			{ return m_running; }		// true while simulator is running

	double getDepth()			{ return m_depth; }			// feet below surface
	double getSOG()				{ return m_sog; }			// knots
	double getCOG()				{ return m_cog; }			// true
	double getWindAngle() 		{ return m_wind_angle; }	// true
	double getWindSpeed() 		{ return m_wind_speed; }	// knots
	double getLat()				{ return m_latitude; }
	double getLon()				{ return m_longitude; }
	double getRPMS()			{ return m_rpms; }
	double getOilPressure()		{ return m_rpms == 0 ? 0 : 50 + random(-30,30); }	// psi
	double getOilTemp()			{ return m_rpms == 0 ? 0 : 180 + random(-40,40); }  // farenheight
	double getCoolantTemp()		{ return m_rpms == 0 ? 0 : 180 + random(-40,40); }  // farenheight
	double getAltVoltage()		{ return m_rpms == 0 ? 0 : 12.0 + (((float)random(-300,300)) / 100.0); }
	double getFuelRate()		{ return m_rpms == 0 ? 0 : 1.5 + (((float) random(-100,100)) / 100.0); } // gph
	double getFuelLevel(int tank) { return (500.0 + ((double) random(-100,100)))/10.0; }	// 0..1
	bool   getGenset()			{ return m_genset; }
	double getGenRPM()			{ return m_genset ? 3600 + random(-100,100) : 0; }
	double getGenOilPressure()	{ return m_genset ? 50 + random(-30,30) : 0; }	// psi
	double getGenCoolTemp()		{ return m_genset ? 180 + random(-40,40) : 0; }	// farenheight
	double getGenVoltage()		{ return m_genset ? 120 + random(-10,10) : 0; }
	double getGenFreq()			{ return m_genset ? 60 + random(-5,5) : 0; }

	int getNumWaypoints()		{ return m_num_waypoints; }		// in the "current route"
	int getWaypointNum() 		{ return m_waypoint_num; }		// return the "current waypoint" number
	const waypoint_t *getWaypoint(int wp_num)				// get a waypoing structure by index
	{
		if (wp_num >= 0 && wp_num < m_num_waypoints)
			return &m_waypoints[wp_num];
		return 0;
	}

	bool getAutopilot()			{ return m_autopilot; }
	bool getRouting()			{ return m_routing; }
	bool getArrived()			{ return m_arrived; }
		// get the autopilot, routing, and arrival status

	double headingToWaypoint();		// true heading to "current waypoint" from current position
	double distanceToWaypoint();	// NM to "current waypoint" from current position

	// helper functions that calculate the apparent wind based
	// the SOG, COG, true windAngle, and true windSpeed.

	double apparentWindAngle()	{ return m_app_wind_angle; }	// degrees relative to the bow of the boat
	double apparentWindSpeed()	{ return m_app_wind_speed; }	// knots


	// setters

	void setDepth			(double depth)		{ m_depth = depth; }
	void setSOG				(double sog)		{ m_sog = sog; calculateApparentWind(); m_rpms=sog?1800:0; }
	void setCOG				(double cog)		{ m_cog = cog; calculateApparentWind(); }
	void setWindAngle		(double angle) 		{ m_wind_angle = angle; calculateApparentWind(); }
	void setWindSpeed 		(double speed)		{ m_wind_speed = speed; calculateApparentWind(); }
	void setRPMS			(double rpms)		{ m_rpms = rpms; }
	void setGenset			(bool on)			{ m_genset = on; display(0,"GENSET %s",m_genset?"ON":"OFF"); }

	void setRoute(const char *route_name);
		// see ge_routes.h for names
		// stops the boat, turns off autopilot and routing
		// 		but does not stop the simulator
		// puts the boat at the 0th waypoint and sets the
		// 		current waypoint number to 1
	void setWaypointNum(int wp_num);
		// Sets the waypoint to navigate to.
		// Does nothing and reports an error if wp_num<0 or wp_num>getNumWaypoints()-1
		// Resets m_arrived and m_closest
	void jumpToWaypoint(int wp_num);
		// magically moves the boat to the given waypoint
		// without otherwise altering the simulator


	void setAutopilot(bool on);
		// While on, the simulator automatically sets heading to the
		// next waypoint in each time slice and starts watching for arrivals,
		// A completed arrival will turn off the autopilot if not routing.
		// Turning the autopilot off also turns routing off.
	void setRouting(bool on);
		// Turns routing on or off.
		// If routing, simulator will automatically advance to the
		// next waypoint after a completed arrival (when the boat
		// *arrives* and starts getting farther away from the current
		// waypoint). Routing will stop the boat upon completed arrival
		// at the final waypoint in the route.
		// Turning routing on or off also turns the autopilot on or off.

	static int g_MON_SIM;
	
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
	bool   m_genset;

	int m_waypoint_num;
	int m_num_waypoints;
	const waypoint_t *m_waypoints;

	// implementation

	uint32_t m_update_num;			// number of executed timeslices
	uint32_t m_last_update_ms;		// ms since last update
	uint16_t m_closest;				// integer feet

	void calculateApparentWind(bool quiet=true);

};	// class Simulator


extern boatSimulator boat;
	// static instance in boatSimulator.cpp



// end of boatSimulator.h
