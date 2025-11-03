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
	uint8_t num_wpts;
} route_t;


extern const route_t *simulator_routes __attribute__((weak));
extern const int simulator_num_routes __attribute__((weak));
	// Note that these are declared as weakly linked so that
	// they can be overriden by your application as desired.


#define TANK1_CAPACITY  72.0
#define TANK2_CAPACITY  72.0

	
class boatSimulator {

public:

	void init();			// stop the simulator and re-initialize
	void start();			// start the simulator (sets m_running=1)
	void stop();			// stop the simulator (sets m_running=0)
	void run();				// run one time slice of the simulator
		// call once per second or so

	// getters

	bool	 running()				{ return m_running; }		// true while simulator is running

	float    getMagneticVariance()	{ return 3.0; }				// in Bocas; On the E80; needed for ST_HEADING output calculations

	float	 getDepth()				{ return m_depth; }				// feet below surface
	float	 getHeading()			{ return m_heading; }			// the true direction the boat is pointing
	float	 getWaterSpeed()		{ return m_water_speed; }		// water relative to boat as measured by log instrument
	float 	 getCurrentSet()		{ return m_current_set; }		// direction current is going to
	float 	 getCurrentDrift()		{ return m_current_drift; }		// speed of the current
	double	 getDesiredHeading()	{ return m_desired_heading; }	// for AP, separate from routing

	float	 getSOG()				{ return m_sog; }				// CALCULATED knots
	float	 getCOG()				{ return m_cog; }				// CALCULATED true
	float	 getWindAngle() 		{ return m_wind_angle; }		// true direction its COMING FROM
	float	 getWindSpeed() 		{ return m_wind_speed; }		// knots
	double	 getLat()				{ return m_latitude; }
	double	 getLon()				{ return m_longitude; }
	float	 getCrossTrackError()	{ return m_track_error; }		// NM
	float 	 getLogTotal()			{ return 4321.0 + m_trip_distance; }	// NM
	float 	 getTripDistance()		{ return m_trip_distance; }		// NM
	bool 	 getTripOn()			{ return m_trip_on; }			// on or off

	uint16_t getRPM()				{ return m_rpm; }
	float 	 getOilPressure()		{ return m_oil_pressure; }		// psi
	float	 getBoostPressure()		{ return m_boost_pressure; }	// psi
	float    getOilTemp()			{ return m_oil_temp; }			// farenheight
	float    getCoolantTemp()		{ return m_coolant_temp; }		// farenheight
	float 	 getAltVoltage()		{ return m_alt_voltage; }		// volts
	float 	 getFuelRate()			{ return m_fuel_rate; }			// gph
	float	 getFuelLevel(int tank) { return tank ? m_fuel_level2 : m_fuel_level1; }	// 0..1
	float 	 getTankCapacity(int tank) { return tank ? TANK1_CAPACITY : TANK2_CAPACITY; }
	
	bool     getGenset()			{ return m_genset; }
	uint16_t getGenRPM()			{ return m_gen_rpm; }
	float 	 getGenOilPressure()	{ return m_gen_oil_pressure; }	// psi
	float 	 getGenCoolTemp()		{ return m_gen_cool_temp; }		// farenheight
	float    getGenVoltage()		{ return m_gen_voltage; }		// volts
	uint8_t  getGenFreq()			{ return m_gen_freq; }

	uint8_t	 getNumWaypoints()		{ return m_num_waypoints; }		// in the "current route"
	uint8_t  getStartWPNum()		{ return m_start_wp_num; }
	uint8_t  getTargetWPNum() 		{ return m_target_wp_num; }		// return the "current waypoint" number

	const waypoint_t *getWaypoint(uint8_t wp_num)				// get a waypoing structure by index
	{
		if (wp_num >= 0 && wp_num < m_num_waypoints)
			return &m_waypoints[wp_num];
		return 0;
	}

	bool getAutopilot()			{ return m_autopilot; }
	bool getRouting()			{ return m_routing; }
	bool getArrived()			{ return m_arrived; }
		// get the autopilot, routing, and arrival status

	float headingToWaypoint();		// true heading to "current waypoint" from current position
	float distanceToWaypoint();	// NM to "current waypoint" from current position

	// helper functions that calculate the apparent wind based
	// the SOG, COG, true windAngle, and true windSpeed.

	float apparentWindAngle()	{ return m_app_wind_angle; }	// degrees relative to the bow of the boat
	float apparentWindSpeed()	{ return m_app_wind_speed; }	// knots

	// setters

	void setDepth		(float depth);
	void setHeading		(float heading);
	void setWaterSpeed	(float speed);
	void setCurrentSet	(float angle);		// angle the water is going TO
	void setCurrentDrift(float speed);		// speed of the current
	void setWindAngle	(float angle);
	void setWindSpeed 	(float speed);
	void setDesiredHeading(float angle) 	{m_desired_heading = angle;}

	void setTripOn		(bool on)			{m_trip_on = on;}
	void setTripDistance(float distance)	{m_trip_distance = distance;}

		// NM, -1=off, 0 resets, or integer sets
	
	void setRPM			(uint16_t rpm);
	void setGenset		(bool on);

	const route_t *getRoute(const char *name);
		// for experimental use
		// previously route innards were not exposed
	void setRoute(const char *route_name);
		// see ge_routes.h for names
		// stops the boat, turns off autopilot and routing
		// 		but does not stop the simulator
		// puts the boat at the 0th waypoint and sets the
		// 		current waypoint number to 1
	void setStartWPNum(uint8_t wp_num);
		// magically moves the boat to the given waypoint
		// without otherwise altering the simulator
	void setTargetWPNum(uint8_t wp_num);
		// Sets the waypoint to navigate to.
		// Does nothing and reports an error if wp_num<0 or wp_num>getNumWaypoints()-1
		// Resets m_arrived and m_closest

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
	
	// added opaque time functions

	int getYear();
	int getMonth();
	int getDay();
	int getHour();
	int getMinute();
	int getSecond();

	void setDateTime(int year, int month, int day, int hour, int minute, int second);		// HH:MM::SS   (24 hour clock);

private:

	bool	 m_inited;
	bool	 m_running;
	bool	 m_autopilot;
	bool	 m_routing;

	bool	 m_trip_on;			// on-off
	float	 m_trip_distance;	// trip odometer

	float	 m_depth;
	float	 m_heading;
	float	 m_water_speed;
	float	 m_current_set;		// absolute angle the current is going TO
	float	 m_current_drift;	// absolute speed of the current
	
	float	 m_sog;
	float	 m_cog;
	float	 m_wind_angle;
	float	 m_wind_speed;
	double	 m_latitude;
	double	 m_longitude;
	float	 m_app_wind_angle;
	float	 m_app_wind_speed;

	float	 m_desired_heading;		// AP speicific
	float 	 m_estimated_set;
	float	 m_estimated_drift;
	bool	 m_arrived;				// ROUTING
	uint16_t m_closest;				// integer feet
	float 	 m_track_error;

	uint16_t m_rpm;
	float	 m_boost_pressure;
	float 	 m_oil_pressure;
	float 	 m_oil_temp;
	float 	 m_coolant_temp;
	float	 m_alt_voltage;
	float	 m_fuel_rate;
	float	 m_fuel_level1;
	float	 m_fuel_level2;

	bool  	 m_genset;
	float 	 m_gen_rpm;
	float    m_gen_oil_pressure;
	float 	 m_gen_cool_temp;
	float	 m_gen_voltage;
	uint8_t	 m_gen_freq;

	uint8_t	 m_start_wp_num;
	uint8_t	 m_target_wp_num;
	uint8_t	 m_num_waypoints;
	const waypoint_t *m_waypoints;

	// implementation

	uint32_t m_update_num;			// number of executed timeslices
	uint32_t m_last_update_ms;		// ms since last update

	void doAutopilot();
	void calcuateCrossTrackError();

	void calculateOverGround(bool quiet=true);
	void calculateApparentWind(bool quiet=true);
	void calculate(bool quiet=true)
	{
		calculateOverGround(quiet);
		calculateApparentWind(quiet);
	}
	
	void sendBinaryBoatState(bool doit=1);
	

};	// class Simulator


extern boatSimulator boat;
	// static instance in boatSimulator.cpp




// end of boatSimulator.h
