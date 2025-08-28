# boatSimulator

A generic platform independent Arduino library that provides
a simple simulation of a boat with a motor.
It has a rudimentary **autopilot** that uses a built-in set of
**Routes**, each consisting of an array of *Waypoints*.

I use this library to to generate data for testing various hardware
interfaces including my Seatalk, NMEA1083 and NMEA2000 hardware projects.
It has been compiled for ESP32 and teensy 4.x platforms using the Arduino IDE.

Specifically, for better or worse, I have used the data to *spoof*
various devices on my workbench, away from the boat:

- Primarily I used the data to drive a Raymarine E80 MFD (multi function
  display) chartplotter using ALL THREE protocols to learn what it responds
  to (and doesn't), and to better understand its behavior.
- I used the data to drive a ST50 Multi display instrument over Seatalk.
  to verify that it was working correctly.
- I used the data to drive a Standard Horizon GX2410G VHF/AIS radio with
  NMEA0183 (TODO: and NMEA2000) to spoof it into accepting the simulator's
  GPS coordinates over its own internal GPS.

In general, this code is used by higher level code that builds and sends out
messages (sentences, packets) in the various protocols.  As such, typically
that higher level code also allows for monitoring (seeing) the messages being
**received**  from various devices, but I defer that discussion to elsewhere.

Note that SOG (speed over ground) is initalized to zero in init(), and so,
until you call **setSOG()** with a non-zero number of knots, the boat will not
move, even if you properly call start() and run() !!



### Basics

There is a single static global instance, **boat**, of the class in
boatSimulator.cpp.

- call **boat.init()** from your setup() method, or additionally later to stop and
  re-initialize the simulator at any time.
- call **boat.run()** from your loop() method (or a task) once per second or so
- call **boat.start()** to start the simulator (sets running=1)
- call **boat.stop()** to stop the simulator (sets running=0) and freeze it
- call **boat.setCOG()** and **boat.setSOG()** to set the boat's speed
  and course over ground to manually drive the boat around.
- call **boat.setAutopilot(true)** to cause the boat to drive to the
  current waypoint at the current SOG
- call **setRouting(true)** to further cause the autopilot to advance
  to the next waypoint in the route after it arrives at the current one.

You can check the state of **running()**. While the simulator is running
you probably want to call the various **getter methods** to obtain
semi-realistic data that can be used (by higher level code) to build and
send out the various protocols.


### Predetermined (compiled-in) Routes and Waypoints.

In the file **ge_routes.h** are (TODO *weakly linked*) extern declarations of
a simple array of *Routes* each consisting of an array of *Waypoints*.
By **weakly linking** these, it means you can provide your own set of Routes
and Wayppoints in your program by simply providing a replacement
ge_routes.cpp file somewhere in your sketch folder.


### Route and Waypoint Initialization

When the simulator is first initialized, the 0th route is set
as the **current route**.  You can explicitly call **setRoute()**
to change the current route if you wish. Thereafter the current
route is not changed by subsequent calls to init().


#### Current position versus Current Waypoint

Anytime init() is called, the boat is magically moved (jumps) to the lat/lon
of 0th waypoint in the current route.  This just sets the position of the
boat and has nothing to do with the notion of the *current waypoint*.

The **current waypoint** is defined as the waypoint the autopilot will
drive to if it is engaged.  Hence when init() is called, since we moved
the boat to the 0th waypoint in the route, we set the current waypoint
to the **1st waypoint**  in the list, where you would want to go from
the 0th one.

You can physically move the boat to any waypoint in the list by calling
**jumptoWaypoint(N)** where N is from 0 to getNumWaypoints()-1, but please
remember that has nothing to do with, and does not change, the *current waypoint*.

You can set the **current waypoint** the autopilot will drive to
by calling **setWaypoint()**



### Autopilot and Routing (Arrivals)

When **setAutopilot(true)** is called, after the new lat/lon are
determined in each timeslice by the simulater, the new COG (heading) will
be calculated by the simulator calling headingToWaypoint(). In this
way the boat continues pointing at the waypoint when the autopilot is
engaged regardless of where the boat is.

When the autopilot is engaged, and the boat gets within 400 feet
(0.064 nautical miles) of the current waypoint, the boat is said to
have **INITIALLY ARRIVED** at the waypoint.  At that point the method
**getArrived()** starts returning *true* and the COG is no longer
automatically calculated on each timeslice.

The boat just continues on that COG.

Upon the INITIAL ARRIVAL, the simulator starts keeping track of the
*closest approach* to the waypoint.  As long as the boat keeps
moving towards the waypoint, getArrived() will continue to
return *true*.  However, once the boat passes the waypoint and
starts moving away it, the simulator starts returning *false*
to any calls to getArrived().  This is termed a **COMPLETED ARRIVAL**.

Upon a COMPLETED ARRIVAL, one of three things happens, based on whether
**Routing** is engaged and whether the boat has reached the
last waypoint in the route.

- If routing is **off**, then the Autopilot will disengage
  and the boat will continue on whatever COG/SOG it was at.
- If routing is **on** and there are more waypoints in
  the route, then the **current waypoint** (number) will be
  advanced to the next waypoint in the list, and the autopilot
  will start driving to that waypoint.
- If routing is **on** and the boat has arrived at the last
  waypoint in the route, the SOG will be set to zero (the boat
  will be stopped), and the autopilot and routing will be disengaged
  (turned off).

This behavior was specifically implemented so that I could test
the E80's **Arrival Alarm** and have it turn off automatically
when routing and starting towards the next waypoint.

*Note to self: TODO.  I currently only send out the NMEA autopilot messages
when getAutopilot() is true.   Because of this, once the autopilot
is turned off, I no longer send any more messages, and the alarm on
the E80 continues to beep until I press the stupid button.  I will
change my higher level code to send out ONE MORE message, with the
arrival alarm turned off, so that I dont have to listen to and
repond to that beeping!*

Note that engaging routing (calling setRouting(true)) automatically
engages the autopilot too, and that disengaging routing also automatically
disengages the autopilot too.


### Engine Simulation

The engine simulation is rudimentary, and generally just returns
reasonable random values so that I can see the display change.

You can explicitly set the RPMs

- if the rpms are non-zero, random, but reasonable,
  oil pressure, alternator voltage, oolant temparature
  and fuel_rate values will be generated.
- if the rpms==0 the simulator returns zero for these values

Changing the SOG sets the rpms.

- setSOG(0) *stops* the motor by setting the rpms to zero
  too, and changing the SOG to a non-zero
- setSOG(non-zero)  sets the rpms to an arbitrary value of
  **1800 rpms.**

Yes, I typically drive the simulator at 90 knots at only
1800 rpms in my sailboat!  It's great!

But if you want to see the RPMS change, you must manually
call **setRPMS()** as needed.


### Fuel Simulation

Likewise, the Fuel Simulation is essentially
random around 50% for each tank.  I just wanted
to make sure I could send the various protocols
to the chartplotter and so I did not implement
any kind of a realistic motor/fuel simulation.

However, the API supports two fuel tanks, because
there are two fuel tanks on my boat, and the
E80 chartplotter display shows two fuel tanks.

TODO: check that the E80 actually DOES display
two fuel tanks.




end of boatSimulator.md

