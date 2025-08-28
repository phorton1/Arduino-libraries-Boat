# Architecture

- teensy 4.0 provides all actual protocol interfaces to boat or desktop systems
- ESP32 provides web based UI
- they communicate via a custom serial protocol
  - commands from the ESP32 to the teensy
  - monitor and historical information from the teensy to the ESP32

The ESP32 will communicate with the teensy via a custom built serial protocol,
which will be a line oriented command sequence allowing for an arbitrary
level of complexity in controlling the teensy.

Likewise, the teensy will send real-time monitoring information and/or
state, and/or historical information to the ESP32 for further processing.

To the degree that the serial protocol can also be implemented in Perl
on the laptop. it should be possible to bypass the ESP32 and connect
directly to the teensy with the (a) laptop to avoid entirely the usage
of wifi on the boat.

### Big Question

It is not clear that a single teensy can handle all of the traffic required
to both simulate, and monitor, the virtual boat and its instruments as well
as the real boat and its instruments.

It seems as if there wants to be two separate devices.  The simulator will
generally not be used on the boat, but the monitor will.  It seems nice to
be able to monitor the simulator itself.  For Seatalk and NMEA2000 this is
probably do-able, but for NMEA0183 it is complicated by the fact that there
is a single TALKER.

As it is, I would need multiple RS232 modules to even monitor the system,
but to simulate in-vitro, I need a way to switch the role (and GND) of
the VHF NMEA output.  With the fundamental requirement that AIS works without
any intermediate MPU, this gets tricky, electronically.


Makes we want to bite the bullet and just stick a teensy in between
the VHF and E80 for the NMEA0183.


I can *almost* envision a teensyt PCB with

- a CANBUS transiever for NMEA2000
- an opto-isolated interface to SeaTalk (requires 12V from Seatalk connector)
- two rs232 interfaces

where I would be mightily tempted to use a teensy 4.1 rather than a 4.0
so-as to have the built in SD card.  Which then further wants a standalone
touch screen LCD UI, which is seriously probably asking too much of a
single teensy.

So as we talk about throwing an ESP32 into the mix, then the ESP32
might/could have a display device (I have several with built in
LCDs, like the big, never used ideaSpark ones).


When, in the end, what I really want to be able to do route, waypoint,
and track management on the E80 via my own interface.  I am sort of
wistfully holding out hope that NMEA2000 will get me there, but I
know, in my heart, that is not the case.

Which then leads me to want to replace the screen in the old desk E80,
and see if I can get at it's CPU innards somehow, but that is probably
a hopelessly complicated and weird approach.  Even dealing with the
ARCHIVE.FSH files and CF cards was nearly hopelessly complicated.






I will/can initially implement the protocol via the teensy USB serial port.


### boatSimulator

The boat simulator will have commands to "drive" the boat, which, in turn
will drive an instrument simulator.

### instSimulator

The instrument simulator will encapsulate a number of particular instruments
on the virtual boat. Some of these will map to my existing actual ST50
or other instruments.

- depth
- log
- gps
- wind
- autopilot
- ais (future)

wheras others will exist solely to drive the E80 and/or for future expansion

- engine
- fridge (future)
- temperature (future)
  - outside
  - cabin

Each instrument can be turned on or off individually, and shall be allowed
to send on any protocol(s) which supports such an instrument.


### boatMonitor

A teensy 4.1 with an SD card can be used to log the "real" boat's protcols
(instruments) at some useful level of detail.  This is tricky because we
want to date/timestamp the log entries and make them of a fixed size so that
we can support reading from the end of the file(s).  A database on the teensy
might be a better proposition for logging and meeting these requirements.

The ESP32 *could* then support a useful UI that includes things like meters
and charts that show the state of the boat in a single glance, without going
so far as to try to emulate a chart-plotter.

- track logging

In addition, I sort of envision getting rid of the whole ESP32/myIOT architecture
on the boat, and using NMEA2000 to connect (and possibly power) my various device:

- fridgeController
- bilgeAlarm
- tempController
- lightController

which would then also require that I develop my own series of proprietary PGNS
to control these devices from the single ESP32 (web) entry point.


### Performance

Which then, in turn, pushes me to simplify the teensy, and make the esp32 more
complex.






