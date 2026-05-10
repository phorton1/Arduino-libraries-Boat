# Arduino Boat Library - Instruments

**[Home](readme.md)** --
**[Architecture](architecture.md)** --
**[Boat Simulator](boatSimulator.md)** --
**Instruments** --
**[Connectors & Cables](pinouts.md)** --
**[ST50 Testing](ST50_testing.md)**

repos: **[phorton1](https://github.com/phorton1)** --
**[teensyBoat Firmware](https://github.com/phorton1/Arduino-boat-teensyBoat/blob/master/docs/readme.md)** --
**[teensyBoat App](https://github.com/phorton1/base-apps-teensyBoat/blob/master/docs/readme.md)** --
**Boat Library** --
**[tbESP32 WiFi](https://github.com/phorton1/Arduino-boat-tbESP32/blob/master/docs/readme.md)** --
**[teensyWind Tester](https://github.com/phorton1/Arduino-boat-teensyWind/blob/master/docs/readme.md)** --
**[teensyGPS](https://github.com/phorton1/Arduino-boat-teensyGPS/blob/master/docs/readme.md)**

The instrument simulator rides on top of the boat simulator.
It's run() method calls the boat simulator's run() method,
and outputs the various protocols for selected instruments
after each call if the boat simulator is running.

Therefore it needs to know:

- a pointer to an actual nmea2000 object to send n2k messages
- the Seatalk (9 bit) serial port to send Seatalk messags
- the NMEA0183 serial port(s) to send NMEA0183 messags.

It encapsulates a number of virtual instruments:

- depth
- log
- gps
- wind
- engine

with the ability to assign each instrument to zero or more of
the three protocols.






---

**Next:** [Connectors & Cables](pinouts.md)
