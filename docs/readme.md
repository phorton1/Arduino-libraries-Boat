# Arduino Boat Library

**Home** --
**[Architecture](architecture.md)** --
**[Boat Simulator](boatSimulator.md)** --
**[Instruments](instSimulator.md)** --
**[Connectors & Cables](pinouts.md)** --
**[ST50 Testing](ST50_testing.md)**

repos: **[phorton1](https://github.com/phorton1)** --
**[teensyBoat Firmware](https://github.com/phorton1/Arduino-boat-teensyBoat/blob/master/docs/readme.md)** --
**[teensyBoat App](https://github.com/phorton1/base-apps-teensyBoat/blob/master/docs/readme.md)** --
**Boat Library** --
**[tbESP32 WiFi](https://github.com/phorton1/Arduino-boat-tbESP32/blob/master/docs/readme.md)** --
**[teensyWind Tester](https://github.com/phorton1/Arduino-boat-teensyWind/blob/master/docs/readme.md)** --
**[teensyGPS](https://github.com/phorton1/Arduino-boat-teensyGPS/blob/master/docs/readme.md)**

The **Arduino Boat Library** is a shared C++ library for Arduino (Teensy 4.x) that
provides a virtual boat simulator, a set of virtual marine instruments, and complete
encoders and decoders for three marine communications protocols: **Seatalk1**,
**NMEA 0183**, and **NMEA 2000**.  It also defines the binary packet protocol used
between the Teensy firmware and the
**[teensyBoat.pm](https://github.com/phorton1/base-apps-teensyBoat)**
Windows application.

The library is the shared core used by
**[teensyBoat.ino](https://github.com/phorton1/Arduino-boat-teensyBoat)**,
**[teensyGPS.ino](https://github.com/phorton1/Arduino-boat-teensyGPS)**, and
**[teensyWind.ino](https://github.com/phorton1/Arduino-boat-teensyWind)**.


## About This Work

The **Seatalk1** implementation in this library extends and, in several places,
corrects the only comprehensive public reference: Thomas Knauf's
*SeaTalk Technical Reference* (see Credits below).  Additional datagrams
not covered by Knauf, corrections to documented byte sequences verified against
real instruments, and complete encoding (transmit) coverage -- where Knauf
documents primarily the receive/decode direction -- are all included here.

The **Connectors & Cables** documentation records pinouts and wiring details
accumulated through hands-on work with Raymarine and Standard Horizon hardware.
Several entries correct or clarify misleading manufacturer diagrams.  The pinout
for the **Raymarine RS125 GPS puck** appears to be the first published anywhere.

The **ST50 instrument testing** documentation records empirical data and
discoveries from extended desktop testing of Raymarine ST50 Speed, Wind, and
Depth instruments without their real transducers.  Notable findings include:

- The ST50 Wind Instrument continuously **self-calibrates** the voltage ellipse
  of its transducer inputs -- a behavior not documented by Raymarine that explains
  why naive voltage spoofing fails on first use.
- The ST50 Wind Instrument operates as a Seatalk1 **repeater**, accepting
  wind datagrams directly and displaying them -- a feature buried in
  the Raymarine documentation.
- Empirical speed-to-frequency and wind-angle-to-voltage tables for the ST50
  Speed and Wind instruments, derived from direct measurement.
- A resistor-divider technique that fools the ST50 Depth instrument into
  sensing a transducer signal, allowing it to produce Seatalk output on the bench.


## Documentation Outline

- **[Architecture](architecture.md)** --
  How the library is organized: the virtual boat, virtual instruments, protocol
  layers, and the binary communication protocol.  How the library is reused across
  multiple projects.

- **[Boat Simulator](boatSimulator.md)** --
  The virtual boat: position, speed, wind, depth, engine, autopilot, and routing.
  API reference for the boatSimulator class.

- **[Instruments](instSimulator.md)** --
  The virtual instruments: depth, GPS, wind, speed/log, engine.  How each instrument
  maps to one or more protocols, and the instSimulator API.

- **[Connectors & Cables](pinouts.md)** --
  Pinouts and wiring for the Raymarine E80 (Seatalk, NMEA 0183, NMEA 2000),
  Standard Horizon GX2410 VHF/GPS, Raymarine RS125 GPS puck, and Seatalk 3-pin
  connectors.  Includes corrections to several manufacturer documents.

- **[ST50 Instrument Testing](ST50_testing.md)** --
  Desktop testing procedures and empirical data for the Raymarine ST50 Speed,
  Wind, and Depth instruments.  Connector pinouts, pulse-frequency and
  voltage tables, calibration procedures, and instrument behavior discoveries.


## Credits

- [**Thomas Knauf**](http://www.thomasknauf.de/seatalk.htm) --
  *SeaTalk Technical Reference Revision 3.23*.  The primary public reference for
  the Seatalk1 datagram protocol.  This library builds on, extends, and in
  places corrects that work.


## License

This software is released under the
[**GNU General Public License v3**](../LICENSE.TXT).


## Please Also See

- [**phorton1/Arduino-boat-teensyBoat**](https://github.com/phorton1/Arduino-boat-teensyBoat) --
  The primary firmware using this library.  Implements a complete multi-protocol
  bridge and desktop testing device for Seatalk1, NMEA 0183, and NMEA 2000;
  includes KiCad PCB design and 3D printed enclosure.

- [**phorton1/base-apps-teensyBoat**](https://github.com/phorton1/base-apps-teensyBoat) --
  The companion wxPerl Windows application.  Consumes the binary packet protocol
  defined in this library to monitor and control the teensyBoat firmware over
  USB serial or WiFi.

- [**phorton1/Arduino-boat-teensyGPS**](https://github.com/phorton1/Arduino-boat-teensyGPS) --
  A marine-grade Teensy/Neo6M GPS device built on this library; outputs Seatalk1
  and/or NMEA 2000.

- [**phorton1/Arduino-boat-teensyWind**](https://github.com/phorton1/Arduino-boat-teensyWind) --
  Interfaces directly with a Raymarine ST50/ST60 wind vane transducer; uses this
  library for signal processing and protocol output.

---

**Next:** [Architecture](architecture.md)
