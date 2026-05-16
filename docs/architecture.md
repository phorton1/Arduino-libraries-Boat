# Arduino Boat Library - Architecture

**[Home](readme.md)** --
**Architecture** --
**[Boat Simulator](boatSimulator.md)** --
**[Instruments](instSimulator.md)** --
**[Commands](commands.md)** --
**[Connectors & Cables](pinouts.md)** --
**[ST50 Testing](ST50_testing.md)** --
**[Seatalk Knowledge](new_st_knowledge.md)**

repos: **[phorton1](https://github.com/phorton1)** --
**[teensyBoat Firmware](https://github.com/phorton1/Arduino-boat-teensyBoat/blob/master/docs/readme.md)** --
**[teensyBoat App](https://github.com/phorton1/base-apps-teensyBoat/blob/master/docs/readme.md)** --
**Boat Library** --
**[tbESP32 WiFi](https://github.com/phorton1/Arduino-boat-tbESP32/blob/master/docs/readme.md)** --
**[teensyWind Tester](https://github.com/phorton1/Arduino-boat-teensyWind/blob/master/docs/readme.md)** --
**[teensyGPS](https://github.com/phorton1/Arduino-boat-teensyGPS/blob/master/docs/readme.md)**

The Boat Library is built in three layers, each one sitting on top of the previous.
Together they let a small microcontroller pretend to be a fully-equipped boat on
a marine network.


## Layer 1 - The Virtual Boat

At the base is a simple model of a boat under way -- the **boatSimulator**.
Think of it as a stripped-down nautical video-game engine.
At any moment it knows where the boat is on earth, what direction it is heading,
how fast it is moving, what the wind is doing, how deep the water is,
and what the engine is up to.
Time advances in one-second steps, and with each step the simulator moves the
boat forward and updates everything consistently.

The simulator also has a basic **autopilot**.  Give it a target position (a waypoint)
and it will steer toward it.  String waypoints together into a route and enable
**routing**, and it will navigate the full sequence automatically, stopping the
boat when it reaches the end.

None of this involves real electronics -- the virtual boat is just numbers in memory,
updated once a second.  Its job is to produce a continuous stream of realistic,
self-consistent data for the layer above.


## Layer 2 - The Virtual Instruments

Real marine instruments take a physical measurement and broadcast it over
one or more communications networks.  The **instSimulator** does exactly the same
thing with the virtual boat's data.

Each virtual instrument -- depth sounder, GPS receiver, wind instrument,
speed/log, engine monitor -- can be switched on or off independently.
Each can also be assigned to any combination of the three marine protocols
that this library supports.  A virtual GPS, for example, can simultaneously
send its position as a Seatalk1 datagram, an NMEA 0183 sentence, and an
NMEA 2000 packet -- just as a real multi-protocol GPS device would.

When the boat is moving, the instrument simulator calls the boat simulator
once per second, reads the updated state, and fires off whatever output
has been configured.


## Layer 3 - The Protocol Encoders and Decoders

The library contains complete send (encode) and receive (decode) implementations
for three marine communications protocols.

**Seatalk1** is Raymarine's older proprietary 9-bit serial protocol, used by
ST50 and ST60 series instruments and by Raymarine chartplotters of the E80/E120
generation.  The Seatalk1 implementation here builds on and extends the only
published public reference, Thomas Knauf's *SeaTalk Technical Reference*
(see the home page for details).

**NMEA 0183** is the long-established industry-standard text protocol understood
by virtually all marine electronics.  The library implements the sentence types
for GPS position, depth, wind, speed, heading, engine data, and AIS.

**NMEA 2000** is the modern CAN-bus-based standard now used in most new marine
installations.  It carries the same kinds of information as the older protocols
but in a structured binary format over a dedicated bus.


## The Binary Protocol

When **teensyBoat.ino** is connected to the **teensyBoat.pm** Windows application
over USB, a fourth channel carries data back and forth in a compact binary format --
the **binary packet protocol** defined in boatBinary.h.  This channel carries
instrument state snapshots for the PC display, monitoring output for each protocol,
and commands from the PC back to the firmware.  Both sides of this protocol
(firmware and PC application) are driven by the definitions in this library.


## Reuse Across Projects

Because all the protocol knowledge and simulation logic live in this shared library,
individual projects only need to add the hardware-specific wiring.

- **teensyBoat.ino** adds the physical serial and CAN interfaces, the USB connection
  to the PC application, and a general-purpose GPIO connector for instrument testing.
- **teensyGPS.ino** uses just the GPS output side of the protocol layer to build
  a standalone marine GPS device.
- **teensyWind.ino** uses the signal-processing math to read a real Raymarine
  wind vane transducer and convert its analog signals into protocol output.

---

**Next:** [Boat Simulator](boatSimulator.md)
