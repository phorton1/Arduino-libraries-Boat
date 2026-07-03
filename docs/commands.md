# Arduino Boat Library - Commands

**[Home](readme.md)** --
**[Architecture](architecture.md)** --
**[Boat Simulator](boatSimulator.md)** --
**[Instruments](instSimulator.md)** --
**Commands** --
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

The Boat library defines the operations that a teensyBoat-family firmware
exposes as a **text command interface** over USB serial.  The same commands are
also accepted when forwarded by a companion device (the tbESP32 WiFi bridge) or
by the **teensyBoat.pm** wxPerl application's HTTP `/api/command` endpoint.

This page is the **authoritative reference** for the commands the Boat library
implements -- their syntax, parameters, and effect on simulator and instrument
state.  Firmware sketches built on this library (teensyBoat.ino, teensyGPS.ino,
teensyWind.ino) may add a small number of sketch-only commands of their own;
those are documented in the respective firmware's `commands.md`.

For the C++ API used inside the library, see **[Boat Simulator](boatSimulator.md)**
and **[Instruments](instSimulator.md)**.


## Command Syntax

- Commands are **case-insensitive**.
- Two forms:
    - **Monadic** -- a single keyword:  `RUN`, `STOP`, `SIM`, `ROUTES`, `LOAD`
    - **Setter**  -- key/value:  `S=5`, `ROUTE=Popa`, `WT=78.8F`
- Each command must be terminated by a newline (`\n` or `\r\n`).
- When commands are sent through the HTTP API, the `=` in setter commands must
  be percent-encoded as `%3D` (e.g.  `cmd=S%3D5`).  See the
  [teensyBoat App Integration](https://github.com/phorton1/base-apps-teensyBoat/blob/master/docs/integration.md)
  page.


## Port Mask Bits

Several commands take a **port mask** -- a bitfield naming a combination of the
five output protocol ports.  These bits appear unchanged everywhere a port mask
is used.

| Bit    | Port          | Notes                                |
|--------|---------------|--------------------------------------|
| 0x01   | ST1           | Seatalk1 port 1                      |
| 0x02   | ST2           | Seatalk1 port 2                      |
| 0x04   | 83A           | NMEA 0183 port A                     |
| 0x08   | 83B           | NMEA 0183 port B                     |
| 0x10   | 2000          | NMEA 2000 CAN bus                    |


## Boat Simulator -- State

These commands set or query the **virtual boat's** physical state.  All setters
take effect immediately; values persist until changed.

| Command   | Value                            | Effect                                                       |
|-----------|----------------------------------|--------------------------------------------------------------|
| `I`       | -                                | Re-initialize the simulator (preserves running state)        |
| `RUN`     | -                                | Start the simulator's 1 Hz time loop                         |
| `STOP`    | -                                | Stop the simulator; state is preserved, instrument output halts |
| `S=N`     | knots                            | Water speed.  **Must be > 0 for the boat to move.**          |
| `H=N`     | degrees true                     | Heading.  Overridden by autopilot each tick if AP is on.     |
| `D=N`     | feet                             | Depth of water under keel                                    |
| `WT=N`    | Celsius, or `N.NF` for Fahrenheit | Water temperature.  Suffix `F` (or `f`) converts from F to C.|
| `WA=N`    | degrees true                     | True wind angle -- the direction the wind is coming FROM     |
| `WS=N`    | knots                            | True wind speed                                              |
| `CS=N`    | degrees true                     | Current set -- angle the water is going TO                   |
| `CD=N`    | knots                            | Current drift -- speed of the current                        |
| `RUD=N`   | -30.0 .. 30.0 degrees            | Rudder angle (direct, unsmoothed)                            |
| `RPM=N`   | integer                          | Engine RPM.  Overridden by SOG: forced to 1800 if SOG > 0, else 0. |
| `GEN=N`   | 0 or 1                           | Stop or start the genset                                     |
| `TRIP=N`  | NM                               | Trip odometer distance                                       |
| `TRIP_ON=N` | 0 or 1                         | Trip odometer on/off                                         |
| `DT=YYYY-MM-DD HH:MM:SS` | UTC                | Set the simulator's date/time (RTC)                          |


## Boat Simulator -- Routing and Autopilot

The simulator can navigate a built-in **route** of named **waypoints**.

| Command       | Value                | Effect                                                       |
|---------------|----------------------|--------------------------------------------------------------|
| `ROUTE=name`  | route name           | Load named route; place boat at waypoint 0; turn off autopilot and routing |
| `J=N`         | waypoint index       | Teleport (Jump) the boat to waypoint N; turn off autopilot and routing |
| `WP=N`        | waypoint index       | Set the target waypoint to N; do not move the boat           |
| `AP=N`        | 0, 1, or 2           | Autopilot mode: 0 = OFF (also disables routing), 1 = AUTO (steer toward target), 2 = VANE (placeholder; not implemented) |
| `R=N`         | 0 or 1               | Routing off/on.  `R=1` also enables autopilot AUTO and starts watching for waypoint arrivals; advances to next waypoint on arrival; stops at final waypoint. |
| `DH=N`        | degrees true         | Autopilot desired heading (used when AP is on but routing is off) |

To list available routes or inspect waypoint coordinates, see **Query Commands**
below.

A typical driving sequence is:

```
ROUTE=Popa     # load route, place boat at wp 0
S=5            # set water speed (boat does not move at S=0)
RUN            # start the simulator
R=1            # start routing (also enables AP AUTO)
SIM            # query state
```


## Boat Simulator -- Virtual AIS Targets

A set of persistent virtual "other boats" (vboats) that circle in and out of range
of the simulated boat and emit AIS.  There is **no on/off command**: the vboats run
whenever the AIS virtual instrument is enabled on any port (`I_AIS=M` -- even a
SeaTalk port, though SeaTalk carries no AIS).  NMEA 0183 emits `!AIVDM` messages 18
and 24; NMEA 2000 emits PGNs 129039 / 129809 / 129810.

| Command         | Value    | Effect                                                                                    |
|-----------------|----------|-------------------------------------------------------------------------------------------|
| `AIS`           | -        | Dump the current vboat state to the serial output                                         |
| `AIS_N=N`       | 0 .. 8   | Number of virtual boats (default 5)                                                       |
| `AIS_RATE=N`    | seconds  | Average seconds between AIS bursts on the wire, >= 1 (default 3).  One vboat sends one message type per burst, so ~ `AIS_N * 3 * rate` seconds cycles through all of them. |
| `AIS_MIN_CPA=N` | NM       | Standoff distance normal vboats keep from the sboat (default 0.5).  Set above the receiver's CPA alarm ring so normal traffic never trips it. |
| `AIS_RANGE=N`   | NM       | Outer range vboats appear within and recycle beyond (default 4; forced to at least `AIS_MIN_CPA + 1`, and raising `AIS_MIN_CPA` bumps it up) |
| `COLLIDE=N`     | 0 or 1   | Put one vboat on a steady collision course toward the sboat to test the receiver's CPA alarm |


## Query Commands

These print state to the serial output (and the wxPerl app's log ring buffer).

| Command           | Output                                                                |
|-------------------|-----------------------------------------------------------------------|
| `SIM`             | Multi-line snapshot of simulator state: running/routing/ap/arrived, route name and target waypoint, position/heading/speed, distance/heading-to-waypoint, cross-track error |
| `ROUTES`          | List of all available routes with waypoint counts                     |
| `ROUTE_WPS=name`  | Every waypoint in the named route with index, name, lat, lon          |

The `SIM` output is the standard means of polling simulator state from automation.


## Virtual Instruments

Each virtual instrument is assigned to a combination of the five protocol
ports via a port mask.  When the simulator is running, enabled instruments
emit on the protocols they are assigned to.

### Per-instrument port assignment

| Command       | Value      | Effect                                                  |
|---------------|------------|---------------------------------------------------------|
| `I_DEPTH=M`   | port mask  | Depth sounder output ports                              |
| `I_LOG=M`     | port mask  | Speed/log instrument output ports                       |
| `I_WIND=M`    | port mask  | Wind instrument output ports                            |
| `I_COMPASS=M` | port mask  | Heading/compass output ports                            |
| `I_GPS=M`     | port mask  | GPS output ports                                        |
| `I_AIS=M`     | port mask  | AIS output ports                                        |
| `I_AP=M`      | port mask  | Autopilot output ports                                  |
| `I_ENG=M`     | port mask  | Engine monitor output ports                             |
| `I_GEN=M`     | port mask  | Genset output ports                                     |

### Per-port all-instruments

Turn every instrument on or off for a specific port in one shot.

| Command     | Value   | Effect                                              |
|-------------|---------|-----------------------------------------------------|
| `I_ST1=N`   | 0 or 1  | Set/clear ST1 bit on every instrument               |
| `I_ST2=N`   | 0 or 1  | Set/clear ST2 bit on every instrument               |
| `I_83A=N`   | 0 or 1  | Set/clear 83A bit on every instrument               |
| `I_83B=N`   | 0 or 1  | Set/clear 83B bit on every instrument               |
| `I_2000=N`  | 0 or 1  | Set/clear 2000 bit on every instrument              |

### Forwarding

The firmware can forward incoming Seatalk1 or NMEA 0183 traffic from one
port to its sibling.  Forwarding is independent of the virtual instruments.

| Command         | Value      | Effect                                                |
|-----------------|------------|-------------------------------------------------------|
| `FWD=M`         | bitfield   | Forwarding mask (see bits below)                      |
| `E80_FILTER=N`  | 0 or 1     | When on, suppress GSA, GLL, RMC NMEA 0183 sentences from being forwarded 83A -> 83B.  These sentences crash the Standard Horizon GX2410's internal GPS. |

Forwarding bits:

| Bit    | Direction                  |
|--------|----------------------------|
| 0x01   | ST1 -> ST2                 |
| 0x02   | ST2 -> ST1                 |
| 0x04   | 83A -> 83B                 |
| 0x08   | 83B -> 83A                 |


## Monitoring

Monitoring controls how much information about traffic on each port is printed
to the serial output (and forwarded into the wxPerl app's log ring buffer).

| Command     | Value     | Meaning                                                       |
|-------------|-----------|---------------------------------------------------------------|
| `M_SIM=N`   | 0..4      | Boat simulator calculation detail (1 is default)              |
| `M_ST1=N`   | 0/1/4+    | Seatalk1 port 1 monitoring: >= 1 = all in/out, >= 4 = with debugging |
| `M_ST2=N`   | 0/1/4+    | Seatalk1 port 2 monitoring (same scale as M_ST1)              |
| `M_83A=N`   | bitfield  | NMEA 0183 port A monitoring (bits below)                      |
| `M_83B=N`   | bitfield  | NMEA 0183 port B monitoring (same bits as M_83A)              |
| `M_2000=N`  | bitfield  | NMEA 2000 monitoring (bits below)                             |

NMEA 0183 monitoring bits (`M_83A`, `M_83B`):

| Bit    | Meaning                  |
|--------|--------------------------|
| 0x01   | all sentences in/out     |
| 0x02   | AIS sentences in only    |

NMEA 2000 monitoring bits (`M_2000`):

| Bit       | Meaning                                              |
|-----------|------------------------------------------------------|
| 0x0001    | Sensors out, known messages in                       |
| 0x0002    | GPS/AIS specifically                                 |
| 0x0004    | Known proprietary in                                 |
| 0x0008    | Unknown (not sensors, not proprietary, not bus) in   |
| 0x0010    | Known BUS in                                         |
| 0x0020    | Known BUS out                                        |
| 0x1000    | Include self-sent messages, not just received        |
| 0x8000    | Show raw "instrument" messages                       |


## EEPROM and State

| Command   | Effect                                                          |
|-----------|-----------------------------------------------------------------|
| `LOAD`    | Load saved instrument configuration from EEPROM                 |
| `SAVE`    | Save current instrument configuration to EEPROM                 |
| `CLEAR`   | Reset all instruments, monitoring, forwarding, GP8 function, and M_SIM.  Does not save. |
| `STATE`   | Emit the current state as a binary `BINARY_TYPE_PROG` packet    |


## GP8 Connector and ST50 Testing

The teensyBoat hardware exposes a **General Purpose 8-pin connector** (the GP8)
whose mode determines whether the speed-pulse and PWM lines drive a real ST50
instrument under test or operate in an alternate mode.  See the
**[ST50 Testing](ST50_testing.md)** page for the testing procedures these
commands support.

### GP8 mode

| Command          | Value                                | Effect                                            |
|------------------|--------------------------------------|---------------------------------------------------|
| `GP8_MODE=X`     | `0`/`off`, `1`/`speed`, `2`/`wind`, `3`/`esp32` | Set the GP8 connector function          |

Modes:

- **off** -- GP8 inactive.
- **speed** -- emit speed pulses for testing the ST50 Speed instrument.
- **wind** -- emit PWMA/PWMB voltages for testing the ST50 Wind instrument.
- **esp32** -- enable serial and UDP communication with a tbESP32 WiFi bridge.

### ST50 testing setters

These have effect only in the appropriate GP8 mode.

| Command         | Value      | Effect                                                                |
|-----------------|------------|-----------------------------------------------------------------------|
| `LAMP=N`        | 0..3       | Send Seatalk1 lamp-intensity message to all ST ports                  |
| `RAW_MODE=N`    | 0 or 1     | 0 = use calculated values for ST50 pulses/PWM; 1 = use the raw setters below |
| `HZ=N`          | hertz      | Raw pulse frequency for `GP8_MODE=speed`                              |
| `PWMA=N`        | 0..255     | Raw duty cycle for the PWMA output (ST50 Wind green)                  |
| `PWMB=N`        | 0..255     | Raw duty cycle for the PWMB output (ST50 Wind blue)                   |
| `WIND_CIRCLE=N` | 0, 1, 2    | Automatic wind-angle sweep in `GP8_MODE=wind`: 0 = off, 1 = 2 deg per 250 ms calibration circle, 2 = 15 deg per 15 s measurement circle |


## NMEA 2000 Device Queries

| Command   | Effect                                              |
|-----------|-----------------------------------------------------|
| `L`       | List known NMEA 2000 devices on the bus             |
| `Q`       | Broadcast a device-query message to the bus         |


## Binary Streaming

The firmware can stream selected channels of state in a compact binary form
to a connected wxPerl host application.  These commands manipulate the
global binary-output mask `g_BINARY`.

| Command       | Value     | Effect                                                                                   |
|---------------|-----------|------------------------------------------------------------------------------------------|
| `B=N`         | bitfield  | Set `g_BINARY` directly to the given mask                                                |
| `B_PROG=N`    | 0 or 1    | Enable/disable `BINARY_TYPE_PROG` (instrument configuration packets)                     |
| `B_SIM=N`     | 0 or 1    | Enable/disable `BINARY_TYPE_SIM` (boat simulator state packets at 1 Hz)                  |
| `B_BOAT=N`    | 0 or 1    | Enable/disable `BINARY_TYPE_BOAT` (reserved -- "actual boat" channel, unimplemented)     |
| `B_AIS=N`     | 0 or 1    | Enable/disable `BINARY_TYPE_AIS` (aggregated virtual AIS target packets)                 |
| `B_ST=N`      | 0 or 1    | Enable/disable both `BINARY_TYPE_ST1` and `BINARY_TYPE_ST2` (decoded Seatalk1 packets)   |
| `B_0183=N`    | 0 or 1    | Enable/disable both `BINARY_TYPE_0183A` and `BINARY_TYPE_0183B` (raw NMEA 0183 sentences) |
| `B_2000=N`    | 0 or 1    | Enable/disable `BINARY_TYPE_2000` (decoded NMEA 2000 PGN data)                           |

The wxPerl application enables specific bits on window-open and disables them on
window-close, so the firmware only emits the binary traffic it is actually being
listened to for.


## Underlying API

The text command parser is implemented in `teensyBoat.ino` (and parallel sketches),
but the underlying actions are all calls into Boat library classes.  Mapping:

- **Boat simulator state, routing, autopilot, query** -- `boatSimulator` member
  functions on the global `boat_sim` instance.  See **[Boat Simulator](boatSimulator.md)**.
- **Virtual instrument configuration, monitoring, forwarding, GP8, ST50 testing,
  EEPROM** -- `instSimulator` member functions on the global `inst_sim` instance.
  See **[Instruments](instSimulator.md)**.
- **NMEA 2000 device queries** -- `tNMEA2000`-derived `nmea2000` instance defined
  in `inst2000.cpp`.
- **Binary streaming** -- `g_BINARY` in `boatBinary.h` and the framing helpers
  `startBinary()` / `endBinary()` in `boatBinary.cpp`.

---

**Next:** [Connectors & Cables](pinouts.md)
