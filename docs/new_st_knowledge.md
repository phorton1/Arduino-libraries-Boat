# Arduino Boat Library - Seatalk Knowledge

**[Home](readme.md)** --
**[Architecture](architecture.md)** --
**[Boat Simulator](boatSimulator.md)** --
**[Instruments](instSimulator.md)** --
**[Connectors & Cables](pinouts.md)** --
**[ST50 Testing](ST50_testing.md)** --
**Seatalk Knowledge**

repos: **[phorton1](https://github.com/phorton1)** --
**[teensyBoat Firmware](https://github.com/phorton1/Arduino-boat-teensyBoat/blob/master/docs/readme.md)** --
**[teensyBoat App](https://github.com/phorton1/base-apps-teensyBoat/blob/master/docs/readme.md)** --
**Boat Library** --
**[tbESP32 WiFi](https://github.com/phorton1/Arduino-boat-tbESP32/blob/master/docs/readme.md)** --
**[teensyWind Tester](https://github.com/phorton1/Arduino-boat-teensyWind/blob/master/docs/readme.md)** --
**[teensyGPS](https://github.com/phorton1/Arduino-boat-teensyGPS/blob/master/docs/readme.md)**

This page records what was learned implementing Seatalk1 encode and decode
in this library against real hardware -- the **Raymarine E80 chartplotter**,
**ST7000 autopilot head**, and several **ST50 instruments** -- and compares
those findings against the only comprehensive public reference:
Thomas Knauf's *SeaTalk Technical Reference Revision 3.23*
(http://www.thomasknauf.de/seatalk.htm).

Entries are grouped as **Corrections** (Knauf is demonstrably wrong or
incomplete), **Extensions** (additional detail not covered by Knauf), and
**New Datagrams** (messages observed on the bus that do not appear in Knauf
at all).  Source evidence for every entry is in `instST_in.cpp` and
`instST_out.cpp` in this library.


---

## Corrections to Knauf

### 0x122 -- Log Total: last byte is not optional

Knauf's description of the three-byte LOG TOTAL field ignores the third
data byte entirely, giving a 16-bit range.  The actual encoding is a full
24-bit little-endian integer divided by 10 to produce nautical miles:

    total_nm = (dg[4] << 16 | dg[3] << 8 | dg[2]) / 10.0

Ignoring `dg[4]` causes the total to wrap at approximately 6,553 nm instead
of correctly rolling over at ~1,677,721 nm.  Real instruments send non-zero
values in `dg[4]` once the trip log exceeds 6,553 nm.

*Source: instST_in.cpp, ST_LOG_TOTAL decode, comment "knaufman's spec is
wrong here, he ignores the last byte".*


### 0x184 / 0x19C -- Heading encoding: "turning direction" bit is wrong

Knauf's heading formula for datagrams 0x184 (Autopilot) and 0x19C (Rudder)
is:

    heading = (U & 0x3) * 90
            + (VW & 0x3F) * 2
            + (U & 0xC ? (U & 0xC == 0xC ? 2 : 1) : 0)

He labels the two high bits of `U` as a "turning direction / ship turns right"
flag.  This is incorrect for the ST7000/E80 generation.  The correct
interpretation is:

    heading = (U & 0x3) * 90        -- quadrant (0..3 * 90 degrees)
            + (VW & 0x3F) * 2       -- twos (0..63 * 2 degrees)
            + ((U >> 2) & 0x3) * 0.5  -- half-degrees (0..3 * 0.5 degrees)

This is identical to the 0x189 (Compass Heading) encoding.  Treating the
high two bits of `U` as half-degrees works correctly with both the E80 and
the ST7000 head; Knauf's "turning bit" interpretation caused incorrect
heading display until this was corrected.

*Source: instST_out.cpp, ST_AUTOPILOT and ST_RUDDER send blocks, extended
comment beginning "KNAUFS DESCRIPTION" and "MY NOTES".*


### 0x184 -- Z byte high nibble must be 0x4 for ST7000

Knauf writes the Z byte field as `0Z` in his datagram header, implying the
high nibble is zero.  On the ST7000 autopilot head, the high nibble of the Z
byte **must be 0x4** (i.e. the byte is `0x4Z`).

If the high nibble is zero the ST7000 accepts the datagram silently but **does
not display the rudder bar indicator**.  The real Raymarine 400G autopilot CPU
always sends `0x4Z`.  This took approximately a full day of debugging to
identify.

*Source: instST_out.cpp, apInst::sendSeatalk(), comment
"THIS IS REQUIRED FOR THE ST7000 to show the Rudder Bar".*


### 0x186 -- AP Keystroke: byte structure and checksum

Knauf's description of the 0x186 keystroke datagram is incomplete.  The
complete structure is:

- `dg[0]` = 0x86 (command byte, with ST command bit)
- `dg[1]` = attribute byte (Knauf calls this part of the header)
- `dg[2]` = key byte
- `dg[3]` = checksum = `0xFF - dg[2]`

The **0x40 bit** in `dg[2]` indicates a **long press**.  The single-key
code is `dg[2] & 0x3F` after stripping 0x40.  Knauf does not document the
checksum or the long-press bit.

*Source: instST_in.cpp, ST_AP_KEYSTROKE decode, comment
"This description supercedes Knauf's description".*


### 0x186 -- Long-press swap on DisplayTrack and MinusTenPlusTen

Two combined-key codes exhibit a counter-intuitive swap when long-pressed:

| Short press    | Value | Long-press output |
| -------------- | ----- | ----------------- |
| Display+Track  | 0x24  | 0x40 OR 0x28      |
| -10+10         | 0x28  | 0x40 OR 0x24      |

The ST7000 sends the other key's code OR'd with the long-press bit 0x40,
not its own code.  Any implementation that decodes long-press by simply
stripping 0x40 from the received byte will swap the meaning of these two
keys on a long press.

*Source: instST_in.cpp, keyName() function, comments on 0x24 and 0x28.*


### 0x185 -- NAV_TO_WP: undocumented checksum byte

Knauf's description of 0x185 (Navigation to Waypoint) shows 7 data bytes.
A confirmed 8th byte `dg[7]` is sent by real hardware and equals
`0xFF - dg[6]` (a checksum on the YF flag byte).

*Source: instST_out.cpp, apInst::sendSeatalk(), comment
"undocumented presumed checksum".*


### 0x1A4 -- Device ID 0xA8 mislabelled

Knauf lists device ID 0xA8 as "ST80 Masterview".  On the E80 device
enumeration bus (0x1A4 query/response), 0xA8 is actually the **ST40 Wind**
instrument.

*Source: instST_in.cpp, deviceName(), comment
"Knauf's description does not match my E80's Device Enumeration here".*


### 0x126 -- Log Speed: 0x04 validity flag is unreliable

Knauf says bit 0x04 of the flags byte `dg[6]` must be set for the speed
value in `dg[2..3]` to be valid.  In practice, real ST50 Speed instruments
send the speed with 0x04 clear (flags = 0x10) and the value is correct.

This library decodes the speed unconditionally.  The 0x08 flag in `dg[6]`
indicates a second speed field (average / dual-speed) in `dg[4..5]`.

*Source: instST_in.cpp, ST_LOG_SPEED decode, comment
"my flags are 0x10 and the speed appears valid even if !0x04".*


### 0x123 -- Water Temperature: Celsius+100 offset (pending verification)

Knauf's specification for 0x123 states `dg[2] = Degrees Celsius + 100`.
This library sends raw Celsius in `dg[2]` (no +100 offset).  The intent
is to send the temperature directly as an integer degree value.  Whether
real ST50 instruments require the +100 offset has not been fully verified
against hardware.  See also 0x127 below for an alternative higher-precision
encoding that does use an offset.

*Source: instST_out.cpp, logInst::sendSeatalk(), comment
"Knauf 0x23: dg[2] = Celsius+100, dg[3] = Fahrenheit".*


---

## Extensions to Knauf

### 0x186 -- Complete AP Keystroke key-code table

The following table extends Knauf's partial listing with all observed codes,
SHORT_BEFORE_LONG behavior, and operational notes gathered from ST7000
testing.  Key values are the base code (0x40 bit stripped for long press).

**Single keys** (all are modal; SHORT_BEFORE_LONG where noted):

| Code | Name     | Short Before Long | Notes                         |
| ---- | -------- | ----------------- | ----------------------------- |
| 0x01 | Auto     | yes               |                               |
| 0x02 | Standby  | no                |                               |
| 0x03 | Track    | no                |                               |
| 0x04 | Display  | no                | Long = backlight toggle cycle |
| 0x05 | -1       | yes               |                               |
| 0x06 | -10      | yes               |                               |
| 0x07 | +1       | yes               |                               |
| 0x08 | +10      | yes               |                               |
| 0x09 | -Resp    | yes               |                               |
| 0x0A | +Resp    | yes               |                               |

**Combined keys:**

| Code | Name             | Short Before Long | Notes                                  |
| ---- | ---------------- | ----------------- | -------------------------------------- |
| 0x20 | -1+1             | yes               | Autotack port                          |
| 0x21 | -1+10            | yes               |                                        |
| 0x22 | +1+10            | yes               | Autotack starboard                     |
| 0x23 | Standby+Auto     | yes               | Enter vane mode                        |
| 0x24 | Display+Track    | no                | Long sends 0x40 or 0x28 (see above)   |
| 0x25 | Standby-10       | yes               |                                        |
| 0x28 | -10+10           | no                | Long sends 0x40 or 0x24 (see above)   |
| 0x2E | -Resp+Resp       | long only         | Rudder Gain Adjustment (planing mode) |
| 0x30 | Standby-1        | yes               | Unreliable; can crash ST7000          |

Behavioral notes:
- **Display+Track short** enters a local contrast-adjust mode on the ST7000;
  Resp-/+ are consumed locally; exit with Display+Track short or Standby.
- **Standby+Auto long** returns to previous automatic heading (AP mode).
- **Rudder Gain Adjustment** (0x2E long) is for planing vessels only.
- No "factory reset" combination has been identified.


### 0x1A4 -- Extended device ID table

Knauf's table ends around 0x19.  The following IDs are confirmed from E80
device enumeration responses and match Raymarine's broader product line.
All entries after 0x19 are beyond Knauf's original table.

| ID   | Device name                    |
| ---- | ------------------------------ |
| 0x01 | Depth                          |
| 0x02 | Speed                          |
| 0x03 | Multi                          |
| 0x04 | Tridata                        |
| 0x05 | Tridata Repeater               |
| 0x06 | Wind                           |
| 0x07 | WMG                            |
| 0x08 | Navdata GPS                    |
| 0x09 | Maxview                        |
| 0x0A | Steering compass               |
| 0x0B | Windtrim                       |
| 0x0C | Speedtrim                      |
| 0x0D | Seatalk GPS                    |
| 0x0E | Seatalk Radar ST50             |
| 0x0F | Rudder Angle Indicator         |
| 0x10 | ST30 Wind                      |
| 0x11 | ST30 Bidata                    |
| 0x12 | ST30 Speed                     |
| 0x13 | ST30 Depth                     |
| 0x14 | LCD Navcenter                  |
| 0x15 | Apelco LCD Chartplotter        |
| 0x16 | Analog Speed                   |
| 0x17 | Analog Depth                   |
| 0x18 | ST30 Compass                   |
| 0x19 | ST50 NMEA Bridge               |
| 0x21 | E80 Analogue Compass           |
| 0x22 | E80 Analogue Multitrim         |
| 0x23 | E80 Analogue Wind              |
| 0x24 | E80 Analogue CH/Wind           |
| 0x25 | E80 Rudder Angle Indicator     |
| 0x26 | E80 Masterview                 |
| 0x27 | E80 Multiview                  |
| 0x28 | Rate Gyro Compass              |
| 0x2B | ST30 Round Bidata              |
| 0x2C | Navcenter 700                  |
| 0x2D | Inboard Autopilot              |
| 0x30 | Maxview Display Heads          |
| 0x41 | ST80 Remote Keypad             |
| 0x42 | ST80 MOB Button                |
| 0x43 | ST80 Autopilot                 |
| 0x44 | ST80 Masterkey                 |
| 0x51 | ST80 Active Speed              |
| 0x52 | ST80 Active Depth              |
| 0x53 | ST80 Active Deep Depth         |
| 0x54 | ST80 Active Wind               |
| 0x55 | ST80 Active Compass            |
| 0x56 | ST80 NMEA Bridge               |
| 0x60 | ST80 Course Computer           |
| 0x61 | SmartPilot                     |
| 0x70 | ST60 Speed                     |
| 0x71 | ST60 Depth                     |
| 0x72 | ST60 TriData                   |
| 0x73 | ST60 Wind Analogue Head        |
| 0x74 | ST60 CH/Wind Analogue Head     |
| 0x75 | ST60 Compass Analogue Head     |
| 0x76 | ST60 Multi                     |
| 0x77 | ST60 Maxview                   |
| 0x78 | ST60 Speed Sail RR             |
| 0x79 | ST60 Speed Power RR            |
| 0x7A | ST60 Depth Feet RR             |
| 0x7B | ST60 Depth Metres RR           |
| 0x7C | ST60 Rudder Angle Indicator RR |
| 0x7D | ST60 Navigator                 |
| 0x7E | ST60 Wind Round Rptr           |
| 0x7F | ST60 Compass Round Rptr        |
| 0x80 | ST6000 Control Unit            |
| 0x81 | ST7000 Control Unit            |
| 0x82 | ST60 GPS Head                  |
| 0x83 | ST60 Wired HHC                 |
| 0x84 | ST60 RF HHC                    |
| 0x85 | Raydata                        |
| 0x86 | ST60/80 RR Group Code          |
| 0x87 | Zodiac GPS                     |
| 0x88 | ST5/5000 (ST60)                |
| 0x89 | Raychart 620                   |
| 0x8A | Navcenter 600                  |
| 0x8B | ST60 Rudder Angle Indicator    |
| 0x90 | SL70 Radar Standalone          |
| 0x91 | SV7 Radar                      |
| 0x92 | RL70                           |
| 0x93 | RL70C                          |
| 0x94 | RC520                          |
| 0x95 | R70                            |
| 0x96 | R70RC                          |
| 0x97 | RL70/RL80C                     |
| 0x98 | RL70RC/RL80RC                  |
| 0x99 | RC530/RC631                    |
| 0xA0 | RS112LP GPS                    |
| 0xA1 | RS114 GPS                      |
| 0xA2 | Raychart 830                   |
| 0xA5 | ST40 Speed                     |
| 0xA6 | ST40 Depth                     |
| 0xA7 | ST40 Bidata                    |
| 0xA8 | ST40 Wind  (Knauf: ST80 Masterview -- INCORRECT) |
| 0xA9 | ST40 Compass                   |
| 0xB0 | L755                           |
| 0xB1 | L760                           |
| 0xB2 | L1250                          |
| 0xB3 | L1250RC                        |
| 0xB4 | L760                           |
| 0xC0 | RN300 Navigator                |
| 0xC1 | RN301 Navigator                |
| 0xC2 | RC320 Chartplotter             |
| 0xC3 | RC321 Chartplotter             |
| 0xC4 | RS120 GPS                      |
| 0xC5 | RS125 GPS                      |
| 0xC7 | C70 Display                    |
| 0xC8 | C80 Display                    |
| 0xC9 | C120 Display                   |
| 0xCA | E80 Display                    |
| 0xCB | E120 Display                   |
| 0xCC | Entry Level Pilot              |
| 0xCD | New ST4000                     |
| 0xCE | RF Base Station                |
| 0xCF | RF S100 FOB                    |
| 0xD0 | RF Smart Controller            |
| 0xD1 | ST8000 Control Unit (ST60)     |
| 0xD3 | GPM400 (US)                    |
| 0xD4 | GPM400 (EU)                    |
| 0xD5 | GPM400 (ROW)                   |


### 0x1A5 sub-messages: additional types not in Knauf

Knauf documents the A5 0D / 0C / 2D / 8D satellite-detail series.  Three
additional sub-message types are observed in bus traffic:

**A5 74 -- Additional PRN_USED values**

    A5 74 PU PU PU PU PU

Carries up to 5 additional PRN numbers used in the GPS solution, supplementing
the 4 PRN_USED values packed into the A5 0C message.  Each `PU` byte = PRN
number OR'd with 0x80 if that satellite is used in the solution.

**A5 98 -- End of satellite detail series**

    A5 98 00 00 00 00 00 00 00 00 00

Sent after the full A5 satellite detail series to signal completion.

**A5 B5 -- Datum specification**

    A5 B5 00 04 80 00 00 00

Explicitly specifies that the WGS-1984 datum is in use.


### 0x1A5 satellite PRN encoding: missing shift in Knauf text

Knauf's description of the A5 satellite position messages contains
ambiguous text for the PRN of the first satellite:
"Satellite PRN: [1] (NN&0xFE)>>1"

The `>>1` (right-shift by one, equivalent to dividing by 2) is correct
but easy to miss.  In the A5 series, PRN0 = `(NN & 0xFE) / 2`.  The
first satellite's PRN occupies the high 7 bits of the NN byte, with
the low bit used for other purposes.  Compare to 0x1A7 below where
the full byte is the PRN.

*Source: instST_in.cpp, decodeST() ST_SAT_DETAIL block, comment
"note the /2 missing from knaufs doc".*


### 0x1A7 -- Differential/SBAS detail: PRN is NOT shifted

The 0x1A7 SBAS/differential satellite message uses the same general
layout as the A5 satellite detail series, but the PRN byte NN is
**not shifted**:

    prn = dg[2]   (the entire byte, not (NN & 0xFE) / 2)

This is distinct from the A5 XQ series where the PRN occupies only the
high 7 bits of NN.

*Source: instST_in.cpp, decodeST() ST_DIF_DETAIL block.*


---

## New Datagrams (not in Knauf)

### 0x127 -- Water Temperature (Celsius, high precision)

A higher-precision water temperature datagram distinct from 0x123.

    27 Z1 LL HH

`HH:LL` is a 16-bit little-endian value where:
    temperature_celsius = (HH * 256 + LL - 100) / 10.0

This gives tenths-of-a-degree precision with a 100-unit offset to avoid
negative raw values.  Example: 25.0 C = raw value 350 = 0x015E.

Observed from ST50 Speed/Log instruments fitted with a temperature probe.


### 0x158 -- Combined Latitude and Longitude

A single datagram encoding both latitude and longitude.  Preferred by E80
and GPS devices over the separate 0x150 (Lat) and 0x151 (Lon) messages.

    58 Z5 LA LM mm LO OM om

- `dg[1]` flags: 0x10 = South latitude; 0x20 = East longitude (clear = West)
- `dg[2]` = integer degrees of latitude
- `dg[3]:dg[4]` = minutes * 1000 (big-endian uint16)
- `dg[5]` = integer degrees of longitude
- `dg[6]:dg[7]` = minutes * 1000 (big-endian uint16)


### 0x197 -- ST7000 Startup Probe

    97 00 00

Sent by the ST7000 autopilot head at power-on and under certain error/
key-combination conditions to determine whether an autopilot CPU is
present on the bus.  Not documented by Knauf.  The expected response is
0x198 from the autopilot CPU.

*Source: instST_in.cpp, decodeST() ST_ST7000 block.*


### 0x198 -- Autopilot CPU Acknowledge

    98 00 00

Sent by the autopilot CPU (e.g. Model 400G) in response to receiving the
0x197 probe from the ST7000 head.  Not documented by Knauf.

*Source: instST_in.cpp, decodeST() ST_AP_CPU block.*


### 0x159 and 0x161 -- E80 Constant Messages

Two messages observed consistently from the E80 whose purpose is not fully
understood:

- `59 11 CE FF` -- always this exact byte sequence from the E80.
- `61 03 03 00 00 00` -- always this exact byte sequence from the E80.

These appear to be periodic status or presence messages.  They are decoded
and named in the library but not further interpreted.


### 0x19E -- Waypoint Definition (tentative)

    9E ...

Observed once during E80 routing operation.  May be a waypoint definition
broadcast or an artifact of other message fragmentation.  Treat as
unconfirmed until additional examples are captured.


### 0x1AD -- Unknown

    AD ...

Observed on the Seatalk bus; origin and purpose unknown.  Logged by the
library as "AD" for further investigation.


---

**Next:** [Home](readme.md)
