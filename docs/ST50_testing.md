# Arduino Boat Library - ST50 Instrument Testing

**[Home](readme.md)** --
**[Architecture](architecture.md)** --
**[Boat Simulator](boatSimulator.md)** --
**[Instruments](instSimulator.md)** --
**[Commands](commands.md)** --
**[Connectors & Cables](pinouts.md)** --
**ST50 Testing** --
**[Seatalk Knowledge](new_st_knowledge.md)**

repos: **[phorton1](https://github.com/phorton1)** --
**[teensyBoat Firmware](https://github.com/phorton1/Arduino-boat-teensyBoat/blob/master/docs/readme.md)** --
**[teensyBoat App](https://github.com/phorton1/base-apps-teensyBoat/blob/master/docs/readme.md)** --
**Boat Library** --
**[tbESP32 WiFi](https://github.com/phorton1/Arduino-boat-tbESP32/blob/master/docs/readme.md)** --
**[teensyWind Tester](https://github.com/phorton1/Arduino-boat-teensyWind/blob/master/docs/readme.md)** --
**[teensyGPS](https://github.com/phorton1/Arduino-boat-teensyGPS/blob/master/docs/readme.md)**

Note that you can use "lamp=0..3" command in teensyBoat.ino to set the
lamp level on any connected ST50 instruments:

- ST50 Multi - works directly from my Seatalk simulator
- ST50 Speed - simple 5V pulse to emulate paddle wheel
- ST50 Wind  - 5V pulse to emulate wind speed, 2-8V orthognal signals for direction.
- ST50 Depth - 2 pin Piezo Driver and Listener


## ST50 Multi Instrument

The **ST50 Multi** is just a Seatalk display device and works directly
to/from either of teensyBoat's Seatalk ports.



## ST50 Depth

I had a somewhat miraculous lucky session attempting to spoof the ST50 Depth Head Unit.

After much discussion with coPilot AI about how Sonars work and what I was likely to
see on the 2 pin ST50 Depth Transducer connector we landed on first trying to visualize
and characterize the output "pulse" on an oscilloscope and then *perhaps* to figure out
a way to spoof it into "seeing" a transducer and sending Seatalk1 datagrams.  If the
head unit doesn't "see" a transducer the display flashes "0.0 Feet" and never sends
any Seatalk, so there was no way to know if an Instrment was working outside of an
in-vitro test.

I had at this point two head units, one with a bad LCD and one with a good LCD.

Pins, my selected colors, clockwise, facing instrument connector with tab on top

- GND, my green dupont to bottom of 10M/1M resistor divider
- PULSE, my yellow dupont to top of 10M/1M resistor divider

Starting with the good LCD head unit, Depth #2, I took the two pins from the transducer
via some jumper cable to a 10M/1M resistor divider network on a small breadboard.
I plugged the GND and probe of my cheap Fnirsi 2C23T handheld oscilloscope across
the 1M resistor likewise using short dupont jumpers clamped into the probe.
I powered up the ST50 Depth Head and ..

LO AND BEHOLD it stopped flashing "0.0 feet" and started showing a depth of
50-65 feet ... AND it output Seatalk messages!!!!

(note that Depth #3 output 10-14 feet)

The scop also clearly shows the 200khz original pulse being generated on the scope.

Without going into detail (I still have the coPilot conversation as a reference),
we surmised that there was enough stray capacitance in our circuit to fake the
ST50 head into seeing a "return" echo and reporting some number of feet.

I was then able to plug the Depth #0 with the bad LCD into that same setup,
and, never before able to verify if it was working or not, IT generated the same
pulse AND IT TOO output seatalk messags of depth in the 50-65 foot range.

Therefore I am able to conclude that it just has a bad LCD but that the electronics
are otherwise probably working fine.

BTW, the scope settings that worked were x10, AC coupling, 5-10V vertical divisions
and 2us (to see the wave form) to 5ms (to see the whole burst) horizontal divisions,
with "normal" triggering and the trigger set to about 1/2 division above zero.




## ST50 Speed Instrument

ST50 Speed (log) is a fairly simple device. It expects pulses as the log wheel turns.
For testing, I connect the device to ateensyBoat Seatalk port which provides it with
power and allows me to monitor its Seatalk output.

When facing the Instrument connector, with the blank position at the top, the
pins, and the presumed internal wire colors, are, clockwise, as follows:

- 1	**blue**	Temperature Sensor
- 2 **white**	Temperature Bias (~8V relative to ground)
- 3 **green**	Speed Pulse (active low, ~4.38V pulled to ground is a 'pulse')
- 4 **black**   Ground
- 5 **red**		Power (~5.0V to power transducer circutry)


The **green** pin #3 is pulled high to 5V by the instrument.  It is a simple matter
to pull it down to ground from a teensy GPIO 0 to 3.3V gpio pin through a 1K resistor
to the gate on a BC547 transistor to provide the square wave to provide the pulses
to the green pin.

The teensyBoat "S=speed through water" command is then used to output pulses at a
given frequency that drive the ST50 Speed Instrument to display the correct
value, and in turn, it should send the ST_LOG_SPEED and other Seatalk messages to a
teensyBoat Seatalk port where it can be monitored, displayed, and or forwarded to
the other Seatalk port. If the E80 is connected to the other teensyBoat Seatalk port,
and forwarding enabled, the E80 should display the Log information, providing further
confirmation.


### Observations

- a 10K resistor between pins 1 & 2 gives a termperature of 25.0 degrees
  Celcius displayed on the Speed Instrument, and Seatalk message, as per 'spec'
- I use an intervalTimer or PWM to output a square wave to a transistor that pulls
  the Green pin to ground.  With the Speed Instrument set to 1.00 calibration
  I found that one knot is approximately equal 5.6 pulses per second (Hertz)
  over the useful range.
- The ST50 Speed Instrument is specified to top out at 60 knots (around 310hz).
  It definitely did not work correctly at 90 knots (504hz),
  The problem is not the ST message encoding range, which would go up to 530kts,
  but rather the frequency detector and internals of the instrument.


---

**Next:** [Seatalk Knowledge](new_st_knowledge.md)
