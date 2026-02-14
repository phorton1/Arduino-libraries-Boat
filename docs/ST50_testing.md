#  ST50 Instrument Testing

Note that you can use "lamp=0..3" to set the lamp level on any connected ST50 instruments.

- ST50 Multi - works directly from my Seatalk simulator
- ST50 Speed - simple 5V pulse to emulate paddle wheel
- ST50 Wind  - 5V pulse to emulate wind speed, 2-8V orthognal signals for direction.
- ST50 Depth - 2 pin Piezo Driver and Listener


## ST50 Multi Instrument

The **ST50 Multi** is just a Seatalk display device and works directly
to/from either of teensyBoat's Seatalk ports.


## ST50 Speed Instrument

ST50 Speed (log) is a fairly simple device. It expects pulses as the log wheel turns.
For testing, I connect the device to the second teensyBoat Seatalk ports (PORT_ST2),
which provides it with power and allows me to monitor its output.

When facing the Instrument connector, with the blank position at the top, the
pins, and the presumed internal wire colors, are, clockwise, as follows:

- 1	**blue**	Temperature Sensor
- 2 **white**	Temperature Bias (~8V relative to ground)
- 3 **green**	Speed Pulse (active low, ~4.38V pulled to ground is a 'pulse')
- 4 **black**   Ground
- 5 **red**		Power (~5.0V to power transducer circutry)


The **green** pin #3 is pulled high to 5V by the instrument.  It is a simple matter
to pull it down to ground through a BC547 transistor in a square wave to provide
the pulses.

The teensyBoat "S=speed through water" command is then used to output pulses at a
given frequency that drive the ST50 Speed Instrument to display the correct
value, and in turn, it should send the ST_LOG_SPEED and other Seatalk messages to a
teensyBoat Seatalk port where it can be monitored, displayed, and or forwarded to
the other Seatalk port. If the E80 is connected to the other teensyBoat Seatalk port,
the E80 should display the Log information, providing further confirmation.


### Observations

- a 10K resistor between pins 1 & 2 gives a termperature of 25.0 degrees
  Celcius on the Speed Instrument, as per 'spec'
- I am using PWM to output a square wave to a transistor that pulls
  pin 3 to ground.  With the Speed Instrument set to 1.00 calibration
  I found that one knot is approximately equal 5.6 pulses per second (Hertz)
  over the useful range.
- The ST50 Speed Instrument is specified to top out at 60 knots (around 310hz).
  It definitely did not work correctly at 90 knots (504hz),
  The problem is not the ST message encoding range, which would go up to 530kts,
  but rather the frequency detector and internals of the instrument.



## ST50 Wind

Facing the INSTRUMENT connector (SOCKETS) with the notch at 6:00pm:

- 1 - Red @ 7 O'clock = 8V DC out
- 2 - Black/Shield @ 10 O'clock = ground
- 3 - Blue @ 12 O'clock = port wind direction
- 4 - Green @ 2 O'clock = starboard wind direction
- 5 - Yellow @ 5 O'clock = wind speed



Facing the VANE connector (PINS) with the notch at 6:00pm

- 1 - Yellow @ 7 o'clock = wind speed
- 2 - Blue @ 10 o'clock = port wind direction
- 3 - Green @ 12 o'clock = starboard wind direction
- 4 - Red @ 2 o'clock = 8VDC in
- 5 - Black/Shield @ 5 o'clock = ground

Note that most puslished descripitons of the VANE connector take the
point of view of looking at the SOCKETS on the mast mounted receptacle,
and not the PINS on the Wind Vane itself, and hence are backwards
from the my description above.



The ST50 Wind instrument is a bit more complicated. It provides the Vane with
GND and 8V, and expects a 0-5V square wave returned on the Yellow pin for the wind speed.
In addition it expects two 2-8V signals returne don the Green and Blue pins that determine
the direction as sine waves 90 degrees out of phase.

NOTE THAT UNTIL IT RECEIVES signals on the Green and Blue pins, the ST50 Wind Instrument
will not come alive. After that it start starts sending Seatalk messages which can
be monitored.

### Initial ST50 Wind Instrument Testing

My initial testing was to use the pulse output from the teensy to drive a similar
BC547 that pulled the Yellow to ground with the gate wired through 1K resistor to
a teensy GPIO pin. This worked more or less as expected (10 hz =~ 18 knots of wind
speed).


For the Green and Blue, I brought the Red 8V to a breadboard, connecting to
two voltage dividers formed by 10K potentiometers on the top, to a
a 3.3K resistor to ground, with the Green and Blue attached to the wipers.
Thus the wipers receive approx 2v to 8v depending on the pot setting.
This allowed me to bring the unit alive, and probe, but not accurately control,
the wind indicator.


### windTester Breadboard

After the Initial ST50 Wind Instrument tester breadboard, I decided to make
a circuit that could emulate the 2-8V 90 degree off phase inputs to the instrument
more accurately.

The windBreadboard is a dedicated bench tool that generates synthetic
wind vane signals for an ST50 Wind instrument using 2 PWM signals and a square wave
from teensyBoat's Teensy 4.0 over the GP8 general purpose connector and an additional
Setalk connector.

**2V to 8V outputs drive the Wind Instrument Angle Indicator**

Instead of pots or a real masthead unit, for the 2-8V signals, the Teensy outputs two
PWM channels, PWMA=Green and PWMB=Blue, which are converted into smooth analog voltages
in the 0–10.2V range using an LM358 dual opAmp.  The LM358 gnd/12V is powered from
the additional Seatalk connector, and the two of the opamp sections form identical
PWM to DC converters with gain and offset. Each PWM input is first filtered by a
by a 10kOhm/100nf RC network into the non-inverting input of each opamp. The
inverting input of each opamp is biased by a 10K resistor to ground and a 22K
resistor from the output, SIGNALA or SIGNALB to give the 0-10.2V working voltage range.
The firmware can then turn the SIGNALA/B off with a duty cycle of zero, or range
it from 2V to 8V with duty cycles constrained analogWrite() values of 50 to 194.
A firmware algorithm converts the teensyBoat's simulated WindAngle into the proper
phase shifted 2-8V valeus to cause the windInstrument to display the given angle.

**Driving the Speed pulse square wave

The square wave is well undertood. A signal (PULSE_SQUARE) comes from a teensy gpio
pin and is sent through a 1K resistor to the gate of a BC546 transistor.  The collector
of the transistor (WIND_SQUARE) is connected to the Wind Instrument's Yellow pin,
which is pulled up to 5V by the instrument, and the emitter of the transistor is
connected to GND. When the teensy outputs a PULSE_SQUARE=high signal, the gate opens
the transistor pulling the WIND_SQUARE 5V to ground, thus making the square wave.
A firmware algorithm convertts the teensyBoats simulated WindSpeed in knots
into the proper frequency in Hz to cause the Wind Instrument to display the given speed.



### Initial ST50 Vane Testing

I started very cautiously proving the pinouts with my bench supply
and a multi-meter starting on 3V and 10ma over current protection,
noticing I was getting signals on the Blue, Green, and Yellow
before proceeding to the full 8V and about 30ma current protection.

At 8V, as advertised, I saw values between 2v and 8v on the Blue
and Green pins, but only 0 to 1.4V or so on the Yellow. I determined
that the head unit supplying 5V is a pullup, probably through a 10K
resistor.

Soon after I made a circuit with a teensy4.0 that can read the
Blue and Green and turn them into an angle 0..360.  I have
not yet implemented the Yellow pulse sensing, but am sure I could.

To begin with I needed to generate an 8V rail. To the degree that
I can envision a teensyWind device that works with ST or NMEA2000,
both of whiich provide 12V low power, I decided to start by using
12V and a L708CV voltage regulator.  The vane draws very little
power so heat from the regulator is not a problem.

- Red to the outpupt leg of the L708CV
- Black to common 12V / teensy grounds
- Blue to a 15K/10K voltage divider to pull it down to teensy 3.3V ranges
- Green to a 15K/10K voltage divider to pull it down to teensy 3.3V ranges
- Yellow pulled up through a 10K resistor to the teeny's 3.3V regulator.

After first using the Arduino plotter to visualize the outputs, i then
switched to 12bit resolution and just went ahead and coded up the angle
math with some help from the coPilot AI.  It worked the first time and
is checked into my new Arduino-boat-teensyWind repo.


### A lot to think about

This gives me a LOT to think about as not only can I test the ST50 Wind
instrument, but am in a position to potentially replace it with a teensy
based solution that speaks Seatalk and/or NMEA2000.

The other side of the coin is that I want to create a "better" ST50 Wind
Instrument tester based on what i learned from the Vane, that can "peg"
a given wind direction and thus fit into my teensyBoat simulation architecture.



## ST50 Depth

I had a somewhat miraculous lucky session attempting to spoof the ST50 Depth Head Unit.

After much discussion with coPilot AI about how Sonars work and what I was likely to
see on the 2 pin ST50 Depth Transducer connector we landed on first trying to visualize
and characterize the output "pulse" on an oscilloscope and then *perhaps* to figure out
a way to spoof it into "seeing" a transducer and sending Seatalk1 datagrams.  If the
head unit doesn't "see" a transducer the display flashes "0.0 Feet" and never sends
any Seatalk, so there was no way to know if it was working.

I had at this point two head units, one with a bad LCD and one with a good LCD.

Starting with the good LCD head unit, I took the two pins from the transducer
via some jumper cable to a 10M/1M resistor divider network on a small breadboard.
I plugged the GND and probe of my cheap Fnirsi 2C23T handheld oscilloscope across
the 1M resistor likewise using short dupont jumpers clamped into the probe.
I powered up the ST50 Depth Head and ..

LO AND BEHOLD it stopped flashing "0.0 feet" and started showing a depth of
50-65 feet ... AND it output Seatalk messages!!!!

It also clearly showed the 200khz original pulse being generated on the scope.

Without going into detail (I still have the coPilot conversation as a reference),
we surmised that there was enough stray capacitance in our circuit to fake the
ST50 head into seeing a "return" echo and reporting some number of feet.

I was then able to plug the head unit with the bad LCD into that same setup,
and never before able to verify if it was working or not, IT generated the same
pulse AND IT TOO output seatalk messags of depth in the 50-65 foot range.

Therefore I am able to conclude that it just has a bad LCD but that the electronics
are otherwise probably working fine.

BTW, the scope settings that worked were x10, AC coupling, 5-10V vertical divisions
and 2us (to see the wave form) to 5ms (to see the whole burst) horizontal divisions,
with "normal" triggering and the trigger set to about 1/2 division above zero.
