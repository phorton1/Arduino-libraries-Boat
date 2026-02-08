#  ST50 Instrument Testing

Note that you can use "lamp=0..3" to set the lamp level on any connected ST50 instruments.


## ST50 Multi Instrument

The **ST50 Multi** is just a Seatalk display device and works directly
to/from either of teensyBoat's Seatalk ports.


## ST50 Speed Instrument

ST50 Speed (log) is a fairly simple device. It expects pulses as the log wheel turns.
For testing, I connect the device to the second teensyBoat Seatalk ports (PORT_ST2),
which provides it with power and allows me to monitor its output.

The **green** pin #3 is pulled high to 5V by the instrument.  It is a simple matter
to pull it down to ground through a BC547 transistor in a square wave to provide
the pulses.

The teensyBoat "S=speed through water" command is then used to output pulses at a
given frequency that drive the ST50 Speed Instrument to display the correct
value, and in turn, it should send the ST_LOG_SPEED and other Seatalk messages to a
teensyBoat Seatalk port where it can be monitored, displayed, and or forwarded to
the other Seatalk port. If the E80 is connected to the other teensyBoat Seatalk port,
the E80 should display the Log information, providing further confirmation.


### ST50 Speed Instrument Connector Pinouts

The ST50 Speed Instrument sensor male connector has five pins and uses a
a proprietary locking scheme similar to a 240-degree DIN type B connector.

When facing the connector, with the blank position at the top, the
pins, and the presumed internal wire colors, are, clockwise, as follows:

- 1	**blue**	Temperature Sensor
- 2 **white**	Temperature Bias (~8V relative to ground)
- 3 **green**	Speed Pulse (active low, ~4.38V pulled to ground is a 'pulse')
- 4 **black**   Ground
- 5 **red**		Power (~5.0V to power transducer circutry)

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

ST50 Wind: With Keyway at Bottom (Solder Side)
Yellow @ 7 O'clock
Green @ 10 O'clock
Blue @ 12 O'clock
Ground @ 2 O'clock
Red @ 5 O'clock

The correct order per notes here and from Raymarine should be clockwise from the notch at 6:00pm:
y=wind speed, 7:00pm
g=starboard wind direction, 10:00pm
blue=port wind direction, 12:00pm
shield=ground, 2:00pm
r=power, 5:00pm

the actual order at the masthead (thank you marina after working on it!):
y, blue, g, r, shield







I then modified the breadboard, and the program, slightly, to allow me to
do some rudimentary testing of a few of the ST50 instruments.

- ST50 Multi - works directly from my Seatalk simulator
- ST50 Speed - simple 5V pulse to emulate paddle wheel
- ST50 Wind  - 5V pulse to emulate wind speed, 2-8V orthognal signals for direction.


The ST50 Wind instrument is a bit more complicated. In addition to a pulse to emulate
the wind speed, it expects two 2-8V signals that determine the direction.  So I created
a voltage divider from 12V to 8V and used a pair of potentiometers in another set of
voltage dividers to allow me to send 2V to 8V to the two direction signal pins on the
ST50 Wind instrument.  By changing the pots I got the instrument to display various
directions and verified, basically, that the instrument was working. What I mostly wanted
to know is the wind speed though, to see if the LCD was bad (which it was).

In an interesting side experiment I tore the polarization filter off of an old cheap
tablet LCD, turned it at angle, and was able to barely see the ST50 Wind LCD was working,
just old and weak.


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
