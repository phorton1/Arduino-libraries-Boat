#  ST50 Instrument Testing

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

Note that most published descripitons of the VANE connector take the
point of view of looking at the SOCKETS on the mast mounted receptacle,
and not the PINS on the Wind Vane itself, and hence are backwards
from the my description above.


The ST50 Wind instrument is a bit more complicated than the Speed/Log instrument.
It provides the Vane with GND and 8V, and like the Speed/Log instrument expects a
0-5V square wave returned on the Yellow pin for the wind speed.  However,
in addition it expects two 2-8V signals returned on the Green and Blue pins
that determine the direction of the wind, delivered as two sine waves 90 degrees
out of phase.

NOTE THAT UNTIL IT RECEIVES signals on the Green and Blue pins, the ST50 Wind Instrument
will not "come alive". After that it start starts displaying an angle on the angle indicator,
an apparent wind speed on the LCD, and sending Seatalk messages which can be monitored.


### Initial ST50 Wind Instrument tester breadboard

My initial testing was to use the pulse output from the teensy to drive a
BC547 that pulled the Yellow to ground with the gate wired through 1K resistor to
a teensy GPIO pin just like on the the Speed/Log instrument. This worked more or
less as expected (10 hz =~ 18 knots of wind speed).

For the Green and Blue, I brought the Red 8V to a breadboard, connected to
two voltage dividers formed by 10K potentiometers on the top, to a
a 3.3K resistor on the bottom (to ground), with the Green and Blue attached
to the wipers.

Thus the wipers receive approx 2v to 8v depending on the pot setting.
This allowed me to bring the unit alive, and probe, but not accurately control,
the angle indicator on the Wind Instrument.


### windTester Breadboard started

After the Initial ST50 Wind Instrument tester breadboard, I decided to make
a circuit that could emulate the 2-8V 90 degree off phase inputs to the instrument
more accurately.

The windBreadboard uses 2 PWM signals through a dual LM358 opAmp to deliver
from 0V to about 10.2V at the opAmp output pin, along with the familiar
"pull-to-ground" BC547 transister for the square wave. By varying the
duty cycle of the PWM I can achieve various voltages at the opAmp output
pin, so I characterized the duty cycle for 2V and 8V, respectively, encoded
in #define constants in the boat library instSimulator.cpp file.

Then, with coPilots help, I developed an implementation which converts teensyBoat's
simulated Apparent Wind Angle into the dual PWM levels using an arbitrary "ellipse".

It did not work very well with a real instrument.  It was complicated by the
fact that the outputs at the opAmp are changed somewhat significantly when you
attach them to the Instruments Blue and Green pins, as the instrument itself
appears to affect the biases on the opAmp

*NOTE:  This table does not look correct. I NEVER sent duty cycles
of 1 to 90% PWM to the opamp and into the instrument as that would
have resulted in 10.2V to the instrument, something I avoided at all
costs.  Therefore I *think* this is a table of the percents across my
2V to 8V duty cycle range, but I dont remember now as I am cleaning
up this doc. Use with caution!*

PWM   Vblue_no_inst   Vgreen_no_inst   Vblue_inst   Vgreen_inst
----------------------------------------------------------------
  1%       1.53 V          1.53 V         1.33 V        1.33 V
  2%       1.58 V          1.58 V         1.38 V        1.38 V
  5%       1.73 V          1.73 V         1.53 V        1.53 V
 10%       1.98 V          1.98 V         1.78 V        1.78 V
 20%       2.48 V          2.48 V         2.28 V        2.28 V
 30%       2.98 V          2.98 V         2.78 V        2.78 V
 40%       3.48 V          3.48 V         3.28 V        3.28 V
 50%       3.98 V          3.98 V         3.78 V        3.78 V
 60%       4.48 V          4.48 V         4.28 V        4.28 V
 70%       4.98 V          4.98 V         4.78 V        4.78 V
 80%       5.48 V          5.48 V         5.28 V        5.28 V
 90%       5.98 V          5.98 V         5.78 V        5.78 V

So a 1K series resistor was added after the opAmp output and into the pins
to somewhat isolate the opAmp from the instrument.  Still it did not work
very well, and thus I decided, in order to know what voltages to send, and
hence what PWM duty cycles to use, it was neccessary to first more carefully
characterize the outputs from an actual ST50 Wind Tranducer connected to
an actual ST50 Wind Instrument (later)


### Initial ST50 Vane (Transducer) Testing - teensyWind.ino and breadboard

Since I only have one, very expensive, Transducer available for testing,
I started very cautiously, providing the documented Red and Black pins with
my bench supply set to 3V with only 10ma over current protection, and a
a multimeter to probe any signals present on the Blue, Green, or Yellow
pins. Once I noticed I was getting signals on the Blue, Green, and Yellow
pins that made sense, I proceeded to the full 8V at about 30ma of current
protection on the bench supply.

At 8V, as advertised, I saw values between 2v and 8v on the Blue and Green
pins as I rotated the direction vane, but only 0 to 1.4V or so on the Yellow
as I rotated the speed cups. I determined that the head unit must be supplying
a pullup to 5V on the yellow pin.

So, soon after, I made a breadboard with a teensy4.0 that used its 5V (vIn) to
pullup the Yellow through a 10K resistor. This provided me with a solid 0.6 to
5V signal on the Yellow as I rotated the speed cups.

By now I began to envision a teensyWind device that delivers ST or NMEA2000,
both of whiich provide 12V low power, so I decided to start by using
12V to the breadboard and a L708CV voltage regulator to provide the 8V
to the transducer.  The vane draws very little power so heat from the
regulator is not a problem.  I quickly came up with the following overall
circuit.

- Red to the outpupt leg of the L708CV 8V voltage regulator
- Black to common 12V / teensy grounds
- Blue to a 15K/10K voltage divider to pull it down to teensy 3.3V ranges
- Green to a 15K/10K voltage divider to pull it down to teensy 3.3V ranges
- Yellow pulled up through a 10K resistor to the teeny's 3.3V regulator.

It turns out that since the teensy will be reading the Yellow pulse,
I might as well only pull it up to 3.3V rather than 5V like the real
Wind Instrument.

After first using the Arduino plotter to visualize the outputs, i then
switched to 12bit resolution and just went ahead and coded up the angle
math with some help from the coPilot AI.  It uses an idealized arbitrary
"ellipse" for the 2 out of phase input signals and produces an angle.
Surprisingly it worked after only a few tweaks, reports 0 and 180 degrees
(two positions on the Transducer I can somewhat reliabably position it in)
fairly accurately, and produces a continuous 0 thru 360 degrees as I rotate
the Vane.  It is probably not accurate to within 1 degree (yet), but
nonetheless I was pretty satisfied with this effort and enthusiastically
checked it in as a new Arduino-boat-teensyWind repo.

This gives me a LOT to think about as not only can I test the ST50 Wind
instrument, but am in a position to potentially replace it with a teensy
based solution that speaks Seatalk and/or NMEA2000.



### Discovery of Wind Instrument "Repeater Mode"

I decided here to inventory my ST50 Wind Instrument heads.  I have
four of them (W0 through W3) at this time.  Noticing that W0, the
one I removed from the boat, has a button labelled "CAL", where the
other three have that button labelled "TRU/APP" I tried to resolve
the difference by finding the relevant presumed different user manuals
for the presumed different devices.

In the end I came to the conclusion that the buttons and electronics
are essentially identical, but that Raymarine merely labelled the oldest
units (and the documentation) with the "CAL" name, but later switched
it to say "TRU/APP". In this process, I more carefully read the docs
and realized, for the first time, that the ST50 Wind Instrument can
**ACT AS A REPEATER**, that is to say that it can **ACCEPT SEATALK
DATAGRAMS** and **display** the corresponding **Wind Angle and Speed**.

Had I realized this sooner I might not have gone down this whole path
of providing spoofed transducer inputs to the instrument as before this
that was the only way I had of making the unit "come alive" and verifying
that it was basically working, or not.

However, now armed with this information, I was able to quickly characterize
all four of my instruments as to how well they were able to display,
on the angle indicator and LED, the information I sent them over Seatalk.
And note that since teensyBoat can also send simulated Heading (Compass
instrumaent) and Water Speed (Speed/Log instrument) ST datagrams, that
the instruments can now display their calculated True Wind Speed and
Angle as well.

This allowed me to characterize my W1 instrument as the "best" unit
in terms of displaying the wind angle most accurately and having a
nice dark readable LCD for the speed/calibration functions.




### Wind Vane direct to Instrument testing

At this point I determined that what was important was to measure the
actual Angles and Blue/Green Voltages on a working Vane/Instrument setup
to characterize the "ellipse" that the working combination presents on
the Blue and Green pins.

Let me preface this by saying that by now I had inventoried my ST50 Wind
Instruments and chosen one (W1) as my "Reference" Instrument and that I
am using an expensive "Refurbished" Wind Vane on my desk as the Reference
Transducer, and that I have calibrated W1 so that it appears to accurately
indicate (and transmit Seatalk) at 0 and 180 degrees (the only positions I
can somewhat reliably position the Transducer Vane at) as well as reporting
the full circle with good approximations of 90 and 270 degrees.

So, in this series of tests, I hook the Vane up to the Instrument
on my desk, through a blank breadboard and jumper connectors so that I can see
how they interact and measure the actual voltages on the Blue and Green
pins in-vitro.

I am able to "see" what the Instrument thinks is the angle by it's output
ST messages, so I am able to position the Vane more or less presicly at
15 degree intervals before taking the voltage meaasurements.

Measured blue/green voltages by reported angle (Vane -> Instrument)

Angle	Blue	Green
---------------------
0 		4.95V	2.62V
15		4.54V	2.44V
30		4.13V	2.40V
45		3.70V	2.43V
60		3.32V	2.54V
75		2.92V	2.75V
90		2.62V	3.05V
105		2.48V	3.42V
120		2.37V	3.85V
135		2.42V	4.26V
150		2.57V	4.65V
165		2.79V	4.99V
180		3.08V	5.25V
195		3.48V	5.47V
210		3.86V	5.56V
225		4.28V	5.56V
240		4.68V	5.44V
255		5.03V	5.22V
270		5.44V	4.67V
285		5.57V	4.27V
300		5.61V	3.87V
315		5.54V	3.46V
330		5.35V	3.05V
345		5.07V	2.72V
359		4.96V	2.64V


### windTester breadboard Angle Summary

Armed with the above table I was able to work with coPilot to
refine the algorithm in instSimulator.cpp to output PWM values
to convert the boatSimulator's Apparent Wind Angle into values
that are now, more or less accurately (off by 1-3 degrees max)
indicated and reported by Seatalk on the Reference W1 instrument.

It actually took almost a full day of curve fitting, additional
measurements and work with the Reference Instrument to acheive this result.
There is a long comment in the code regarding this effort.

Now that I have the Reference Wind Instrument Calibrated to the Reference
Transducer and tha ability to drive the Reference Wind Instrument to fairly
precise Angles, I *could* henceforth **Calibrate the remaining Wind Units
to the windTester breadboard** and they *should* be reasonably closely
calibrated to the the

Also note that there is a single constant in the algorithm,

	#define ANGLE_OFFSET	18

that characterizes the Reference Wind Vane's offset.
coPilot has led me to believe that **we have more or less captured
the internal "ellipse" used generically by all instances of the
Wind Instrument**, and that the above constant *might* have to
be changed, much as those instruments might have to be calibrated
differently, should a different Transducer be introduced into
the equation.  Subsequent versions of the teensyBoat.ino program
(and Boat library) *might* make this a configurable parameter
stored in teensyBoat's EEPROM.


### The next day DISASTER ON ELM STREET turns into Instrument Ellipse Self Calibration identified

Sigh, the next morning I booted up teensyBoat.ino, the windTester,
and the reference W1 Instrument with the same exact bench supply,
wiring, breadboard, firmware and everything, and the algorithm
for converting the simulated Wind Angle into PWM values and driving
the instrument **utterly failed**.   For reference, here is the debug
output that shows how bad the algorithm "achieved" the desired angles:

WARNING - angle(0.0)  deg(342.0)  theta(5.9690) c(0.9511) s(-0.3090) cr(0.8090) sr(-0.5878) ex(1.5978) ey(-0.5191) Vb(4.9475) Vg(2.6109)
WARNING -     angle(0.0)  pwma_green=59 pwmb_blue=113
ACHIEVED ST_WIND_ANGLE=342.00
ACHIEVED ST_WIND_ANGLE=341.00
WARNING - angle(15.0)  deg(327.0)  theta(5.7072) c(0.8387) s(-0.5446) cr(0.8090) sr(-0.5878) ex(1.4090) ey(-0.9150) Vb(4.5621) Vg(2.4016)
WARNING -     angle(15.0)  pwma_green=55 pwmb_blue=104
ACHIEVED ST_WIND_ANGLE=3.00
WARNING - angle(30.0)  deg(312.0)  theta(5.4454) c(0.6691) s(-0.7431) cr(0.8090) sr(-0.5878) ex(1.1241) ey(-1.2485) Vb(4.1356) Vg(2.2992)
WARNING -     angle(30.0)  pwma_green=52 pwmb_blue=94
ACHIEVED ST_WIND_ANGLE=27.00
WARNING - angle(45.0)  deg(297.0)  theta(5.1836) c(0.4540) s(-0.8910) cr(0.8090) sr(-0.5878) ex(0.7627) ey(-1.4969) Vb(3.6972) Vg(2.3107)
WARNING -     angle(45.0)  pwma_green=53 pwmb_blue=84
ACHIEVED ST_WIND_ANGLE=49.00
WARNING - angle(60.0)  deg(282.0)  theta(4.9218) c(0.2079) s(-0.9781) cr(0.8090) sr(-0.5878) ex(0.3493) ey(-1.6433) Vb(3.2767) Vg(2.4352)
WARNING -     angle(60.0)  pwma_green=55 pwmb_blue=75
ACHIEVED ST_WIND_ANGLE=69.00
WARNING - angle(75.0)  deg(267.0)  theta(4.6600) c(-0.0523) s(-0.9986) cr(0.8090) sr(-0.5878) ex(-0.0879) ey(-1.6777) Vb(2.9027) Vg(2.6644)
WARNING -     angle(75.0)  pwma_green=61 pwmb_blue=66
ACHIEVED ST_WIND_ANGLE=94.00
WARNING - angle(90.0)  deg(252.0)  theta(4.3982) c(-0.3090) s(-0.9511) cr(0.8090) sr(-0.5878) ex(-0.5191) ey(-1.5978) Vb(2.6009) Vg(2.9825)
WARNING -     angle(90.0)  pwma_green=68 pwmb_blue=59
ACHIEVED ST_WIND_ANGLE=115.00
WARNING - angle(105.0)  deg(237.0)  theta(4.1364) c(-0.5446) s(-0.8387) cr(0.8090) sr(-0.5878) ex(-0.9150) ey(-1.4090) Vb(2.3916) Vg(3.3679)
WARNING -     angle(105.0)  pwma_green=77 pwmb_blue=54
ACHIEVED ST_WIND_ANGLE=134.00
WARNING - angle(120.0)  deg(222.0)  theta(3.8746) c(-0.7431) s(-0.6691) cr(0.8090) sr(-0.5878) ex(-1.2485) ey(-1.1241) Vb(2.2892) Vg(3.7944)
WARNING -     angle(120.0)  pwma_green=86 pwmb_blue=52
ACHIEVED ST_WIND_ANGLE=149.00
WARNING - angle(135.0)  deg(207.0)  theta(3.6128) c(-0.8910) s(-0.4540) cr(0.8090) sr(-0.5878) ex(-1.4969) ey(-0.7627) Vb(2.3007) Vg(4.2328)
WARNING -     angle(135.0)  pwma_green=96 pwmb_blue=52
ACHIEVED ST_WIND_ANGLE=163.00
WARNING - angle(150.0)  deg(192.0)  theta(3.3510) c(-0.9781) s(-0.2079) cr(0.8090) sr(-0.5878) ex(-1.6433) ey(-0.3493) Vb(2.4252) Vg(4.6533)
WARNING -     angle(150.0)  pwma_green=106 pwmb_blue=55
ACHIEVED ST_WIND_ANGLE=172.00
WARNING - angle(165.0)  deg(177.0)  theta(3.0892) c(-0.9986) s(0.0523) cr(0.8090) sr(-0.5878) ex(-1.6777) ey(0.0879) Vb(2.6544) Vg(5.0273)
WARNING -     angle(165.0)  pwma_green=115 pwmb_blue=60
ACHIEVED ST_WIND_ANGLE=134.00
WARNING - angle(180.0)  deg(162.0)  theta(2.8274) c(-0.9511) s(0.3090) cr(0.8090) sr(-0.5878) ex(-1.5978) ey(0.5191) Vb(2.9725) Vg(5.3291)
WARNING -     angle(180.0)  pwma_green=121 pwmb_blue=68
ACHIEVED ST_WIND_ANGLE=154.00
WARNING - angle(195.0)  deg(147.0)  theta(2.5656) c(-0.8387) s(0.5446) cr(0.8090) sr(-0.5878) ex(-1.4090) ey(0.9150) Vb(3.3579) Vg(5.5384)
WARNING -     angle(195.0)  pwma_green=126 pwmb_blue=76
ACHIEVED ST_WIND_ANGLE=178.00
WARNING - angle(210.0)  deg(132.0)  theta(2.3038) c(-0.6691) s(0.7431) cr(0.8090) sr(-0.5878) ex(-1.1241) ey(1.2485) Vb(3.7844) Vg(5.6408)
WARNING -     angle(210.0)  pwma_green=129 pwmb_blue=86
ACHIEVED ST_WIND_ANGLE=205.00
WARNING - angle(225.0)  deg(117.0)  theta(2.0420) c(-0.4540) s(0.8910) cr(0.8090) sr(-0.5878) ex(-0.7627) ey(1.4969) Vb(4.2228) Vg(5.6293)
WARNING -     angle(225.0)  pwma_green=128 pwmb_blue=96
ACHIEVED ST_WIND_ANGLE=231.00
WARNING - angle(240.0)  deg(102.0)  theta(1.7802) c(-0.2079) s(0.9781) cr(0.8090) sr(-0.5878) ex(-0.3493) ey(1.6433) Vb(4.6433) Vg(5.5048)
WARNING -     angle(240.0)  pwma_green=125 pwmb_blue=106
ACHIEVED ST_WIND_ANGLE=258.00
WARNING - angle(255.0)  deg(87.0)  theta(1.5184) c(0.0523) s(0.9986) cr(0.8090) sr(-0.5878) ex(0.0879) ey(1.6777) Vb(5.0173) Vg(5.2756)
WARNING -     angle(255.0)  pwma_green=120 pwmb_blue=114
ACHIEVED ST_WIND_ANGLE=279.00
WARNING - angle(270.0)  deg(72.0)  theta(1.2566) c(0.3090) s(0.9511) cr(0.8090) sr(-0.5878) ex(0.5191) ey(1.5978) Vb(5.3191) Vg(4.9575)
WARNING -     angle(270.0)  pwma_green=113 pwmb_blue=121
ACHIEVED ST_WIND_ANGLE=300.00
WARNING - angle(285.0)  deg(57.0)  theta(0.9948) c(0.5446) s(0.8387) cr(0.8090) sr(-0.5878) ex(0.9150) ey(1.4090) Vb(5.5284) Vg(4.5721)
WARNING -     angle(285.0)  pwma_green=104 pwmb_blue=126
ACHIEVED ST_WIND_ANGLE=258.00
WARNING - angle(300.0)  deg(42.0)  theta(0.7330) c(0.7431) s(0.6691) cr(0.8090) sr(-0.5878) ex(1.2485) ey(1.1241) Vb(5.6308) Vg(4.1456)
WARNING -     angle(300.0)  pwma_green=94 pwmb_blue=128
ACHIEVED ST_WIND_ANGLE=269.00
WARNING - angle(315.0)  deg(27.0)  theta(0.4712) c(0.8910) s(0.4540) cr(0.8090) sr(-0.5878) ex(1.4969) ey(0.7627) Vb(5.6193) Vg(3.7072)
WARNING -     angle(315.0)  pwma_green=84 pwmb_blue=128
ACHIEVED ST_WIND_ANGLE=284.00
WARNING - angle(330.0)  deg(12.0)  theta(0.2094) c(0.9781) s(0.2079) cr(0.8090) sr(-0.5878) ex(1.6433) ey(0.3493) Vb(5.4948) Vg(3.2867)
WARNING -     angle(330.0)  pwma_green=75 pwmb_blue=125
ACHIEVED ST_WIND_ANGLE=299.00
WARNING - angle(345.0)  deg(357.0)  theta(6.2308) c(0.9986) s(-0.0523) cr(0.8090) sr(-0.5878) ex(1.6777) ey(-0.0879) Vb(5.2656) Vg(2.9127)
WARNING -     angle(345.0)  pwma_green=66 pwmb_blue=120
ACHIEVED ST_WIND_ANGLE=321.00
WARNING - angle(0.0)  deg(342.0)  theta(5.9690) c(0.9511) s(-0.3090) cr(0.8090) sr(-0.5878) ex(1.5978) ey(-0.5191) Vb(4.9475) Vg(2.6109)
WARNING -     angle(0.0)  pwma_green=59 pwmb_blue=113
ACHIEVED ST_WIND_ANGLE=342.00
ACHIEVED ST_WIND_ANGLE=342.00

Now coPilot is telling me that the likely reason for this huge shift is that the impedance of my
PWM->opamp->series resitor->blue/green signal circuit does not match the real transducer and that
the Instrument is likely extremely senstive to the **IMPEDANCE** of the circuit, not just the voltages.
There followed a pass where I rehooked up the transducer, measure resistances and ohms, that didnt
really teach me anything.

Then on a hunch I connected it to the instrument again, and slowly rotated the vane two times
and did measurements based on the physical angle I set on the vane and what the instrument reported
and, lo and behold, it got "better".

The conclusion is that the Wind Instrument itself actually characterizes the "ellipse" by
continous measurements and approximations of the centers and amplitude of the Blue/Green
Elipses.

To further prove this notion, I tooke the Vane out, re-attached the windTester breadboard
and modified my "automatic QUICK_TEST" loop to first do two quick (90 seconds) circles
at 2 degree incrments every 500 ms before doing its 15 degree every 20 seconds measurement
stage.

**THE RESULT WAS ASTOUNDING** ... after this step the unit got back to reporting angles
within 1 or 2 degrees of what I sent it:

---------------- CIRCLE(2) COMPLETED -------------
ACHIEVED ST_WIND_ANGLE=359.00
WARNING - angle(15.0)  deg(327.0)  theta(5.7072) c(0.8387) s(-0.5446) cr(0.8090) sr(-0.5878) ex(1.4090) ey(-0.9150) Vb(4.5621) Vg(2.4016)
WARNING -     angle(15.0)  pwma_green=55 pwmb_blue=104
ACHIEVED ST_WIND_ANGLE=14.00
WARNING - angle(30.0)  deg(312.0)  theta(5.4454) c(0.6691) s(-0.7431) cr(0.8090) sr(-0.5878) ex(1.1241) ey(-1.2485) Vb(4.1356) Vg(2.2992)
WARNING -     angle(30.0)  pwma_green=52 pwmb_blue=94
ACHIEVED ST_WIND_ANGLE=30.00
WARNING - angle(45.0)  deg(297.0)  theta(5.1836) c(0.4540) s(-0.8910) cr(0.8090) sr(-0.5878) ex(0.7627) ey(-1.4969) Vb(3.6972) Vg(2.3107)
WARNING -     angle(45.0)  pwma_green=53 pwmb_blue=84
ACHIEVED ST_WIND_ANGLE=45.00
WARNING - angle(60.0)  deg(282.0)  theta(4.9218) c(0.2079) s(-0.9781) cr(0.8090) sr(-0.5878) ex(0.3493) ey(-1.6433) Vb(3.2767) Vg(2.4352)
WARNING -     angle(60.0)  pwma_green=55 pwmb_blue=75
ACHIEVED ST_WIND_ANGLE=59.00
WARNING - angle(75.0)  deg(267.0)  theta(4.6600) c(-0.0523) s(-0.9986) cr(0.8090) sr(-0.5878) ex(-0.0879) ey(-1.6777) Vb(2.9027) Vg(2.6644)
WARNING -     angle(75.0)  pwma_green=61 pwmb_blue=66
ACHIEVED ST_WIND_ANGLE=75.00
WARNING - angle(90.0)  deg(252.0)  theta(4.3982) c(-0.3090) s(-0.9511) cr(0.8090) sr(-0.5878) ex(-0.5191) ey(-1.5978) Vb(2.6009) Vg(2.9825)
WARNING -     angle(90.0)  pwma_green=68 pwmb_blue=59
ACHIEVED ST_WIND_ANGLE=89.00
WARNING - angle(105.0)  deg(237.0)  theta(4.1364) c(-0.5446) s(-0.8387) cr(0.8090) sr(-0.5878) ex(-0.9150) ey(-1.4090) Vb(2.3916) Vg(3.3679)
WARNING -     angle(105.0)  pwma_green=77 pwmb_blue=54
ACHIEVED ST_WIND_ANGLE=105.00
WARNING - angle(120.0)  deg(222.0)  theta(3.8746) c(-0.7431) s(-0.6691) cr(0.8090) sr(-0.5878) ex(-1.2485) ey(-1.1241) Vb(2.2892) Vg(3.7944)
WARNING -     angle(120.0)  pwma_green=86 pwmb_blue=52
ACHIEVED ST_WIND_ANGLE=118.00
WARNING - angle(135.0)  deg(207.0)  theta(3.6128) c(-0.8910) s(-0.4540) cr(0.8090) sr(-0.5878) ex(-1.4969) ey(-0.7627) Vb(2.3007) Vg(4.2328)
WARNING -     angle(135.0)  pwma_green=96 pwmb_blue=52
ACHIEVED ST_WIND_ANGLE=133.00
WARNING - angle(150.0)  deg(192.0)  theta(3.3510) c(-0.9781) s(-0.2079) cr(0.8090) sr(-0.5878) ex(-1.6433) ey(-0.3493) Vb(2.4252) Vg(4.6533)
WARNING -     angle(150.0)  pwma_green=106 pwmb_blue=55
ACHIEVED ST_WIND_ANGLE=149.00
WARNING - angle(165.0)  deg(177.0)  theta(3.0892) c(-0.9986) s(0.0523) cr(0.8090) sr(-0.5878) ex(-1.6777) ey(0.0879) Vb(2.6544) Vg(5.0273)
WARNING -     angle(165.0)  pwma_green=115 pwmb_blue=60
ACHIEVED ST_WIND_ANGLE=163.00
WARNING - angle(180.0)  deg(162.0)  theta(2.8274) c(-0.9511) s(0.3090) cr(0.8090) sr(-0.5878) ex(-1.5978) ey(0.5191) Vb(2.9725) Vg(5.3291)
WARNING -     angle(180.0)  pwma_green=121 pwmb_blue=68
ACHIEVED ST_WIND_ANGLE=178.00
WARNING - angle(195.0)  deg(147.0)  theta(2.5656) c(-0.8387) s(0.5446) cr(0.8090) sr(-0.5878) ex(-1.4090) ey(0.9150) Vb(3.3579) Vg(5.5384)
WARNING -     angle(195.0)  pwma_green=126 pwmb_blue=76
ACHIEVED ST_WIND_ANGLE=193.00
WARNING - angle(210.0)  deg(132.0)  theta(2.3038) c(-0.6691) s(0.7431) cr(0.8090) sr(-0.5878) ex(-1.1241) ey(1.2485) Vb(3.7844) Vg(5.6408)
WARNING -     angle(210.0)  pwma_green=129 pwmb_blue=86
ACHIEVED ST_WIND_ANGLE=208.00
WARNING - angle(225.0)  deg(117.0)  theta(2.0420) c(-0.4540) s(0.8910) cr(0.8090) sr(-0.5878) ex(-0.7627) ey(1.4969) Vb(4.2228) Vg(5.6293)
WARNING -     angle(225.0)  pwma_green=128 pwmb_blue=96
ACHIEVED ST_WIND_ANGLE=223.00
WARNING - angle(240.0)  deg(102.0)  theta(1.7802) c(-0.2079) s(0.9781) cr(0.8090) sr(-0.5878) ex(-0.3493) ey(1.6433) Vb(4.6433) Vg(5.5048)
WARNING -     angle(240.0)  pwma_green=125 pwmb_blue=106
ACHIEVED ST_WIND_ANGLE=239.00
WARNING - angle(255.0)  deg(87.0)  theta(1.5184) c(0.0523) s(0.9986) cr(0.8090) sr(-0.5878) ex(0.0879) ey(1.6777) Vb(5.0173) Vg(5.2756)
WARNING -     angle(255.0)  pwma_green=120 pwmb_blue=114
ACHIEVED ST_WIND_ANGLE=254.00
WARNING - angle(270.0)  deg(72.0)  theta(1.2566) c(0.3090) s(0.9511) cr(0.8090) sr(-0.5878) ex(0.5191) ey(1.5978) Vb(5.3191) Vg(4.9575)
WARNING -     angle(270.0)  pwma_green=113 pwmb_blue=121
ACHIEVED ST_WIND_ANGLE=269.00
WARNING - angle(285.0)  deg(57.0)  theta(0.9948) c(0.5446) s(0.8387) cr(0.8090) sr(-0.5878) ex(0.9150) ey(1.4090) Vb(5.5284) Vg(4.5721)
WARNING -     angle(285.0)  pwma_green=104 pwmb_blue=126
ACHIEVED ST_WIND_ANGLE=285.00
WARNING - angle(300.0)  deg(42.0)  theta(0.7330) c(0.7431) s(0.6691) cr(0.8090) sr(-0.5878) ex(1.2485) ey(1.1241) Vb(5.6308) Vg(4.1456)
WARNING -     angle(300.0)  pwma_green=94 pwmb_blue=128
ACHIEVED ST_WIND_ANGLE=299.00
WARNING - angle(315.0)  deg(27.0)  theta(0.4712) c(0.8910) s(0.4540) cr(0.8090) sr(-0.5878) ex(1.4969) ey(0.7627) Vb(5.6193) Vg(3.7072)
WARNING -     angle(315.0)  pwma_green=84 pwmb_blue=128
ACHIEVED ST_WIND_ANGLE=315.00
WARNING - angle(330.0)  deg(12.0)  theta(0.2094) c(0.9781) s(0.2079) cr(0.8090) sr(-0.5878) ex(1.6433) ey(0.3493) Vb(5.4948) Vg(3.2867)
WARNING -     angle(330.0)  pwma_green=75 pwmb_blue=125
ACHIEVED ST_WIND_ANGLE=329.00
WARNING - angle(345.0)  deg(357.0)  theta(6.2308) c(0.9986) s(-0.0523) cr(0.8090) sr(-0.5878) ex(1.6777) ey(-0.0879) Vb(5.2656) Vg(2.9127)
WARNING -     angle(345.0)  pwma_green=66 pwmb_blue=120
ACHIEVED ST_WIND_ANGLE=344.00
WARNING - angle(0.0)  deg(342.0)  theta(5.9690) c(0.9511) s(-0.3090) cr(0.8090) sr(-0.5878) ex(1.5978) ey(-0.5191) Vb(4.9475) Vg(2.6109)
WARNING -     angle(0.0)  pwma_green=59 pwmb_blue=113
ACHIEVED ST_WIND_ANGLE=359.00
---------------- CIRCLE(3) COMPLETED -------------

Hence I now need to codify this step into the teensyBoat ST50 Wind Testing methodolgy
via additional UI.


### Wind Speed

The wind speed algorithm was developed in the middle of the angle
testing and implmentation.  Suffice it to say that I measured the
hz necessary for the Reference Instrument to display and report
a limited set of Wind Speeds, from which a somewhat general,
somwhat close, algorithm was implmented in instSimulator.cpp.


SPEED		PULSE_HZ		Dif Hz
---------------------------------------------
1			0.21 Hz		    0.215
2			0.90 Hz			0.79
3			1.65 Hz			0.75
4			2.38 Hz			0.73
5			3.10 Hz			0.72
6			3.85 Hz			0.74
7			4.59 Hz			0.74
8			5.31 Hz			0.72
9			6.01 Hz			0.70
10			6.72 Hz			0.71
11			7.44 Hz			0.72
12			8.16 Hz			0.72
13			8.88 Hz			0.72
14			9.61			0.73
15			10.34
16			11.20
17			11.80
18			12.60
19			13.30
20			14.00



## Teensyboat ST Testing UI Redesign

After a sanity checkin above the above hastily encoded UI and test code
I decided to re-implement the UI for ST50 Wind & Speed instrment testing
according to what I learned.

The UI is essentially driven by the GP8 General Purpose Connector
"Function" which can be Off, Speed, Wind, or ESP32.  That will not
change.

My initial approach to the for UI was that I would start by outputing
"raw" values, HZ for the speed instrument and HZ/PWMA/PWMB for the
Wind Instrument, and then have a "test_mode" that "turned on" the
conversion from boatSim's WaterSpeed or WindSpeed/WindAngle into the
correct HZ or HZ/PWMA/PWMB outputs.

	RAW_MODE = 0/1, default(0)

I will now switch that to the opposite.  That RAW_MODE must be asserted
and that the default will be TEST_MODE, in order to facilitate easier
testing of my inventory instruments.  In particular several ST50 Wind
commands will be added that work in TEST_MODE, leveraging off of what
I though was just "temporary test code" in the previous implementation.

ST50 Wind (GP8 WIND) TEST_MODE monadic commands added:

	WIND_OFF - will stop any circle currently in progress
	WIND_CALIB - will do a quick (45s) circle sending 2 degree increments
		of the apparent wind calculated PWM outputs to the instrument
		every 250 milliseconds.
	WIND_CIRCLE - will do a slow 15 seconds per 15 degree increment circle
		of the apparent wind.


Turning the GP8 mode off will CANCEL either of these modes.
Sending apparent wind angles (WA=xxx) while they are running will
cause unpredictable events.

The default heading of the simulated boat has been changed to zero and the
default wind angle has also been changed to zero, so that upon booting
teensyBoat these values are in known useful positions for ST50 testing
rather than optimized for a known initial E80 display position.

Thereafter (when not using the WIND_CALIB or WIND_CIRCLE) the water
speed can be adjusted with S=nnn for the Speed Instrument and
the wind angle and speed with WA=nnn and WS=nnn for the Wind
instrument.








a
known pulse frequency (HZ) and


