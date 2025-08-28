# instSimulator

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





