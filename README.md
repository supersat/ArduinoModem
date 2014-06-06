ArduinoModem
============

A fully functional modem implemented entirely on an Arduino.

This is a work in progress. Right now only Bell 103 (as an answering station)
is supported. Simply hook up a 6-bit R2R DAC to PORTD (D8-D13 on the Arduino)
and your audio input to A0 (offset by AREF/2).

NOTE: The Arduino Uno is NOT capable of supporting 300 bps connections
out-of-the-box. You must modify your ATmega8U2's firmware to enable 300
bps serial connections to work. See this forum post for details:
http://forum.arduino.cc/index.php?topic=110939.0;wap2
