# Typewriter
(c) Scott Baker, http://www.smbaker.com/

## Purpose

This allows you to turn an old-school parallel printer (the kind with a centronics 36-pin interface) into
a typewriter. I used this with a Royal OfficeMaster 2000 Daisy-Wheel Printer.

## Design

I used an atmega328 microcontroller, more or less direct-wired to the parallel port and to the PS2
keyboard. We read scancodes from the keyboard and write ascii characters to the parallel port. Several
of the function keys are mapped as follows:

* F1 ... Print head power 0
* F2 ... Print head power 1
* F3 ... Print head power 2
* F4 ... Print head power 3
* F5 ... Bold
* F6 ... Shadow
* F7 ... Underline
* F10 ... Reset Bold, Shadow, and Underline

The control codes for these function keys cam eout of the Royal OfficeMaster 2000 manual, and are probably
not useful on other printers.
