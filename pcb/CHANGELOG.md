(DD.MM.YYYY) - version

**19.08.2021 - v1.0**
Initial version

**11.09.2021 - v1.1**
*Complete redesign (of PCB, schematic mostly untouched)*
- input connectors are now on the same side to improve mounting in the server, as well as routing
- fixed completely incorrect input pinout
- fixed component spacing issues (gaps betwen caps were almost nonexistent)
- removed unnecessary silkscreen markings
- removed unnecessary decoupling caps
- replaced 3mm mounting holes /w 2mm

**07.10.2021 - v1.2**
*Minor fixes & improvements*
- fixed mirrored input connectors (GND flipped /w VCC, FAN_SENSE /w NC pin 2)
- replaced TC2050 /w TC2030-NL - less pins, no legs, etc
- added power indication LED
- added a SMD 2.5mm pitch pin header on top of the TC footprint (as an alternative)
- generally improved trace routing a bit (mostly optically & logically)
