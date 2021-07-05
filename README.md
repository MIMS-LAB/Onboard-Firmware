# Onboard-Firmware

This firmware was written using Platform.io, an extension to
vscode for microcontrollers programming, please use it to be
able to compile and upload the code properly.

This reposotory also uses git submodules to point to the 
require libraries needed, please clone this repo, do not 
download it, then run the following:

```bash
git submodule init
git submodule update
```


## Current status

The firmware at the moment successfuly does the folllwing:

* Initlizie and read sensors:
  * ADXL357 accelerometer
  * MS5611 barometer
  * Generic NMEA GPS
* Write readings to builtin SD card
* Transmit the collected data after encoding

The following tasks need to be done:

* For the GPS:
  * look into GPS modules specs similar to that of the featherWeight specifically the Cold & Warm Starts (due to the issue of it shutting off and losing lock when rocket exceeds the COCOM limits)
  * Make the GPS update at 10 Hz 
  
* Handle sensor initalization errors
* Order a PCB and field test the system
* Simulate power transients with Eagle/Kicad
* rfd900+ frequency switching 
* Bay thermodynamics(for two stage)
* Improve the way the rocket descends and lands(dynamic flight stability) 
