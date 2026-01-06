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
  * MPU9250 IMU
  * MS5611 barometer
  * Generic NMEA GPS (BN220)
  * INA260
  * HC-SR04 ULTRASONIC SENSOR
* 4 MICRO QUADROTORS
* RADIO IS THE XBEE S2C
* CAMERA STREAM WITH ESP32 CAM

