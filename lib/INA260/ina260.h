#ifndef __INA260_H__
#include <Wire.h>
#include <Arduino.h>
#include <stdint.h>

#define ADDR 0x40
#define WIRE Wire2

#define INA260_REG_CONFIG 0x00     ///< Configuration register
#define INA260_REG_CURRENT 0x01    ///< Current measurement register (signed) in mA
#define INA260_REG_BUSVOLTAGE 0x02 ///< Bus voltage measurement register in mV
#define INA260_REG_POWER 0x03      ///< Power calculation register in mW

class INA260
{
public:
    // config functions
    void config_setup();

    // data read functions
    float get_volt(int Vnum);
    float get_current(int Cnum);
};

#endif