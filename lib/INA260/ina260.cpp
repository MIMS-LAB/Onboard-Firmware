#include "ina260.h"
uint8_t ina_addr = 0x40;

void INA260::config_setup()
{ // config setup
    WIRE.beginTransmission(ADDR);
    WIRE.write(INA260_REG_CONFIG);
    WIRE.write(189);
    WIRE.endTransmission(true);
}

float INA260::get_volt()
{
    WIRE.beginTransmission(ADDR);
    WIRE.write(INA260_REG_BUSVOLTAGE);
    WIRE.endTransmission(false);
    WIRE.requestFrom(ina_addr, (size_t)6, true);

    float volt = WIRE.read() << 8 | (WIRE.read()); // MSB|lsb


    return volt_lsb_calib_val*volt;
}

float INA260::get_current()
{
    WIRE.beginTransmission(ADDR);
    WIRE.write(INA260_REG_CURRENT);
    WIRE.endTransmission(true);
    WIRE.requestFrom(ina_addr, (size_t)6, true);

    float current = WIRE.read() << 8 | WIRE.read(); // MSB|lsb

    return current;
}
