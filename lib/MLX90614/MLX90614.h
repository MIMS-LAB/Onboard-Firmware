#ifndef __MLX90614_H__
#include <Wire.h>
#include <Arduino.h>
#include <stdint.h>

#define WIRE Wire1
#define ADDR 0x5A
#define MLX90614_TEMP_ADDRESS 0x07
#define MLX90614_REGISTER_PWMCTRL 0x22
#define MLX90614_REGISTER_CONFIG 0x25
/*

NOTE: this sensor sends LSB fisrt not MSB 

*/

class MLX90614
{
public:
    float get_temp();
    void pwmctrl();

private:
    void config_setup();
};

#endif // MLX90614_H
