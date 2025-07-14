#include "MLX90614.h"
uint8_t mlx_addr = 0x5A;

void MLX90614::pwmctrl()
{ // config setup
    WIRE.beginTransmission(ADDR);
    WIRE.write(MLX90614_REGISTER_PWMCTRL);
    WIRE.write(0x00);
    WIRE.endTransmission(true);
}

void MLX90614::config_setup()
{ // config setup
    WIRE.beginTransmission(ADDR);
    WIRE.write(MLX90614_REGISTER_CONFIG);
    WIRE.write(896);
    WIRE.endTransmission(true);
}

float MLX90614::get_temp()
{
    WIRE.beginTransmission(ADDR);
    WIRE.write(MLX90614_TEMP_ADDRESS);
    WIRE.endTransmission(false);
    WIRE.requestFrom(mlx_addr, (uint8_t)2, true);

    uint16_t raw =  WIRE.read() | WIRE.read() << 8;  // LSB is sent first then MSB so==> Wire.read is the LSB and Wire.read<<8 is the MSB
    float temp = raw * 0.02 - 273.15;


    return temp;
}
