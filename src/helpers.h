#ifndef __RRC_HELPER_FUNCS__
#define __RRC_HELPER_FUNCS__

////    Includes    ////
#include <Arduino.h>
#include <gps.h>
#include <ms5611.h>
#include <MPU6050.h>
#include <ina260.h>
#include <SD.h>
#include "rrc_encoder/src/rrc_encoder.h"

////    constants    ////

#define radio_BAUD 9600
#define radio_SERIAL Serial2
#define GPS_SERIAL Serial7
#define BARO_WIRE Wire2
#define BUZZER PIN_A12
#define IMU_WIRE Wire1
#define AD0_VAL 0
#define SERIAL_MONITOR_BAUD 115200
#define pin_thermistor PIN_A5 //PIN_A8
#define PI 3.14159265358979323846
#define SERIESRESISTOR 9400
#define THERMISTORNOMINAL 10000 // resistance at 25 degrees C
#define BCOEFFICIENT 4000       // the beta coefficient of the thermistor (usually 3000-4000)
#define TEMPERATURENOMINAL 25   // temp. for nominal resistance (almost always 25 C)s
#define freq 1000
#define sea_press 1012.25 // kPa
#define sea_temp 288.15 // deg kelvin
#define R_gas_const 287.05 // J/(kgK)
#define gravitational_const 9.81 // m/s^2
#define temp_lapse_rate 0.0065 // k/m 
String logFileName = "log.txt";

const char outputFormat[] =
    R"""(
timestamp:   %lu
accel(g) ==> X = %lf , Y = %lf , Z = %lf , ||a|| = %lf 
gyro(deg/s) ==> X = %lf , Y = %lf , Z = %lf  
atmospheric ==> T = %lf C , P = %lf mbar, alt = %lf m 
battery temp  =  %lf C
Location:    %lf, %lf
initial board angles (deg) ==> X: %lf , Y: %lf  , Z: %lf 
tilt changes (deg)==>  X: %lf , Y: %lf  , Z: %lf 
)""";

/*
const char outputFormat[] =
    R"""(%lu,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf
)""";
*/

//===================
//      VARIABLES:
//=====================
double temp, pres, lon, lat, alt;
float ADC_raw, resistance_var, board_temp;
double angleX, angleY, angleZ, angleX_old, angleY_old, angleZ_old, angleX_diff, angleY_diff, angleZ_diff, accel_resultant;
uint32_t start;
uint8_t counter = 0;

////    Objects    ////
Ms5611 baro;
GPS gps;
MPU mpu;
AStruct imu_acc;
GStruct imu_gyro;

struct
{
    bool baro = false;
    bool gps = false;
    bool sdcard = false;
    bool radio = false;
} partsStates;

////    Functions prototypes:    ////
void buzzFor(unsigned int time_ms, unsigned int after = 0);
void setParts(void);
float analogToTemp(uint16_t pin);

////    Functions definitions:   ////

void buzzFor(unsigned int time_ms, unsigned int after)
{

    digitalWrite(BUZZER, HIGH);
    delay(time_ms);
    digitalWrite(BUZZER, LOW);
    delay(after);
}
// Function to convert radians to degrees
double rad_to_deg(double rad)
{
    return (rad * 180.0) / PI;
}

float analogToTemp(uint16_t pin)
{
    float a, b, temperature;
    ADC_raw = analogRead(pin);
    a = 1023 / ADC_raw;                  // ratio of max ADC to raw ADC, used for debugging
    b = a - 1;                           // used for debugging
    resistance_var = SERIESRESISTOR / b; // ADC = ratio resistance var/total resistance * max possible ADC val ==> solve for resistance_var

    // using the steinhart equation to convert resistnace to temp we get==> temperature = [1/B * ln(R/Ro) + (1/To)]^-1, To is in deg Kelvin must convert to degC after
    temperature = (1.0 / BCOEFFICIENT) * log((resistance_var / THERMISTORNOMINAL)) + (1.0 / (TEMPERATURENOMINAL + 273.15)); // inside brackets of equation
    temperature = 1.0 / temperature;                                                                                        // Invert
    temperature -= 273.15;                                                                                                  // convert absolute temp to C

    return temperature;
}

void setParts(void)
{
    // init MS5611
    if (!partsStates.baro)
    {
        if (baro.init(&BARO_WIRE))
        {

            partsStates.baro = false;
            Serial.println("MS5611 init error");
            buzzFor(500, 50);
        }
        else
        {
            partsStates.baro = true;
            Serial.println("MS5611 init OK");
        }
    }

    // init GPS
    if (!partsStates.gps)
    {
        gps.init(&GPS_SERIAL, 9600);
        partsStates.gps = true;
        Serial.println("GPS init OK");
    }

    // init radio
    if (!partsStates.radio)
    {
        radio_SERIAL.begin(radio_BAUD);
        partsStates.radio = true;
        Serial.println("radio init OK");
    }

    // init SD card
    if (!partsStates.sdcard)
    {
        if (!SD.begin(BUILTIN_SDCARD))
        {
            partsStates.sdcard = false;
            Serial.println("SD Card init error");
            buzzFor(500, 50);
        }
        else
        {
            partsStates.sdcard = true;
            Serial.println("SD Card init OK");
        }
    }
}

#endif
