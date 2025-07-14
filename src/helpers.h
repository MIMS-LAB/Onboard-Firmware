#ifndef ___HELPER_FUNCS__
#define ___HELPER_FUNCS__

////    Includes    ////
#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <gps.h>
#include <ms5611.h>
#include <MPU6050.h>

//#include <MLX90614.h>
#include <SD.h>
#include <ina260.h>//#include <Adafruit_INA260.h>/

// =====================================================================================
//                                    constants:
// =====================================================================================

#define radio_BAUD 9600
#define radio_SERIAL Serial2
#define esp32_SERIAL Serial5
#define GPS_SERIAL Serial7
#define BARO_WIRE Wire2
#define BUZZER PIN_A12
#define AD0_VAL 0
#define SERIAL_MONITOR_BAUD 115200
#define PI 3.14159265358979323846
#define freq 1000
#define sea_press 1012.25            // kPa
#define sea_temp 288.15              // deg kelvin
#define R_gas_const 8.31432          // N·m/(kmol·K)
#define gravitational_const 9.81     // m/s^2
#define temp_lapse_rate 0.0065       // k/m
#define molecular_weight_air 28.9644 // kg/kmol
#define TRIG_PIN PIN_A1
#define ECHO_PIN PIN_A0
#define PULSE_TIMEOUT 150000L  // ultrasonic sensor pulse timeout in 100ms // PULSE_TIMEOUT- can be a range of 10us - 3minutes and determines how long it waits to read the pulse
#define SR04_THRESHOLD_UPPER 18 // ultrasonic distance sensor threshold [cm] 
#define SR04_THRESHOLD_LOWER 5
#define SR04_MICROSECONDS_TO_CENTIMETERS (100.0/5882.0) // ultrasonic sensor conversion from ms to centimeters 

// Constants
String radio_read;
const double alt_dt_toronto = 75.0; // in meters

String logFileName = "droneLog.txt";
const char outputFormat[] =
    R"""(
timestamp:   %lu
accel(g) ==> X = %lf , Y = %lf , Z = %lf , ||a|| = %lf 
gyro(deg/s) ==> X = %lf , Y = %lf , Z = %lf  
atmospheric ==> T = %lf degK , P = %lf kPa, alt = %lf m 
battery  voltage ==>  %lf V
GPS==> location:    %lf, %lf,  quality: %d,  alttitude: %lf m
initial board angles (deg) ==> X: %lf , Y: %lf  , Z: %lf 
tilt changes (deg)==>  X: %lf , Y: %lf  , Z: %lf 
distance from an object: %lf cm 



)""";
// =====================================================================================
//                                    Global variables:
// =====================================================================================
double temp, pres, lon, lat, alt;

//float batt_temp;
double angleX, angleY, angleZ, angleX_old, angleY_old, angleZ_old, angleX_diff, angleY_diff, angleZ_diff, accel_resultant;
char string[1000] = {0};

float batt_volt;

int counter, gps_quality;
float gps_alt;
double  SR04_dist_cm, duration;


// =====================================================================================
//                                    Objects:
// =====================================================================================
Ms5611 baro;
GPS gps;

MPU mpu;
AStruct imu_acc;
GStruct imu_gyro;

//Adafruit_INA260 ina260 = Adafruit_INA260();
INA260 ina;
//MLX90614 mlx;



struct
{
    bool baro = false;
    bool gps = false;
    bool sdcard = false;
    bool radio = false;
    bool imu = false;
   // bool ina = false; 
} partsStates;

// =====================================================================================
//                                    protoypes:
// =====================================================================================

void buzzFor(unsigned int time_ms, unsigned int after = 0);
void setParts(void);
double rad_to_deg(double rad);
long SR04_Distance(); 

// =====================================================================================
//                                    Functions:
// =====================================================================================

//buzzer function used for debugging:
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

// ultrasonic sensor function:
long SR04_Distance() {
  duration = 0;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  duration = pulseIn(ECHO_PIN, HIGH, PULSE_TIMEOUT); // receives a pulse: waits for falling edge then starts a clock timer and returns 0 if no pulse was received
  // PULSE_TIMEOUT- can be a range of 10us - 3minutes and determines how long it waits to read the pulse
  delay(25);
  return duration;
}

//all sensor initialization check function:
void setParts(void)
{

    // init IMU:

    if (!partsStates.imu)
    {

        mpu.pwr_setup();
        mpu.acc_setup(1);
        mpu.gyro_setup(1);
        mpu.get_acc(1, &imu_acc);

        angleX = rad_to_deg(atan(imu_acc.ZAxis / imu_acc.XAxis));
        angleY = rad_to_deg(atan(imu_acc.ZAxis / imu_acc.YAxis));
        angleZ = rad_to_deg(atan(imu_acc.YAxis / imu_acc.ZAxis));

        angleX_old = angleX;
        angleY_old = angleY;
        angleZ_old = angleZ;

        partsStates.imu = true;
        Serial.println("IMU init OK");
    }

    // init barometer:

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
