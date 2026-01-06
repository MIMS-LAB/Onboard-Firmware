#ifndef ___HELPER_FUNCS__
#define ___HELPER_FUNCS__

////    Includes    ////
#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <gps.h>
#include <ms5611.h>
#include <MPU9250.h>
#include <Servo.h>
// #include <MLX90614.h>
#include <ina260.h> //#include <Adafruit_INA260.h>/

// =====================================================================================
//                                    constants:
// =====================================================================================

#define radio_BAUD 9600 // 230400
#define GPS_BAUD 9600
#define radio_SERIAL Serial2
//#define esp32_SERIAL Serial5
#define GPS_SERIAL Serial5
#define BARO_WIRE Wire
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
#define PULSE_TIMEOUT 150000L   // ultrasonic sensor pulse timeout in 100ms // PULSE_TIMEOUT- can be a range of 10us - 3minutes and determines how long it waits to read the pulse
#define SR04_THRESHOLD_UPPER 18 // ultrasonic distance sensor threshold [cm]
#define SR04_THRESHOLD_LOWER 5
#define SR04_MICROSECONDS_TO_CENTIMETERS (100.0 / 5882.0) // ultrasonic sensor conversion from ms to centimeters
#define MOTOR1_PWM_PIN PIN_A8
#define MOTOR2_PWM_PIN PIN_A9
#define MOTOR3_PWM_PIN PIN_A13
#define MOTOR4_PWM_PIN PIN_A10

#define thrust_threshold 0.1
#define MOTOR1_PWM_PERIOD 20000 // microseconds
// Constants
String radio_read, radio_read2, radio_read_old, radio_read_old2, radio_read3, radio_read_old3;

const double alt_dt_toronto = 75.0; // in meters
const int minPulse = 1000;          // Minimum pulse width for ESC (microseconds) - usually corresponds to stopped motor
const int maxPulse = 2020;          // Maximum pulse width for ESC (microseconds) - usually corresponds to full throttle
const char outputFormat[] =
    R"""(
timestamp:   %lu
accel(g) ==> X = %lf , Y = %lf , Z = %lf , ||a|| = %lf 
gyro(deg/s) ==> X = %lf , Y = %lf , Z = %lf  
atmospheric ==> T = %lf degK , P = %lf kPa, alt from SL  = %lf m , alt from start = %lf m
battery  voltage ==>  %lf V
GPS==> location:    %lf, %lf,  quality: %d,  alttitude: %lf m
initial board angles (deg) ==> X: %lf , Y: %lf  , Z: %lf 
tilt changes (deg)==>  X: %lf , Y: %lf  , Z: %lf 
distance from an object: %lf cm 
throttle (%%): %lf



)""";

/*
const char outputFormat[] =
    R"""(%lu, %lf ,%lf , %lf ,  %lf ,%lf ,%lf , %lf ,%lf  ,%lf,  %lf , %lf, %lf, %lf, %lf, %d,  %lf, %lf ,  %lf  , %lf, %lf , %lf  ,  %lf, %lf, %lf
)""";
*/
// =====================================================================================
//                                    Global variables:
// =====================================================================================
double temp, pres, lon, lat, alt;
double temp_start, pres_start, alt_start;

// float batt_temp;
double angleX, angleY, angleZ, angleX_old, angleY_old, angleZ_old, angleX_diff, angleY_diff, angleZ_diff, accel_resultant;
char string[1600] = {0};

float batt_volt;

int counter, gps_quality;
float gps_alt, DUTY_CYCLE_FRACT_T;
double SR04_dist_cm, duration;
float pos = 0.0;        // postion int for servo test
bool servoState = true; // bool used for state of servo direction ; 0 = CW , 1= CCW

// =====================================================================================
//                                    Objects:
// =====================================================================================
Ms5611 baro;
GPS gps;

MPU mpu;
AStruct imu_acc;
GStruct imu_gyro;


Servo esc, esc2,esc3,esc4; // create servo object to control a servo

// Adafruit_INA260 ina260 = Adafruit_INA260();
INA260 ina;
// MLX90614 mlx;

struct
{
    bool baro = false;
    bool gps = false;
    bool radio = false;
    bool imu = false;
    // bool ina = false;
} partsStates;

// =====================================================================================
//                                    protoypes:
// =====================================================================================

void setParts(void);
double rad_to_deg(double rad);
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
long SR04_Distance();
void rotateMotor(int i, int delayTime, float dutycycle_fraction);

// =====================================================================================
//                                    Functions:
// =====================================================================================



// Function to convert radians to degrees
double rad_to_deg(double rad)
{
    return (rad * 180.0) / PI;
}
// converting float bounds to other float bound ranges:

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ultrasonic sensor function:
long SR04_Distance()
{
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

// motor code:

/**

    @param dutycycle_fraction: int width of pwm signal (val is %)

*/
void rotateMotor(int i, int delayTime, float dutycycle_fraction)
{
    switch (i)
    {
    case 1:
        esc.writeMicroseconds(minPulse + dutycycle_fraction * (maxPulse - minPulse));
        break;
    case 2:
        esc2.writeMicroseconds(minPulse + dutycycle_fraction * (maxPulse - minPulse));
    case 3:
        esc3.writeMicroseconds(minPulse + dutycycle_fraction * (maxPulse - minPulse));
    case 4:
        esc4.writeMicroseconds(minPulse + dutycycle_fraction * (maxPulse - minPulse));
    }
    delayMicroseconds(delayTime);
}

// all sensor initialization check function:
void setParts(void)
{

    // init IMU:

    if (!partsStates.imu)
    {
        mpu.init(1,1); 

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
           // buzzFor(500, 50);
        }
        else
        {
            partsStates.baro = true;
            Serial.println("MS5611 init OK");

            baro.getTempPress(&temp_start, &pres_start);

            // convert pres from mbar to kpa:

            pres_start = 0.1 * pres_start;
            // convert air temp from degC to kelvin:

            temp_start = temp_start + 273.15;

            alt_start = (log(pres_start / sea_press) * R_gas_const * temp_start) / (-gravitational_const * molecular_weight_air) + alt_dt_toronto; // https://en.wikipedia.org/wiki/Barometric_formula
        }
    }
    // init GPS
    if (!partsStates.gps)
    {
        gps.init(&GPS_SERIAL, GPS_BAUD);
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

}

#endif
