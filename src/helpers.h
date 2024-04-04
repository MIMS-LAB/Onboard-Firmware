#ifndef __RRC_HELPER_FUNCS__
#define __RRC_HELPER_FUNCS__


////    Includes    ////
#include <Arduino.h>
#include <gps.h>
#include <ms5611.h>
#include <MPU6050.h> 
#include <SD.h>
#include "rrc_encoder/src/rrc_encoder.h"
//#include "ICM_20948.h"


////    Defines    ////
#define RFD_BAUD      115200
#define RFD_SERIAL    Serial2
#define GPS_SERIAL    Serial7
#define BARO_WIRE     Wire2
#define BUZZER        PIN_A12
#define BUZZER_ENABLE PIN_A13//always apply
//#define IMU_WIRE      Wire1
#define AD0_VAL        0
#define SERIAL_MONITOR_BAUD 115200
#define PIN_RED        4
#define PIN_GREEN      3
#define PIN_BLUE       2                                                                                                                         

long freq = 1000;
bool led_debug = false;

////    Constants    ////
String logFileName = "log.txt";

/*
const char outputFormat[] =
    R"""(
timestamp:   %lu
x = %lf g    y = %lf g   z = %lf g   total = %lf g
T = %lf C    P = %lf mbar
Location:    %lf, %lf

)""";
*/
const char outputFormat[] =
    R"""(%lu,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf
)""";

////    Objects    ////
Ms5611      baro;
GPS         gps;
struct
{
    bool    baro   = false;
    bool    gps    = false;
    bool    sdcard = false;
    bool    rfd    = false;
} partsStates;
//ICM_20948_I2C myICM;

////    Functions declairations    ////
void transmit   (double data, uint8_t header, uint32_t time);
void debug      (void);
void buzzFor    (unsigned int time_ms, unsigned int after = 0);
void setParts   (void);
//void getScaledAGMT(ICM_20948_I2C *sensor);

////    Functions definitions    ////
void transmit(double data, uint8_t header, uint32_t time)
{
    uint8_t  package[10] = {0};
    encode(data, header, time, package);
    RFD_SERIAL.write(package, 10);
}


void debug(void)
{
    static int count = 0;
    Serial.printf("debug    %d\n", count++);
}

void buzzFor(unsigned int time_ms, unsigned int after)
{
    if(digitalRead(BUZZER_ENABLE))
    {
        digitalWrite(BUZZER, HIGH);
        delay(time_ms);
        digitalWrite(BUZZER, LOW);
        delay(after);
        led_debug = true; 
    }
}



void errorLED(int errorcode)
{
  unsigned long time = 0;
  unsigned long delta = 0;
  time = millis();
  
  switch(errorcode) {
    case 1:      
      analogWrite(PIN_RED,   255);
      analogWrite(PIN_GREEN, 0);
      analogWrite(PIN_BLUE,  0);
      break;
  
    case 2:

      analogWrite(PIN_RED,   0);
      analogWrite(PIN_GREEN, 255);
      analogWrite(PIN_BLUE,  0);
      break;

    case 3:

      analogWrite(PIN_RED,   0);
      analogWrite(PIN_GREEN, 0);
      analogWrite(PIN_BLUE,  255);
      break;

    case 4:

      analogWrite(PIN_RED,   0);
      analogWrite(PIN_GREEN, 200);
      analogWrite(PIN_BLUE,  255);

      break;
    
    case 5:

      analogWrite(PIN_RED,   200);
      analogWrite(PIN_GREEN, 12);
      analogWrite(PIN_BLUE,  0);

      break;
      case 6:
            analogWrite(PIN_RED,   0);

 analogWrite(PIN_GREEN, 0);
      analogWrite(PIN_BLUE,  0);

      break;
      

  }
}

void setParts(void)
{
    // init MS5611
    if(!partsStates.baro)
        {        
        if(baro.init(&BARO_WIRE)){

            partsStates.baro = false;
            Serial.println("MS5611 init error");
           // buzzFor(500, 50);
            buzzFor(50, 50);
            //buzzFor(50, 250);
        }
        else
        {
            partsStates.baro = true;
            Serial.println("MS5611 init OK");
            //buzzFor(50, 100);
        }
    }

    // init GPS
    if(!partsStates.gps)
    {
        gps.init(&GPS_SERIAL, 9600);
        partsStates.gps = true;
        Serial.println("GPS init OK");
        //buzzFor(50, 100);
    }


    // init RFD
    if(!partsStates.rfd)
    {
        RFD_SERIAL.begin(RFD_BAUD);
        partsStates.rfd = true;
        Serial.println("RFD init OK");
        //buzzFor(50, 100);
    }


    // init SD card
    if(!partsStates.sdcard)
    {
        if(!SD.begin(BUILTIN_SDCARD))
        {
            partsStates.sdcard = false;
            Serial.println("SD Card init error");
           // buzzFor(500, 50);
           // buzzFor(50, 50);
           // buzzFor(50, 50);
           // buzzFor(50, 250);

        }
        else
        {
            partsStates.sdcard = true;
            Serial.println("SD Card init OK");
           // buzzFor(50, 100);
        }
    }
}



#endif  //  #ifndef __RRC_HELPER_FUNCS__

//IMU
/*
void getScaledAGMT(ICM_20948_I2C *sensor,float *x,float *y,float *z) //array [0]
{
  *x = sensor->accX();
  *y = sensor->accY();
  *z = sensor->accZ();
}
*/