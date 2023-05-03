#ifndef __RRC_HELPER_FUNCS__
#define __RRC_HELPER_FUNCS__


////    Includes    ////
#include <Arduino.h>
#include <gps.h>
#include <ms5611.h>
#include <SD.h>
#include "rrc_encoder/src/rrc_encoder.h"
#include "ICM_20948.h"


////    Defines    ////
#define RFD_BAUD      38400
#define RFD_SERIAL    Serial2
#define GPS_SERIAL    Serial7
#define BARO_WIRE     Wire1
#define BUZZER        PIN_A12
#define BUZZER_ENABLE PIN_A13
#define startSerial_ms_timeDelay 90000
//#define startSerial_min_timeDelay 15
#define WIRE_PORT Wire2
#define AD0_VAL 0

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
ICM_20948_I2C myICM;

////    Functions declairations    ////
void transmit   (double data, uint8_t header, uint32_t time);
void debug      (void);
void buzzFor    (unsigned int time_ms, unsigned int after = 0);
void setParts   (void);
void getScaledAGMT(ICM_20948_I2C *sensor);

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
    }
}


void setParts(void)
{
    // init MS5611
    if(!partsStates.baro)
    {
        if(baro.init(&BARO_WIRE))
        {
            partsStates.baro = false;
            Serial.println("MS5611 init error");
            buzzFor(500, 50);
            buzzFor(50, 50);
            buzzFor(50, 250);
        }
        else
        {
            partsStates.baro = true;
            Serial.println("MS5611 init OK");
            buzzFor(50, 100);
        }
    }

    // init GPS
    if(!partsStates.gps)
    {
        gps.init(&GPS_SERIAL, 9600);

        partsStates.gps = true;
        Serial.println("GPS init OK");
        buzzFor(50, 100);
    }


    // init RFD
    if(!partsStates.rfd)
    {
        RFD_SERIAL.begin(RFD_BAUD);

        partsStates.rfd = true;
        Serial.println("RFD init OK");
        buzzFor(50, 100);
    }


    // init SD card
    if(!partsStates.sdcard)
    {
        if(!SD.begin(BUILTIN_SDCARD))
        {
            partsStates.sdcard = false;
            Serial.println("SD Card init error");
            buzzFor(500, 50);
            buzzFor(50, 50);
            buzzFor(50, 50);
            buzzFor(50, 250);

        }
        else
        {
            partsStates.sdcard = true;
            Serial.println("SD Card init OK");
            buzzFor(50, 100);
        }
    }
}


#endif  //  #ifndef __RRC_HELPER_FUNCS__

//IMU
void getScaledAGMT(ICM_20948_I2C *sensor,float *x,float *y,float *z) //array [0]
{
  *x = sensor->accX();
  *y = sensor->accY();
  *z = sensor->accZ();
}
