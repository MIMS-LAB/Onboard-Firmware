#ifndef __RRC_HELPER_FUNCS__
#define __RRC_HELPER_FUNCS__


////    Includes    ////
#include <Arduino.h>
#include <adxl357.h>
#include <gps.h>
#include <ms5611.h>
#include <SD.h>
#include "rrc_encoder/src/rrc_encoder.h"
#include "ICM_20948.h"


////    Defines    ////
#define RFD_BAUD      57600
#define RFD_SERIAL    Serial2
#define GPS_SERIAL    Serial1
#define ADXL_WIRE     Wire
#define BARO_WIRE     Wire2
#define BUZZER        PIN_A13
#define BUZZER_ENABLE PIN_A12
#define startSerial_ms_timeDelay 90000
//#define startSerial_min_timeDelay 15
#define WIRE_PORT Wire
#define AD0_VAL 0

////    Objects    ////
Adxl357     adxl357;
Ms5611      baro;
GPS         gps;
struct
{
    bool    adxl   = false;
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
void printPaddedInt16b(int16_t val);
void printFormattedFloat(float val, uint8_t leading, uint8_t decimals);
void printScaledAGMT(ICM_20948_I2C *sensor);

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
    // init ADXL357
    if(!partsStates.adxl)
    {
        if(adxl357.init(ADXL357_DEF_ADD, &ADXL_WIRE))
        {
            partsStates.adxl = false;
            Serial.println("ADXL init error");
            buzzFor(500, 50);
            buzzFor(50, 250);
        }
        else
        {
            adxl357.setAccelRange(ADXL357_FOUTY_G);
            adxl357.setPowerCTL(ADXL357_ALL_ON);
            adxl357.setCalibrationConstant(1.0 / (double) 12490);  // calculate your own calibration constant

            partsStates.adxl = true;
            Serial.println("ADXL357 init OK");
            buzzFor(50, 100);
        }
    }


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
void printPaddedInt16b(int16_t val)
{
  if (val > 0)
  {
    Serial.print(" ");
    if (val < 10000)
    {
      Serial.print("0");
    }
    if (val < 1000)
    {
      Serial.print("0");
    }
    if (val < 100)
    {
      Serial.print("0");
    }
    if (val < 10)
    {
      Serial.print("0");
    }
  }
  else
  {
    Serial.print("-");
    if (abs(val) < 10000)
    {
      Serial.print("0");
    }
    if (abs(val) < 1000)
    {
      Serial.print("0");
    }
    if (abs(val) < 100)
    {
      Serial.print("0");
    }
    if (abs(val) < 10)
    {
      Serial.print("0");
    }
  }
  Serial.print(abs(val));
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    Serial.print("-");
  }
  else
  {
    Serial.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      Serial.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    Serial.print(-val, decimals);
  }
  else
  {
    Serial.print(val, decimals);
  }
}

void printScaledAGMT(ICM_20948_I2C *sensor)
{
  Serial.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  Serial.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  Serial.print(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  Serial.print(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5, 2);
  Serial.print(" ]");
  Serial.println();
}
