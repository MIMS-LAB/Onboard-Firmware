////    Includes    ////
#include <Arduino.h>
#include <math.h>

#include <adxl357.h>
#include <ms5611.h>
#include <gps.h>
#include <SD.h>
#include "rrc_encoder/src/rrc_encoder.h"

////    Defines    ////
#define DEBUG       true
#define RFD_BAUD    57600
#define RFD_SERIAL  Serial2
#define GPS_SERIAL  Serial1
#define ADXL_WIRE   Wire
#define BARO_WIRE   Wire2


////    Globals    ////
Adxl357     adxl357;
Ms5611      baro;
GPS         gps;
uint32_t    freq = 1000;
uint32_t    timestamp = 0;


////    Constants    ////
const char outputFormat[] = 
R"""(
timestamp:   %u
x = %lf g    y = %lf g   z = %lf g   total = %lf g
T = %lf C    P = %lf mbar
Location:    %f, %f

)""";


////    Functions declairations    ////
void transmit   (double data, uint8_t header);
void debug      ();


////    Initilization setup    ////
void setup(void)
{
    // start serial monitor
    Serial.begin(9600);
    while(!Serial);
    Serial.println("serial monitor started");

    // init ADXL357
    while(adxl357.init(ADXL357_DEF_ADD, &ADXL_WIRE))
    {
        Serial.println("Can't detect an ADXL357 device");
        delay(1000);
    }

    adxl357.setAccelRange(ADXL357_FOUTY_G);
    adxl357.setPowerCTL(ADXL357_ALL_ON);
    adxl357.setCalibrationConstant(1.0 / (double) 12490);  // calculate your own calibration constant

    Serial.println("ADXL357 init OK");
    delay(100);

    // init MS5611
    while(baro.init(&BARO_WIRE))
    {
        Serial.println("Can't detect an MS5611 device");
        delay(1000);
    }

    Serial.println("MS5611 init OK");
    delay(100);

    // init GPS
    gps.init(&GPS_SERIAL, 9600);

    Serial.println("GPS init OK");
    delay(100);

    // init SD card
    while(!SD.begin(BUILTIN_SDCARD))
    {
        Serial.println("Card failed or not present");
        delay(1000);
    }

    Serial.println("card initialized.");
    delay(100);

    // init RFD
    RFD_SERIAL.begin(RFD_BAUD);

    Serial.println("RFD init OK");
    delay(100);

}


////    Main loop    ////
void loop(void)
{
    double   temp, pres, lon, lat;
    double   x, y, z, r  = 0;
    char     string[256] = {0};
    uint8_t  package[10] = {0};
    uint32_t start       = millis();          // store current time


    // read ADXL357
    if(adxl357.isDataReady())
    {
        if (adxl357.getScaledAccelData(&x, &y, &z))
        {
            Serial.printf("acceleration read failed\n");
            goto accelReadBreak;
        }

        r = sqrt(x*x + y*y + z*z);
    }
    else
    {
        Serial.printf("acceleration data is not ready\n");
    }
    accelReadBreak:


    // read MS5611
    if(baro.getTempPress(&temp, &pres))
    {
        Serial.printf("baro read failed\n");
    }


    // read gps
    if(gps.readLocation(&lon, &lat, freq))
    {
        Serial.printf("gps read failed\n");
    }


    // print stuff to serial and SD card
    sprintf(
        string, outputFormat,
        timestamp++, x, y, z, r, temp, pres, lat, lon);

    Serial.printf("%s", string);
    
    File dataFile = SD.open("log.txt", FILE_WRITE);

    if (dataFile)
    {
        dataFile.println(string);
        dataFile.close();
    } 
    else
    {
        Serial.println("error opening datalog.txt");
    }


    // encode and transmit data
    transmit(x,    RRC_HEAD_ACC_X);
    transmit(y,    RRC_HEAD_ACC_Y);
    transmit(z,    RRC_HEAD_ACC_Z);
    transmit(temp, RRC_HEAD_TEMP);
    transmit(pres, RRC_HEAD_PRESS);
    transmit(lat,  RRC_HEAD_GPS_LAT);
    transmit(lon,  RRC_HEAD_GPS_LONG);


    // loop end lable
    loopEnd:
    while(millis() - start <= freq);
    
    loopEndNoDelay:
        ;
}


////    helper functions    ////
void transmit(double data, uint8_t header)
{
    uint8_t  package[10] = {0};
    encode(data, header, timestamp, package);
    RFD_SERIAL.write(package, 10);
}


void debug()
{
    static int count = 0;
    Serial.printf("debug    %d\n", count++);
}