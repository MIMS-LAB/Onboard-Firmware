////    Includes    ////
#include <Arduino.h>
#include <math.h>

#include <adxl357.h>
#include <ms5611.h>
#include <gps.h>
#include <SD.h>


////    Defines    ////
#define DEBUG   true


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


////    debug function    ////
void debug()
{
    static int count = 0;
    Serial.printf("debug    %d\n", count++);
}


////    Initilization setup    ////
void setup(void)
{
    // start serial monitor
    Serial.begin(9600);
    while(!Serial);
    Serial.println("serial monitor started");

    // init ADXL357
    while(adxl357.init(ADXL357_DEF_ADD, &Wire))
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
    while(baro.init(&Wire2))
    {
        Serial.println("Can't detect an MS5611 device");
        delay(1000);
    }

    Serial.println("MS5611 init OK");
    delay(100);

    // init GPS
    gps.init(&Serial1, 9600);

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
}


////    Main loop    ////
void loop(void)
{
    double x, y, z, r = 0;
    double temp, pres, lon, lat;
    char   string[256] = {0};
    uint32_t start = millis();          // store current time


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


    // loop end lable
    loopEnd:
    while(millis() - start <= freq);
    
    loopEndNoDelay:
        ;
}