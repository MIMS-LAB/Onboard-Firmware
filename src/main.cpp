////    Includes    ////
#include <Arduino.h>
#include <math.h>
#include "helpers.h"
bool rfd_comms_ini = true;

double temp, pres, lon, lat;
// float PDOP, VDOP, HDOP;
float x, y, z, r = 0;
uint32_t start;
uint8_t counter = 0;
////    Initilization setup    ////
void setup(void)
{
    pinMode(BUZZER, OUTPUT);
    pinMode(BUZZER_ENABLE, INPUT_PULLUP);
    buzzFor(1000, 1000);

    // start serial monitor
    Serial.begin(SERIAL_MONITOR_BAUD);
    if (!Serial)
    {
        buzzFor(100, 50);
        buzzFor(100, 500);
    }
    else
    {
        Serial.println("serial monitor started");
    }

    setParts();

    RFD_SERIAL.printf("idle\n");

    // basically rfd uses the most power & since rocket is going to be idle on platorm for awhile dont do anything until bit is sent
    /*
    while(true)
     {
         if(RFD_SERIAL.available())
         {
             String command = RFD_SERIAL.readStringUntil('\n');
             command.toLowerCase();

             if(command.equals("launch"))
             {
                 break;
             }
             else
             {
                 Serial.printf("command \"%s\" unrecognized\n", command.c_str());
             }
         }
     }


     Serial.printf("launching\n");
     //RFD_SERIAL.printf("idle\n");

 */
    IMU_WIRE.begin();
    IMU_WIRE.setClock(400000);

    bool initialized = false;
    if (!initialized)
    {
        myICM.begin(IMU_WIRE, AD0_VAL);

        Serial.print(F("Initialization of the sensor returned: "));
        Serial.println(myICM.statusString());
        if (myICM.status != ICM_20948_Stat_Ok)
        {
            Serial.println("Trying again...");
            //         delay(500);
        }
        else
        {
            initialized = true;
        }
    }
}

////    Main loop    ////
void loop(void)
{
    static uint32_t timestamp = 0;

    char string[256] = {0};
    start = millis(); // store current time

    setParts();

    // read MS5611
    if (partsStates.baro)
    {
        if (baro.getTempPress(&temp, &pres))
        {
            Serial.printf("baro read failed\n");
        }
    }
    int numbSat, quality;
    char opMode;
    float HDOP, PDOP, sigStrength;
    // read gps
    if (partsStates.gps)
    {
        if (gps.read_RMC(&lon, &lat, freq))
        {
            Serial.printf("gps RMC read failed\n");
        }
        /*if (gps.read_GSA(&opMode, &HDOP, &PDOP, 1000))
        {
            Serial.println(F("GSA read fail"));
        }
        delay(100);
        if (gps.read_GSV(&numbSat, &sigStrength, 1000))
        {
            Serial.println(F("GSV read fail"));
        }
        delay(100);
        if (gps.read_GGA(&quality, 1000))
        {
            Serial.println(F("GGA read fail"));
        }*/
    }

    if (myICM.dataReady())
    {
        myICM.getAGMT();
        getScaledAGMT(&myICM, &x, &y, &z);
        r = sqrt(x * x + y * y + z * z);
        // delay(500);
    }
    else
    {
        Serial.println("Waiting for data");
        // delay(500);
    }

    // print stuff to serial and SD card (need to call array for xyz, use equation for r here)
    sprintf(
        string, outputFormat,
        timestamp++, x / 1000, y / 1000, z / 1000, r, temp, pres, lat, lon);

    Serial.printf("%s", string);

    if (partsStates.sdcard)
    {
        File dataFile = SD.open(logFileName.c_str(), FILE_WRITE);

        if (dataFile)
        {
            dataFile.println(string);
            dataFile.close();
        }
        else
        {
            Serial.printf("error opening %s\n", logFileName.c_str());
            partsStates.sdcard = false;
        }
    }

    // encode and transmit data

    if (rfd_comms_ini == true)
    {

        String command = RFD_SERIAL.readStringUntil('\n');
        command.toLowerCase();
        if (command.equals("launch"))
        {
            rfd_comms_ini = false;
            Serial.printf("launching\n");
            RFD_SERIAL.printf("start\n");
        }
        else
        {
            if (counter<=5) // 5 seconds total ; triggers once a second
            {
                RFD_SERIAL.printf("idle\n");
                delay(1000);
              
                counter++;
            }
            else if (counter>=9999)
            {
                counter =0; // reset the counter
            }

            Serial.printf("command \"%s\" unrecognized\n", command.c_str());
        }
    }
    else
    {
        transmit(x, RRC_HEAD_ACC_X, timestamp);
        transmit(y, RRC_HEAD_ACC_Y, timestamp);
        transmit(z, RRC_HEAD_ACC_Z, timestamp);
        transmit(temp, RRC_HEAD_TEMP, timestamp);
        transmit(pres, RRC_HEAD_PRESS, timestamp);
        transmit(lat, RRC_HEAD_GPS_LAT, timestamp);
        transmit(lon, RRC_HEAD_GPS_LONG, timestamp);
        Serial.println("transmitting");
    }
    while (millis() - start <= freq) // print every 4 second
    {
        ;
    }
}