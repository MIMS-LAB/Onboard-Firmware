////    Includes    ////
#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include "helpers.h"
#include <Adafruit_INA260.h>
bool rfd_comms_ini = true;

double temp, pres, lon, lat;
// float PDOP, VDOP, HDOP;
MPU mpu;
AStruct imu_acc;
Adafruit_INA260 ina260 = Adafruit_INA260();

float x, y, z, r = 0;
uint32_t start;
uint8_t counter = 0;
////    Initilization setup    ////
void setup(void)
{

    pinMode(BUZZER, OUTPUT);
    pinMode(BUZZER_ENABLE, INPUT_PULLUP);


    Wire.begin();
    Wire1.begin();
    Wire2.begin();
    ina260.begin();
    mpu.pwr_setup();
    mpu.acc_setup(1);
  
    if (!ina260.begin())
    {
        buzzFor(100, 100);
    }

    // start serial monitor
    Serial.begin(SERIAL_MONITOR_BAUD);

    setParts();
    pinMode(34, OUTPUT);
    digitalWrite(34, LOW);

    // basically rfd uses the most power & since rocket is going to be idle on platorm for awhile dont do anything until bit is sent
    /*
        while (true)
        {
            // RFD_SERIAL.printf("idle\n");
            if (RFD_SERIAL.available())
            {
                String command = RFD_SERIAL.readStringUntil('\n');
                command.toLowerCase();

                if (command.equals("launch"))
                {
                    break;
                }
                else
                {
                    Serial.printf("command \"%s\" unrecognized\n", command.c_str());
                }
            }
        }
        */

    Serial.printf("launching\n");
}

////    Main loop    ////
void loop(void)
{
    static uint32_t timestamp = 0;
    char string[256] = {0};
    start = millis(); // store current time

    setParts();
    float volt_battery = ina260.readBusVoltage();

    // read MS5611
    if (partsStates.baro)
    {
        if (baro.getTempPress(&temp, &pres))
        {
            Serial.printf("baro read failed\n");
            buzzFor(250, 250);
        }
    }

    if (mpu.getErr())
    {
        mpu.pwr_setup();
        mpu.acc_setup(1);
        Serial.printf("imu read failed\n");
        buzzFor(20, 20);

        mpu.get_acc(1, &imu_acc);
    }

    else
    {
        mpu.get_acc(1, &imu_acc);
    }

    int numbSat, quality;
    char opMode;
    float HDOP, PDOP, sigStrength;
    // read gps

    if (partsStates.gps)
    {
        if (gps.read_RMC(&lon, &lat, 4000))
        {
            Serial.printf("gps RMC read failed\n");
            buzzFor(250, 250);
        }
    }

    // print stuff to serial and SD card (need to call array for xyz, use equation for r here)
    sprintf(
        string, outputFormat,
        timestamp++, volt_battery, imu_acc.XAxis, imu_acc.YAxis, imu_acc.ZAxis, temp, pres, lat, lon);

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
            buzzFor(100, 20);
        }
    }

    // encode and transmit data
    rfd_comms_ini = false;

    if (rfd_comms_ini == true)
    {

        String command = RFD_SERIAL.readStringUntil('\n');
        command.toLowerCase();
        if (command.equals("launch"))
        {
            rfd_comms_ini = false;
            Serial.printf("launching\n");
        }
        else
        {
            if (counter == 20) // triggers every 20seconds
            {
                RFD_SERIAL.printf("idle\n");
            }
            else
            {
                counter = 0; // reset the counter
                delay(1000);
            }
            counter++;
            Serial.printf("command \"%s\" unrecognized %d \n", command.c_str(), counter);
        }
    }
    /*
        else
        {
            RFD_SERIAL.println(string);
            Serial.println("transmitting");
        }

        }*/
    /**/
    else
    {

        RFD_SERIAL.printf(string);
        Serial.println("transmitting");
    }

    while (millis() - start <= 4000) // print every 4 second
    {
    }
}
