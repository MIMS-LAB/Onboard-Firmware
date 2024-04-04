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

    pinMode(PIN_RED, OUTPUT);
    pinMode(PIN_GREEN, OUTPUT);
    pinMode(PIN_BLUE, OUTPUT);

    buzzFor(1000, 1000);
    Wire.begin();
    Wire1.begin();
    Wire2.begin();
    ina260.begin();
    mpu.pwr_setup();
    mpu.acc_setup(1);
    if (!ina260.begin())
    {
        buzzFor(100, 100);
        errorLED(1);
        errorLED(0);
        delay(100);
        errorLED(1);
        errorLED(0);
        delay(100);
        errorLED(1);
    }

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

    // basically rfd uses the most power & since rocket is going to be idle on platorm for awhile dont do anything until bit is sent
    /*
    while(true)
     {
        //RFD_SERIAL.printf("idle\n");
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
    */
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
            errorLED(1);
            buzzFor(250, 250);

            if (led_debug)
            {
                errorLED(6);
                delay(250);

                errorLED(1);
            }
            else 
            {led_debug=false; 
            }
        }
    }

    else
    {
        errorLED(1);
        delay(250);
    }

    if(mpu.getErr()){
        mpu.pwr_setup();
        mpu.acc_setup(1);
        Serial.printf("imu read failed\n");
        errorLED(2);
        buzzFor(250,250);

        delay(100);
        mpu.get_acc(1,&imu_acc);
    }

    else{
        mpu.get_acc(1,&imu_acc);
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
            errorLED(3);
            // buzzFor(250,250);
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
            errorLED(3);
            partsStates.sdcard = false;
        }
    }

    // encode and transmit data

    if (rfd_comms_ini == true)
    {

        String command = RFD_SERIAL.readStringUntil('\n');
        command.toLowerCase();
        /*if (command.equals("launch"))
        {
            rfd_comms_ini = false;
            Serial.printf("launching\n");
            RFD_SERIAL.printf("start\n");
        }
        else
        {
            if (counter <= 5) // 5 seconds total ; triggers once a second
            {
                RFD_SERIAL.printf("idle\n");
                delay(1000);

                counter++;
            }
            else if (counter >= 9999)
            {
                counter = 0; // reset the counter
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
        transmit(lon, RRC_HEAD_GPS_LONG, timestamp);*/
        RFD_SERIAL.println(outputFormat);
        // transmit(volt_battery,RRC_HEAD_BATT_V,timestamp);
        Serial.println("transmitting");
    }
    while (millis() - start <= 4000) // print every 4 second
    {
        
    }
}
