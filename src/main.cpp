////    Includes    ////
#include <Arduino.h>
#include <math.h>
#include "helpers.h"

////    Constants    ////
String logFileName = "log.txt";
const char outputFormat[] =
R"""(
timestamp:   %lu
x = %lf g    y = %lf g   z = %lf g   total = %lf g
T = %lf C    P = %lf mbar
Location:    %lf, %lf

)""";

////    Initilization setup    ////
void setup(void)
{
    pinMode(BUZZER, OUTPUT);
    pinMode(BUZZER_ENABLE, INPUT_PULLUP);
    buzzFor(1000, 1000);

    // start serial monitor
    Serial.begin(115200);
    if(!Serial)
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
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);

    bool initialized = false;
    if (!initialized){
        myICM.begin(WIRE_PORT, AD0_VAL);

        Serial.print(F("Initialization of the sensor returned: "));
        Serial.println(myICM.statusString());
        if (myICM.status != ICM_20948_Stat_Ok){
            Serial.println("Trying again...");
            delay(500);
        }
        else{
            initialized = true;
        }
  }
}

////    Main loop    ////
void loop(void)
{
    const uint32_t freq = 1000;
    static uint32_t timestamp = 0;

    double temp, pres, lon, lat;
    //float PDOP, VDOP, HDOP;
    float x, y, z, r = 0;
    char string[256] = {0};
    uint32_t start = millis(); // store current time

    setParts();

    // read MS5611
    if(partsStates.baro)
    {
        if(baro.getTempPress(&temp, &pres))
        {
            Serial.printf("baro read failed\n");
        }
    }
     
    // read gps
    if (partsStates.gps)
    {
        if (gps.read_RMC(&lon, &lat, freq))
        {
            Serial.printf("gps RMC read failed\n");
        }
        
    }

    if (myICM.dataReady())
    {
        myICM.getAGMT();
        getScaledAGMT(&myICM, &x, &y, &z); 
        r = sqrt(x * x + y * y + z * z);
        //delay(500);
    }
    else
    {
        Serial.println("Waiting for data");
       // delay(500);
    }

    // print stuff to serial and SD card (need to call array for xyz, use equation for r here)
    sprintf(
        string, outputFormat,
        timestamp++, x/1000, y/1000, z/1000, r, temp, pres, lat, lon);

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
    
    transmit(x, RRC_HEAD_ACC_X, timestamp);
    transmit(y, RRC_HEAD_ACC_Y, timestamp);
    transmit(z, RRC_HEAD_ACC_Z, timestamp);
    transmit(temp, RRC_HEAD_TEMP, timestamp);
    transmit(pres, RRC_HEAD_PRESS, timestamp);
    transmit(lat, RRC_HEAD_GPS_LAT, timestamp);
    transmit(lon, RRC_HEAD_GPS_LONG, timestamp);

    // loop end lable
    
    loopEnd:
        while (millis() - start <= freq) // print every 1 second
            ;

    loopEndNoDelay:;


}