////    Includes    ////
#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include "helpers.h"

////    Initilization setup    ////
void setup(void)
{

    pinMode(BUZZER, OUTPUT);
    pinMode(pin_thermistor, INPUT);

    Wire.begin();
    Wire1.begin();
    Wire2.begin();
    mpu.pwr_setup();
    mpu.acc_setup(1);
    mpu.gyro_setup(1);
    mpu.get_acc(1, &imu_acc);
    angleX = rad_to_deg(atan(imu_acc.ZAxis / imu_acc.XAxis));
    angleY = rad_to_deg(atan(imu_acc.ZAxis / imu_acc.YAxis));
    angleZ = rad_to_deg(atan(imu_acc.YAxis / imu_acc.ZAxis));

    angleX_old = angleX;
    angleY_old = angleY;
    angleZ_old = angleZ;

    // start serial monitor
    Serial.begin(SERIAL_MONITOR_BAUD);

    setParts();

    Serial.printf("launching\n");
    radio_SERIAL.printf("launching\n");


}

////    Main loop    ////
void loop(void)
{
    static uint32_t timestamp = 0;
    char string[400] = {0};
    start = millis(); // store current time
    setParts();

    // read MS5611
    if (partsStates.baro)
    {
        if (baro.getTempPress(&temp, &pres))
        {
            Serial.printf("baro read failed\n");

            buzzFor(250, 250);
        }
    }

    // read IMU:
    if (mpu.getErr())
    {
        mpu.pwr_setup();
        mpu.acc_setup(1);
        mpu.gyro_setup(1);

        Serial.printf("imu read failed\n");
        buzzFor(20, 20);
    }

    else
    {
        mpu.get_acc(1, &imu_acc);
        mpu.get_gyro(1, &imu_gyro);
        accel_resultant = sqrt(pow(imu_acc.XAxis, 2) + pow(imu_acc.YAxis, 2) + pow(imu_acc.ZAxis, 2));
        angleX = rad_to_deg(atan(imu_acc.ZAxis / imu_acc.XAxis));
        angleY = rad_to_deg(atan(imu_acc.ZAxis / imu_acc.YAxis));
        angleZ = rad_to_deg(atan(imu_acc.YAxis / imu_acc.ZAxis));
        angleX_diff = angleX - angleX_old;
        angleY_diff = angleY - angleY_old;
        angleZ_diff = angleZ - angleZ_old;
    }

    gps.read_RMC(&lon, &lat, 400);

    alt = ((pow((sea_press / pres), R_gas_const*temp_lapse_rate/gravitational_const) - 1.0) * (temp + 273.15 )) / (temp_lapse_rate); //(sea_temp/temp_lapse_rate)*(pow(pres/sea_press,R_gas_const*temp_lapse_rate/gravitational_const)-1); 
    board_temp = analogToTemp(pin_thermistor);

    // print stuff to serial and SD card (need to call array for xyz, use equation for r here)
    sprintf(
        string, outputFormat,
        timestamp++, imu_acc.XAxis, imu_acc.YAxis, imu_acc.ZAxis, accel_resultant, imu_gyro.XAxis, imu_gyro.YAxis, imu_gyro.ZAxis, temp, pres, alt, board_temp, lat, lon, angleX, angleY, angleZ, angleX_diff, angleY_diff, angleZ_diff);

    Serial.printf("%s", string);
    radio_SERIAL.printf("%s", string);

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
}
