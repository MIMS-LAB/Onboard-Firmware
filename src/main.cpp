#include "helpers.h"
// =====================================================================================
//                                    Setup:
// =====================================================================================
void setup(void)
{

    pinMode(BUZZER, OUTPUT); // buzzer pin for debugging
  // SR04 ultrasonic sensor:
    pinMode(ECHO_PIN, INPUT);  // what receives the soundwaves for parsing
    pinMode(TRIG_PIN, OUTPUT); // what sends out the soundwave

    // start serial monitor
    Serial.begin(SERIAL_MONITOR_BAUD);

    Wire.begin();
    // Wire1.begin();
    Wire2.begin();

    ina.config_setup();//ina260.begin(); //
    setParts();

    //  mlx.pwmctrl();

    Serial.printf("launching\n");
    radio_SERIAL.printf("launching\n");
}

// =====================================================================================
//                                    Main Loop:
// =====================================================================================
void loop(void)
{

    uint32_t timestamp = millis();

    String radio_read_old = radio_read;
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

    mpu.get_acc(1, &imu_acc);
    mpu.get_gyro(1, &imu_gyro);
    accel_resultant = sqrt(pow(imu_acc.XAxis, 2) + pow(imu_acc.YAxis, 2) + pow(imu_acc.ZAxis, 2));
    angleX = rad_to_deg(atan(imu_acc.ZAxis / imu_acc.XAxis));
    angleY = rad_to_deg(atan(imu_acc.ZAxis / imu_acc.YAxis));
    angleZ = rad_to_deg(atan(imu_acc.YAxis / imu_acc.ZAxis));
    angleX_diff = angleX - angleX_old;
    angleY_diff = angleY - angleY_old;
    angleZ_diff = angleZ - angleZ_old;

    gps.read_RMC(&lon, &lat);
    gps.read_GGA(&gps_quality, &gps_alt);

    // convert pres from mbar to kpa:

    pres = 0.1 * pres;
    // convert air temp from degC to kelvin:

    temp = temp + 273.15;

    alt = (log(pres / sea_press) * R_gas_const * temp) / (-gravitational_const * molecular_weight_air) + alt_dt_toronto; // https://en.wikipedia.org/wiki/Barometric_formula

    // batt_temp = mlx.get_temp();                                                                                                         // mplx.get_temp(0); // analogToTemp(pin_thermistor);

    batt_volt = ina.get_volt()/1000.0;//ina260.readBusVoltage()/1000.0; // ;//   //

    SR04_dist_cm = SR04_MICROSECONDS_TO_CENTIMETERS * SR04_Distance();

    // joystick controls:
    radio_read = radio_SERIAL.readStringUntil('\n');
    if (radio_read != radio_read_old)
    {
        Serial.print("radio read joystick values");
        Serial.println(radio_read.c_str());
    }

    // print stuff to serial and SD card (need to call array for xyz, use equation for r here)
    sprintf(
        string, outputFormat,
        timestamp / 1000, imu_acc.XAxis, imu_acc.YAxis, imu_acc.ZAxis, accel_resultant, imu_gyro.XAxis, imu_gyro.YAxis, imu_gyro.ZAxis, temp, pres, alt, batt_volt, lat, lon, gps_quality, gps_alt, angleX, angleY, angleZ, angleX_diff, angleY_diff, angleZ_diff, SR04_dist_cm);

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