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

    // motor stuff:
    esc.attach(MOTOR1_PWM_PIN);
    esc2.attach(MOTOR2_PWM_PIN);
    esc3.attach(MOTOR3_PWM_PIN);
    esc4.attach(MOTOR4_PWM_PIN);

    // start serial monitor
    Serial.begin(SERIAL_MONITOR_BAUD);

    Wire.begin();


    ina.config_setup(); // ina260.begin(); //
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

    Serial.println("inside loop");
    uint32_t timestamp = millis();

    radio_read_old = radio_read;
    radio_read_old2 = radio_read2;
    radio_read_old3 = radio_read3;

    setParts();

    // read MS5611
    if (partsStates.baro)
    {
        if (baro.getTempPress(&temp, &pres))
        {
            Serial.printf("baro read failed\n");

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

    batt_volt = ina.get_volt() / 1000.0; // ina260.readBusVoltage()/1000.0; // ;//   //

    SR04_dist_cm = SR04_MICROSECONDS_TO_CENTIMETERS * SR04_Distance();

    // joystick controls:
    if (radio_SERIAL.find('X'))
    {
        radio_read = radio_SERIAL.readStringUntil('\n');
        Serial.print("joystick X: ");
        Serial.println(radio_read.c_str());
 
        if (radio_read != radio_read_old)
        {

            /*
            
         PUT YOUR flight CODE HERE   
            */
        }
    }

    if (radio_SERIAL.find('Y'))
    {

        radio_read2 = radio_SERIAL.readStringUntil('\n');
        Serial.print("joystick Y:");
        Serial.println(radio_read2.c_str());
        if (radio_read2 != radio_read_old2)
        {
            /*
            
         PUT YOUR flight CODE HERE   
            */
        }
    }
    if (radio_SERIAL.find("T"))
    {

        radio_read3 = radio_SERIAL.readStringUntil('\n');
        Serial.print("joystick T:");
        Serial.println(radio_read3.c_str()); 
        DUTY_CYCLE_FRACT_T = mapfloat(radio_read3.toFloat(), -1.0, 1.0, 0.0, 1.0);

         rotateMotor(1, MOTOR1_PWM_PERIOD, DUTY_CYCLE_FRACT_T);
            rotateMotor(2, MOTOR1_PWM_PERIOD, DUTY_CYCLE_FRACT_T);
            
            rotateMotor(3, MOTOR1_PWM_PERIOD, DUTY_CYCLE_FRACT_T);
            rotateMotor(4, MOTOR1_PWM_PERIOD, DUTY_CYCLE_FRACT_T);
        if (radio_read3.toFloat() <= radio_read_old3.toFloat() + thrust_threshold && radio_read3.toFloat() >= radio_read_old3.toFloat() - thrust_threshold)
        {

            /*
            
         PUT YOUR flight CODE HERE   
            */
       
        }
    }

    // motor controls:

    // rotateMotor(MOTOR1_PWM_PERIOD, DUTY_CYCLE_FRACT_Y);

    // print stuff to serial 
    sprintf(
        string, outputFormat,
        timestamp / 1000, imu_acc.XAxis, imu_acc.YAxis, imu_acc.ZAxis, accel_resultant, imu_gyro.XAxis, imu_gyro.YAxis, imu_gyro.ZAxis, temp, pres, alt, (alt - alt_start), batt_volt, lat, lon, gps_quality, gps_alt, angleX_old, angleY_old, angleZ_old, angleX_diff, angleY_diff, angleZ_diff, SR04_dist_cm, 100 * DUTY_CYCLE_FRACT_T);

 Serial.printf("%s", string);
    radio_SERIAL.printf("%s", string);

}