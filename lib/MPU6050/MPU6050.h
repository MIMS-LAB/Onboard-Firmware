#ifndef __MPU6050_H__
#include <Wire.h>
#include<Arduino.h>
#include <stdint.h>

#define __MPU6050_H__

#define ADDR 0x68
#define WIRE Wire1
//IMU Configurations registers
#define PWR_MGMT_1 0x6B
#define GYRO_CONFIG 0x1B
#define ACC_CONFIG 0x1C

//IMU Data measurements registers 

#define ACCEL_XOUT_H 0X3B
#define ACCEL_XOUT_L 0X3C
#define ACCEL_YOUT_H 0X3D
#define ACCEL_YOUT_L 0X3E
#define ACCEL_ZOUT_H 0X3F
#define ACCEL_ZOUT_L 0X40

#define TEMP_OUT_H 0X41
#define TEMP_OUT_L 0X42

#define GYRO_XOUT_H 0X43
#define GYRO_XOUT_L 0X44
#define GYRO_YOUT_H 0X45
#define GYRO_YOUT_L 0X46
#define GYRO_ZOUT_H 0X47
#define GYRO_ZOUT_L 0X48

//Data Structs

struct AStruct//acceleration
{
    float XAxis,YAxis,ZAxis;
};

struct GStruct//gyroscope
{
    float XAxis,YAxis,ZAxis;
};

struct TStruct//temperature
{
    float TempC;
};



class MPU{
public:
    //error check
    bool readFail=false;
    bool getErr();
    //range variables 
    float GyroRange[4]={131.0,65.5,32.8,16.4};
    float AccelRange[4]={16384.0,8192.0,4096.0,2048.0};

    //config functions
    void pwr_setup();
    void gyro_setup(int range);
    void acc_setup(int range);
        
    //data read functions
    void get_acc(int Anum,struct AStruct *acc);
    void get_temp(struct TStruct *temp);
    void get_gyro(int Gnum,struct GStruct *gyro);

};

#endif