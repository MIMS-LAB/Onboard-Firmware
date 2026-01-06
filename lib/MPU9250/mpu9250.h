#ifndef __MPU9250_H__
#include <Wire.h>
#include<Arduino.h>
#include <stdint.h>

#define __MPU9250_H__

#define ADDR 0x68
#define WIRE Wire
#define  magnet_calib  0.6 // uT/LSB

//IMU Configurations registers
#define GYRO_CONFIG 0x1B
#define ACC_CONFIG 0x1C

//IMU Data measurements registers 

#define ACCEL_XOUT_H 0X3B
#define ACCEL_XOUT_L 0X3C
#define ACCEL_YOUT_H 0X3D
#define ACCEL_YOUT_L 0X3E
#define ACCEL_ZOUT_H 0X3F
#define ACCEL_ZOUT_L 0X40


#define GYRO_XOUT_H 0X43
#define GYRO_XOUT_L 0X44
#define GYRO_YOUT_H 0X45
#define GYRO_YOUT_L 0X46
#define GYRO_ZOUT_H 0X47
#define GYRO_ZOUT_L 0X48

#define MAGNET_XOUT_H 0X4
#define MAGNET_XOUT_L 0X3
#define MAGNET_YOUT_H 0X6
#define MAGNET_YOUT_L 0X5
#define MAGNET_ZOUT_H 0X8
#define MAGNET_ZOUT_L 0X7



//Data Structs

struct AStruct//acceleration
{
    float XAxis,YAxis,ZAxis;
};

struct GStruct//gyroscope
{
    float XAxis,YAxis,ZAxis;
};

struct MStruct//magnometer
{
    float XAxis,YAxis,ZAxis;
};


class MPU{
public:
    //range variables 
    float GyroRange[4]={131.0,65.5,32.8,16.4};
    float AccelRange[4]={16384.0,8192.0,4096.0,2048.0};

    //config functions
    void gyro_setup(int range);
    void acc_setup(int range);

    void init(int accelRange,int gyroRange); 

    //data read functions
    void get_acc(int Anum,struct AStruct *acc);
    void get_gyro(int Gnum,struct GStruct *gyro);
    void get_magnet(struct MStruct *mag); 
};

#endif
