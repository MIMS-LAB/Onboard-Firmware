
#include "MPU6050.h"

uint8_t mpuaddr = 0x68;

void MPU::pwr_setup()
{ // power management registers setup
     WIRE.beginTransmission(ADDR);
     WIRE.write(PWR_MGMT_1);
     WIRE.write(0x01);
     WIRE.endTransmission(true);
}

void MPU::gyro_setup(int range)
{ // gyroscope registers setup
     WIRE.beginTransmission(ADDR);
     WIRE.write(GYRO_CONFIG);
     switch (range)
     {
     case 0:

          WIRE.write(0x00);
          break;
     case 1:
          WIRE.write(0x08);
          break;
     case 2:
          WIRE.write(0x10);
          break;
     case 3:
          WIRE.write(0x18);
          break;
     }
     WIRE.endTransmission(true);
}

void MPU::acc_setup(int range)
{ // accelerometer registers setup
     WIRE.beginTransmission(ADDR);
     WIRE.write(ACC_CONFIG);

     switch (range)

     {
     case 0:
          WIRE.write(0x00);
          break;

     case 1:
          WIRE.write(0x8);
          break;
     case 2:
          WIRE.write(0x10);
          break;
     case 3:

          WIRE.write(0x18);
          break;
     }
     WIRE.endTransmission(true);
}

void MPU::get_acc(int Anum, struct AStruct *acc)
{
     WIRE.beginTransmission(ADDR);
     WIRE.write(ACCEL_XOUT_H);
     WIRE.endTransmission(false);
     WIRE.requestFrom(mpuaddr, (size_t)6, true);

     int16_t xdata = WIRE.read() << 8 | WIRE.read();
     int16_t ydata = WIRE.read() << 8 | WIRE.read();
     int16_t zdata = WIRE.read() << 8 | WIRE.read();

     acc->XAxis = (float)xdata / AccelRange[Anum];
     acc->YAxis = (float)ydata / AccelRange[Anum];
     acc->ZAxis = (float)zdata / AccelRange[Anum];
}

void MPU::get_gyro(int Gnum, struct GStruct *gyro)
{
     WIRE.beginTransmission(ADDR);
     WIRE.write(GYRO_XOUT_H);
     WIRE.endTransmission(true);
     WIRE.requestFrom(mpuaddr, (size_t)6, true);

     int16_t xdata = WIRE.read() << 8 | WIRE.read();
     int16_t ydata = WIRE.read() << 8 | WIRE.read();
     int16_t zdata = WIRE.read() << 8 | WIRE.read();

     gyro->XAxis = (float)xdata / GyroRange[Gnum];
     gyro->YAxis = (float)ydata / GyroRange[Gnum];
     gyro->ZAxis = (float)zdata / GyroRange[Gnum];
}
