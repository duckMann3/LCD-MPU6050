#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu6050;

void setup(void)
{
    Serial.begin(9600);
    Wire.begin();
    mpu6050.begin();

    if(!mpu6050.available())
    {
        Serial.print("Unable to connect to MPU!");
        while(1);
    }
    
    // set the scaling registers
    mpu6050.setAccConfig(MPU6050_ACC_FULL_SCALE_2_G);
    mpu6050.setGyroConfig(MPU6050_GYRO_FULL_SCALE_500_DPS);

    // normalise attitude readings between -180 and 180 degrees
    mpu6050.setRange(RANGE_RELATIVE);

    // run the self calibration function, must not move the MPU during this function call!
    mpu6050.calibrate();
}


void loop(void)
{
    mpu6050.update();
    attitude_t attitude = mpu6050.getAttitude(UNITS_DEGREES);

    Serial.print("Roll: ");
    Serial.print(attitude.roll);
    Serial.print("\tPitch: ");
    Serial.print(attitude.pitch);
    Serial.print("\tYaw: ");
    Serial.println(attitude.yaw);
    Serial.flush();

    delay(500);
}



