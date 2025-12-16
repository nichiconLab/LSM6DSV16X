#include <Arduino.h>
#include "LSM6DSV16X.h"


#define IMU_CS 10
#define IMU_SPI_FREQ 10000000
LSM6DSV16X IMU(&SPI);

static float Accel[3];
static float Gyro[3];

void setup() {
    Serial.begin(115200);

    pinMode(IMU_CS, OUTPUT);
    digitalWrite(IMU_CS, HIGH);
    SPI.begin();

    IMU.Init(IMU_CS, IMU_SPI_FREQ);
    IMU.SetAccelScale(ACCEL_SCALE_8G);
    IMU.SetGyroScale(GYRO_SCALE_500DPS);
    
}

void loop() {
    IMU.ReadAccel(Accel);
    IMU.ReadGyro(Gyro);
    Serial.printf("ax: %#10.6f, ay: %#10.6f, az: %#10.6f, gx: %#10.6f, gy: %#10.6f, gz: %#10.6f\r\n", Accel[0], Accel[1], Accel[2], Gyro[0], Gyro[1], Gyro[2]);
    delay(100);
}

