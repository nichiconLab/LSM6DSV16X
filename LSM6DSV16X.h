
#ifndef LSM6DSV16X_H
#define LSM6DSV16X_H

#include <Arduino.h>
#include <SPI.h>

#include "RegisterList.h"

#define SPI_WRITE 0x00
#define SPI_READ  0x80

#define ACCEL_SCALE_2G  0x00
#define ACCEL_SCALE_4G  0x01
#define ACCEL_SCALE_8G  0x02
#define ACCEL_SCALE_16G 0x03

#define GYRO_SCALE_125DPS  0x00
#define GYRO_SCALE_250DPS  0x01
#define GYRO_SCALE_500DPS  0x02
#define GYRO_SCALE_1000DPS 0x03
#define GYRO_SCALE_2000DPS 0x04
#define GYRO_SCALE_4000DPS 0x0C


class LSM6DSV16X{
    public:
        LSM6DSV16X(SPIClass *spi);
        void ResetDevice();
        float SetAccelScale(uint8_t AccelScale);
        float SetGyroScale(uint8_t GyroScale);
        void Init(int CSpin, uint32_t SPI_Freq);
        void ReadAccelRaw(int16_t *Accel);
        void ReadGyroRaw(int16_t *Gyro);
        void ReadAccel(float *Accel);
        void ReadGyro(float *Gyro);

    private:
        SPIClass *mySPI;
        int myCS;
        uint32_t mySPI_Freq;
        float myAccelRes, myGyroRes;
        float K_ToMPS2, K_ToRPS;
        float ConvertAccelToMPS2(int16_t AccelRaw);
        float ConvertGyroToRPS(int16_t GyroRaw);
        void WriteBytes(uint8_t *TxData, uint8_t Length);
        void ReadBytes(uint8_t *RxData, uint8_t Length);
        void WriteRegister(uint8_t RegAddr, uint8_t value);
        void WriteRegister_Block(uint8_t RegAddr, uint8_t *value, uint8_t Length);
        uint8_t ReadRegister(uint8_t RegAddr);
        void ReadRegister_Block(uint8_t RegAddr, uint8_t *value, uint8_t Length);
};

#endif