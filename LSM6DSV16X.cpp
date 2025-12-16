
#include "LSM6DSV16X.h"

LSM6DSV16X::LSM6DSV16X(SPIClass *spi){
    mySPI = spi;
}

void LSM6DSV16X::ResetDevice(){
    uint8_t Reg_CTRL3 = ReadRegister(CTRL3) | 0x01;
    WriteRegister(CTRL3, Reg_CTRL3);  //Software reset
}

float LSM6DSV16X::SetAccelScale(uint8_t AccelScale){
    float AccelResolution;

    switch(AccelScale){
        case ACCEL_SCALE_2G:
            AccelResolution = 2.0/32768;
            break;
        
        case ACCEL_SCALE_4G:
            AccelResolution = 4.0/32768;
            break;

        case ACCEL_SCALE_8G:
            AccelResolution = 8.0/32768;
            break;

        case ACCEL_SCALE_16G:
            AccelResolution = 16.0/32768;
            break;

        default:
            AccelResolution = -1.0;
            break;
    }
    if(AccelResolution != -1.0){
        myAccelRes = AccelResolution;
        K_ToMPS2 = AccelResolution*9.8;
        uint8_t CTRL8value = (ReadRegister(CTRL8) & 0xFC) | AccelScale;
        WriteRegister(CTRL8, CTRL8value);
    }

    return AccelResolution;
}

float LSM6DSV16X::SetGyroScale(uint8_t GyroScale){
    float GyroResolution;

    switch(GyroScale){
        case GYRO_SCALE_125DPS:
            //GyroResolution = 125.0/32768;
            GyroResolution = 0.004375;
            break;

        case GYRO_SCALE_250DPS:
            //GyroResolution = 250.0/32768;
            GyroResolution = 0.00875;
            break;

        case GYRO_SCALE_500DPS:
            //GyroResolution = 500.0/32768;
            GyroResolution = 0.0175;
            break;

        case GYRO_SCALE_1000DPS:
            //GyroResolution = 1000.0/32768;
            GyroResolution = 0.035;
            break;

        case GYRO_SCALE_2000DPS:
            //GyroResolution = 2000.0/32768;
            GyroResolution = 0.070;
            break;

        case GYRO_SCALE_4000DPS:
            //GyroResolution = 4000.0/32768;
            GyroResolution = 0.140;
            break;

        default:
            GyroResolution = -1.0;
            break;
    }
    if(GyroResolution != -1.0){
        myGyroRes = GyroResolution;
        K_ToRPS = GyroResolution*0.0174533;  // pi/180=0.01745329...
        uint8_t CTRL6value = (ReadRegister(CTRL6) & 0xF0) | GyroScale;
        WriteRegister(CTRL6, CTRL6value);
    }

    return GyroResolution;
}

void LSM6DSV16X::Init(int CSpin, uint32_t SPI_Freq){
    myCS = CSpin;
    mySPI_Freq = SPI_Freq;

    //pinMode(myCS, OUTPUT);
    //digitalWrite(myCS, HIGH);

    ResetDevice();
    delay(10);

    WriteRegister(CTRL1, 0x0C);
    WriteRegister(CTRL2, 0x0C);
    WriteRegister(CTRL3, 0x04);

    //SetAccelScale(ACCEL_SCALE_8G);
    //SetGyroScale(GYRO_SCALE_500DPS);
}

void LSM6DSV16X::ReadAccelRaw(int16_t *Accel){
    uint8_t AccelReg[6];

    ReadRegister_Block(OUTX_L_A, AccelReg, 6);
    Accel[0] = ((int16_t)AccelReg[1] << 8) | AccelReg[0];
    Accel[1] = ((int16_t)AccelReg[3] << 8) | AccelReg[2];
    Accel[2] = ((int16_t)AccelReg[5] << 8) | AccelReg[4];
}

void LSM6DSV16X::ReadGyroRaw(int16_t *Gyro){
    uint8_t GyroReg[6];

    ReadRegister_Block(OUTX_L_G, GyroReg, 6);
    Gyro[0] = ((int16_t)GyroReg[1] << 8) | GyroReg[0];
    Gyro[1] = ((int16_t)GyroReg[3] << 8) | GyroReg[2];
    Gyro[2] = ((int16_t)GyroReg[5] << 8) | GyroReg[4];
}

/* Return value at m/s^2 */
void LSM6DSV16X::ReadAccel(float *Accel){
    int16_t AccelValueRaw[3];

    ReadAccelRaw(AccelValueRaw);
    Accel[0] = ConvertAccelToMPS2(AccelValueRaw[0]);
    Accel[1] = ConvertAccelToMPS2(AccelValueRaw[1]);
    Accel[2] = ConvertAccelToMPS2(AccelValueRaw[2]);
}

/* Return value at rad/s */
void LSM6DSV16X::ReadGyro(float *Gyro){
    int16_t GyroValueRaw[3];

    ReadGyroRaw(GyroValueRaw);
    Gyro[0] = ConvertGyroToRPS(GyroValueRaw[0]);
    Gyro[1] = ConvertGyroToRPS(GyroValueRaw[1]);
    Gyro[2] = ConvertGyroToRPS(GyroValueRaw[2]);
}



float LSM6DSV16X::ConvertAccelToMPS2(int16_t AccelRaw){
    return (AccelRaw*K_ToMPS2);
}

float LSM6DSV16X::ConvertGyroToRPS(int16_t GyroRaw){
    return (GyroRaw*K_ToRPS);
}

void LSM6DSV16X::WriteBytes(uint8_t *TxData, uint8_t Length){
    SPISettings mySettings = SPISettings(mySPI_Freq, MSBFIRST, SPI_MODE3);
    mySPI->beginTransaction(mySettings);
    digitalWrite(myCS, LOW);
    for(uint8_t n=0; n<Length; n++){
        mySPI->transfer(TxData[n]);
    }
    digitalWrite(myCS, HIGH);
    mySPI->endTransaction();
}

void LSM6DSV16X::ReadBytes(uint8_t *RxData, uint8_t Length){
    SPISettings mySettings = SPISettings(mySPI_Freq, MSBFIRST, SPI_MODE3);
    mySPI->beginTransaction(mySettings);
    digitalWrite(myCS, LOW);
    for(uint8_t n=0; n<Length; n++){
        RxData[n] = mySPI->transfer(0);
    }
    digitalWrite(myCS, HIGH);
    mySPI->endTransaction();
}

void LSM6DSV16X::WriteRegister(uint8_t RegAddr, uint8_t value){
    SPISettings mySettings = SPISettings(mySPI_Freq, MSBFIRST, SPI_MODE3);
    mySPI->beginTransaction(mySettings);
    digitalWrite(myCS, LOW);
    mySPI->transfer(RegAddr);
    mySPI->transfer(value);
    
    digitalWrite(myCS, HIGH);
    mySPI->endTransaction();
}

void LSM6DSV16X::WriteRegister_Block(uint8_t RegAddr, uint8_t *value, uint8_t Length){
    SPISettings mySettings = SPISettings(mySPI_Freq, MSBFIRST, SPI_MODE3);
    mySPI->beginTransaction(mySettings);
    digitalWrite(myCS, LOW);
    mySPI->transfer(RegAddr);
    for(uint8_t n=0; n<Length; n++){
        mySPI->transfer(value[n]);
    }
    digitalWrite(myCS, HIGH);
    mySPI->endTransaction();
}

uint8_t LSM6DSV16X::ReadRegister(uint8_t RegAddr){
    SPISettings mySettings = SPISettings(mySPI_Freq, MSBFIRST, SPI_MODE3);
    mySPI->beginTransaction(mySettings);
    digitalWrite(myCS, LOW);
    mySPI->transfer(RegAddr | SPI_READ);
    uint8_t RxData = mySPI->transfer(0);
    digitalWrite(myCS, HIGH);
    mySPI->endTransaction();

    return RxData;
}

void LSM6DSV16X::ReadRegister_Block(uint8_t RegAddr, uint8_t *value, uint8_t Length){
    SPISettings mySettings = SPISettings(mySPI_Freq, MSBFIRST, SPI_MODE3);
    mySPI->beginTransaction(mySettings);
    digitalWrite(myCS, LOW);
    mySPI->transfer(RegAddr | SPI_READ);
    for(uint8_t n=0; n<Length; n++){
        value[n] = mySPI->transfer(0);
    }
    digitalWrite(myCS, HIGH);
    mySPI->endTransaction();
}

