/*
  TemperatureZero.h - Arduino library for internal temperature of the family SAMD -
  Copyright (c) 2018 Electronic Cats.  All right reserved.
    Based in the work of Mitchell Pontague https://github.com/arduino/ArduinoCore-samd/pull/277
*/

#ifndef TEMPERATUREZERO_h
#define TEMPERATUREZERO_h

#define TZ_SAMPLES_1   0
#define TZ_SAMPLES_2   1
#define TZ_SAMPLES_4   2
#define TZ_SAMPLES_8   3
#define TZ_SAMPLES_16  4
#define TZ_SAMPLES_32  5
#define TZ_SAMPLES_64  6
#define TZ_SAMPLES_128 7
#define TZ_SAMPLES_256 8

class TemperatureZero
{
  public:
    TemperatureZero();
    void init();
    void wakeup();
    float readInternalTemperature();
  
  private:
    float _roomTemperature;
    uint16_t _roomReading;
    float _hotTemperature;
    uint16_t _hotReading;
    float _roomInt1vRef;
    float _hotInt1vRef;
    float _roomVoltageCompensated;
    float _hotVoltageCompensated;
    
    void getFactoryCalibration();
    uint16_t readInternalTemperatureRaw(uint8_t);
    float raw2temp(uint16_t);
    float convert_dec_to_frac(uint8_t);
};

#endif

