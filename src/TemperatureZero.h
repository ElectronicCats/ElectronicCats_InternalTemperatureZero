/*
  TemperatureZero.h - Arduino library for internal temperature of the family SAMD21 and SAMD51 -
  Copyright (c) 2018 Electronic Cats.  All right reserved.
  Based in the work of Mitchell Pontague https://github.com/arduino/ArduinoCore-samd/pull/277
  Based in the work of @manitou48 for SAMD51 https://github.com/manitou48/samd51/blob/master/m4temp.ino 
  and CircuitPython https://github.com/adafruit/circuitpython/blob/master/ports/atmel-samd/common-hal/microcontroller/Processor.c
  Thanks!
*/

#ifndef TEMPERATUREZERO_h
#define TEMPERATUREZERO_h

#define TZ_AVERAGING_1   0
#define TZ_AVERAGING_2   1
#define TZ_AVERAGING_4   2
#define TZ_AVERAGING_8   3
#define TZ_AVERAGING_16  4
#define TZ_AVERAGING_32  5
#define TZ_AVERAGING_64  6
#define TZ_AVERAGING_128 7
#define TZ_AVERAGING_256 8

class TemperatureZero
{
  public:
    TemperatureZero();
    void init();
    void wakeup();
    void disable();
    void setAveraging(uint8_t averaging);
    void setUserCalibration2P(float userCalColdGroundTruth,
                            float userCalColdMeasurement,
                            float userCalHotGroundTruth,
                            float userCalHotMeasurement,
                            bool isEnabled);
    void setUserCalibration(float userCalGainCorrection,
                            float userCalOffsetCorrection,
                            bool isEnabled);
    void enableUserCalibration();
    void disableUserCalibration();
    uint16_t readInternalTemperatureRaw();
    float raw2temp (uint16_t adcReading);
    float raw2temp(uint16_t TP, uint16_t TC);
    float readInternalTemperature();
#ifdef TZ_WITH_DEBUG_CODE
    void enableDebugging(Stream &debugPort);
    void disableDebugging(void); 
#endif
  
  private:
#ifdef TZ_WITH_DEBUG_CODE
    bool _debug;
    Stream * _debugSerial;
#endif
    uint8_t _averaging;

    float _roomTemperature;
    uint16_t _roomReading;
    float _hotTemperature;
    uint16_t _hotReading;
    float _roomInt1vRef;
    float _hotInt1vRef;
    float _roomVoltageCompensated;
    float _hotVoltageCompensated;

    bool _isUserCalEnabled;
    float _userCalGainCorrection;
    float _userCalOffsetCorrection;
    
    void getFactoryCalibration();
    float convertDecToFrac(uint8_t);
};

#endif

