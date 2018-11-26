/*
  TemperatureZero.h - Arduino library for internal temperature of the family SAMD -
  Copyright (c) 2018 Electronic Cats.  All right reserved.
    Based in the work of Mitchell Pontague https://github.com/arduino/ArduinoCore-samd/pull/277
*/

#ifndef TEMPERATUREZERO_h
#define TEMPERATUREZERO_h

class TemperatureZero
{
  public:
    TemperatureZero();
    void init();
    float readInternalTemperature();
};

#endif

