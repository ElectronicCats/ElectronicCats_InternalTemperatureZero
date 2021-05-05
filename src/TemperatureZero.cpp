/*
  TemperatureZero.h - Arduino library for internal temperature of the family SAMD21 and SAMD51 -
  Copyright (c) 2018 Electronic Cats.  All right reserved.
  Based in the work of Mitchell Pontague https://github.com/arduino/ArduinoCore-samd/pull/277
  Based in the work of @manitou48 for SAMD51 https://github.com/manitou48/samd51/blob/master/m4temp.ino 
  and CircuitPython https://github.com/adafruit/circuitpython/blob/master/ports/atmel-samd/common-hal/microcontroller/Processor.c
  Thanks!
*/

#include "Arduino.h"
#include "TemperatureZero.h"

#ifdef __SAMD51__ // M4
// m4 SAMD51 chip temperature sensor on ADC
// Decimal to fraction conversion. (adapted from ASF sample).
//#define NVMCTRL_TEMP_LOG              (0x00800100)  // ref pg 59
#define NVMCTRL_TEMP_LOG NVMCTRL_TEMP_LOG_W0

static float convert_dec_to_frac(uint8_t val) {
  float float_val = (float)val;
  if (val < 10) {
    return (float_val / 10.0);
  } else if (val < 100) {
    return (float_val / 100.0);
  } else {
    return (float_val / 1000.0);
  }
}
#endif

#ifdef _SAMD21_ // M0
// Convert raw 12 bit adc reading into temperature float.
// uses factory calibration data and, only when set and enabled, user calibration data
float TemperatureZero::raw2temp (uint16_t adcReading) {
  // Get course temperature first, in order to estimate the internal 1V reference voltage level at this temperature
  float meaurementVoltage = ((float)adcReading)/ADC_12BIT_FULL_SCALE_VALUE_FLOAT;
  float coarse_temp = _roomTemperature + (((_hotTemperature - _roomTemperature)/(_hotVoltageCompensated - _roomVoltageCompensated)) * (meaurementVoltage - _roomVoltageCompensated));
  // Estimate the reference voltage using the course temperature
  float ref1VAtMeasurement = _roomInt1vRef + (((_hotInt1vRef - _roomInt1vRef) * (coarse_temp - _roomTemperature))/(_hotTemperature - _roomTemperature));
  // Now first compensate the raw adc reading using the estimation of the 1V reference output at current temperature 
  float measureVoltageCompensated = ((float)adcReading * ref1VAtMeasurement)/ADC_12BIT_FULL_SCALE_VALUE_FLOAT;
  // Repeat the temperature interpolation using the compensated measurement voltage
  float refinedTemp = _roomTemperature + (((_hotTemperature - _roomTemperature)/(_hotVoltageCompensated - _roomVoltageCompensated)) * (measureVoltageCompensated - _roomVoltageCompensated));
  float result = refinedTemp;
  if (_isUserCalEnabled) {
    result = (refinedTemp - _userCalOffsetCorrection) * _userCalGainCorrection;
  }
  #ifdef TZ_WITH_DEBUG_CODE
  if (_debug) {
    _debugSerial->println(F("\n+++ Temperature calculation:"));
    _debugSerial->print(F("raw adc reading : "));
    _debugSerial->println(adcReading);
    _debugSerial->print(F("Course temperature : "));
    _debugSerial->println(coarse_temp, 1);
    _debugSerial->print(F("Estimated 1V ref @Course temperature : "));
    _debugSerial->println(ref1VAtMeasurement, 4);
    _debugSerial->print(F("Temperature compensated measurement voltage : "));
    _debugSerial->println(measureVoltageCompensated, 4);
    _debugSerial->print(F("Refined temperature : "));
    _debugSerial->println(refinedTemp, 1);
    _debugSerial->print(F("User calibration post processing is : "));
    if (_isUserCalEnabled) {
      _debugSerial->println(F("Enabled"));
      _debugSerial->print(F("User calibration offset correction : "));
      _debugSerial->println(_userCalOffsetCorrection, 4);
      _debugSerial->print(F("User calibration gain correction : "));
      _debugSerial->println(_userCalGainCorrection, 4);
      _debugSerial->print(F("User calibration corrected temperature = "));
      _debugSerial->println(result, 2);
    } else {
      _debugSerial->println(F("Disabled"));
    }
  }
#endif  
  return result;
}
#endif

#ifdef __SAMD51__ // M4
float TemperatureZero::raw2temp(uint16_t TP, uint16_t TC) {
    uint32_t TLI = (*(uint32_t *)FUSES_ROOM_TEMP_VAL_INT_ADDR & FUSES_ROOM_TEMP_VAL_INT_Msk) >> FUSES_ROOM_TEMP_VAL_INT_Pos;
    uint32_t TLD = (*(uint32_t *)FUSES_ROOM_TEMP_VAL_DEC_ADDR & FUSES_ROOM_TEMP_VAL_DEC_Msk) >> FUSES_ROOM_TEMP_VAL_DEC_Pos;
    float TL = TLI + convert_dec_to_frac(TLD);

    uint32_t THI = (*(uint32_t *)FUSES_HOT_TEMP_VAL_INT_ADDR & FUSES_HOT_TEMP_VAL_INT_Msk) >> FUSES_HOT_TEMP_VAL_INT_Pos;
    uint32_t THD = (*(uint32_t *)FUSES_HOT_TEMP_VAL_DEC_ADDR & FUSES_HOT_TEMP_VAL_DEC_Msk) >> FUSES_HOT_TEMP_VAL_DEC_Pos;
    float TH = THI + convert_dec_to_frac(THD);

    uint16_t VPL = (*(uint32_t *)FUSES_ROOM_ADC_VAL_PTAT_ADDR & FUSES_ROOM_ADC_VAL_PTAT_Msk) >> FUSES_ROOM_ADC_VAL_PTAT_Pos;
    uint16_t VPH = (*(uint32_t *)FUSES_HOT_ADC_VAL_PTAT_ADDR & FUSES_HOT_ADC_VAL_PTAT_Msk) >> FUSES_HOT_ADC_VAL_PTAT_Pos;

    uint16_t VCL = (*(uint32_t *)FUSES_ROOM_ADC_VAL_CTAT_ADDR & FUSES_ROOM_ADC_VAL_CTAT_Msk) >> FUSES_ROOM_ADC_VAL_CTAT_Pos;
    uint16_t VCH = (*(uint32_t *)FUSES_HOT_ADC_VAL_CTAT_ADDR & FUSES_HOT_ADC_VAL_CTAT_Msk) >> FUSES_HOT_ADC_VAL_CTAT_Pos;

    // From SAMD51 datasheet: section 45.6.3.1 (page 1327).
    return (TL*VPH*TC - VPL*TH*TC - TL*VCH*TP + TH*VCL*TP) / (VCL*TP - VCH*TP - VPL*TC + VPH*TC);
}
#endif

TemperatureZero::TemperatureZero() {
}

void TemperatureZero::init() {
#ifdef TZ_WITH_DEBUG_CODE
  _debug = false;
#endif
  _averaging = TZ_AVERAGING_64; // on 48Mhz takes approx 26 ms
  _isUserCalEnabled = false;
  getFactoryCalibration();
  wakeup();
}


// After sleeping, the temperature sensor seems disabled. So, let's re-enable it.
void TemperatureZero::wakeup() {
  #ifdef _SAMD21_
  SYSCTRL->VREF.reg |= SYSCTRL_VREF_TSEN; // Enable the temperature sensor  
  while( ADC->STATUS.bit.SYNCBUSY == 1 ); // Wait for synchronization of registers between the clock domains
  #endif
  #ifdef __SAMD51__
  SUPC->VREF.reg |= SUPC_VREF_TSEN | SUPC_VREF_ONDEMAND; // Enable the temperature sensor  
  while( ADC0->SYNCBUSY.reg == 1 ); // Wait for synchronization of registers between the clock domains
  #endif
}



void TemperatureZero::disable() {
  #ifdef _SAMD21_
  SYSCTRL->VREF.reg &= ~SYSCTRL_VREF_TSEN; // Disable the temperature sensor  
  while( ADC->STATUS.bit.SYNCBUSY == 1 );  // Wait for synchronization of registers between the clock domains
  #endif
  #ifdef __SAMD51__
  SUPC->VREF.reg &= ~SUPC_VREF_TSEN | SUPC_VREF_ONDEMAND; // Disable the temperature sensor  
  while( ADC0->SYNCBUSY.reg == 1 ); // Wait for synchronization of registers between the clock domains
  #endif
}


// Set the sample averaging as the internal sensor is somewhat noisy
// Default value is TZ_AVERAGING_64 which takes approx 26 ms at 48 Mhz clock
void TemperatureZero::setAveraging(uint8_t averaging) {
  _averaging = averaging;
}

// Reads temperature using internal ADC channel
// Datasheet chapter 37.10.8 - Temperature Sensor Characteristics
float TemperatureZero::readInternalTemperature() {
  #ifdef _SAMD21_ // M0
   uint16_t adcReading = readInternalTemperatureRaw();
   return raw2temp(adcReading);
   #endif
   #ifdef __SAMD51__ // M4
   return readInternalTemperatureRaw();
   #endif
}

#ifdef TZ_WITH_DEBUG_CODE
// To follow along, the detailed temperature calculation, enable library debugging
void TemperatureZero::enableDebugging(Stream &debugPort) {
  _debugSerial = &debugPort;
  _debug = true;
}

// Disable library debugging
void TemperatureZero::disableDebugging(void) {
  _debug = false;
}
#endif


// Get all factory calibration parameters and process them
// This includes both the temperature sensor calibration as well as the 1v reference calibration
void TemperatureZero::getFactoryCalibration() {
  #ifdef _SAMD21_ // M0
   // Factory room temperature readings
  uint8_t roomInteger = (*(uint32_t*)FUSES_ROOM_TEMP_VAL_INT_ADDR & FUSES_ROOM_TEMP_VAL_INT_Msk) >> FUSES_ROOM_TEMP_VAL_INT_Pos;
  uint8_t roomDecimal = (*(uint32_t*)FUSES_ROOM_TEMP_VAL_DEC_ADDR & FUSES_ROOM_TEMP_VAL_DEC_Msk) >> FUSES_ROOM_TEMP_VAL_DEC_Pos;
  _roomTemperature = roomInteger + convertDecToFrac(roomDecimal);
  _roomReading = ((*(uint32_t*)FUSES_ROOM_ADC_VAL_ADDR & FUSES_ROOM_ADC_VAL_Msk) >> FUSES_ROOM_ADC_VAL_Pos);
   // Factory hot temperature readings
  uint8_t hotInteger = (*(uint32_t*)FUSES_HOT_TEMP_VAL_INT_ADDR & FUSES_HOT_TEMP_VAL_INT_Msk) >> FUSES_HOT_TEMP_VAL_INT_Pos;
  uint8_t hotDecimal = (*(uint32_t*)FUSES_HOT_TEMP_VAL_DEC_ADDR & FUSES_HOT_TEMP_VAL_DEC_Msk) >> FUSES_HOT_TEMP_VAL_DEC_Pos;
  _hotTemperature = hotInteger + convertDecToFrac(hotDecimal);
  _hotReading = ((*(uint32_t*)FUSES_HOT_ADC_VAL_ADDR & FUSES_HOT_ADC_VAL_Msk) >> FUSES_HOT_ADC_VAL_Pos);
  // Factory internal 1V voltage reference readings at both room and hot temperatures
  int8_t roomInt1vRefRaw = (int8_t)((*(uint32_t*)FUSES_ROOM_INT1V_VAL_ADDR & FUSES_ROOM_INT1V_VAL_Msk) >> FUSES_ROOM_INT1V_VAL_Pos);
  int8_t hotInt1vRefRaw  = (int8_t)((*(uint32_t*)FUSES_HOT_INT1V_VAL_ADDR & FUSES_HOT_INT1V_VAL_Msk) >> FUSES_HOT_INT1V_VAL_Pos);
  _roomInt1vRef = 1 - ((float)roomInt1vRefRaw/INT1V_DIVIDER_1000);
  _hotInt1vRef  = 1 - ((float)hotInt1vRefRaw/INT1V_DIVIDER_1000);
  // Combining the temperature dependent 1v reference with the ADC readings
  _roomVoltageCompensated = ((float)_roomReading * _roomInt1vRef)/ADC_12BIT_FULL_SCALE_VALUE_FLOAT;
  _hotVoltageCompensated = ((float)_hotReading * _hotInt1vRef)/ADC_12BIT_FULL_SCALE_VALUE_FLOAT;
  #endif
  #ifdef __SAMD51__
  uint32_t TLI = (*(uint32_t *)FUSES_ROOM_TEMP_VAL_INT_ADDR & FUSES_ROOM_TEMP_VAL_INT_Msk) >> FUSES_ROOM_TEMP_VAL_INT_Pos;
  uint32_t TLD = (*(uint32_t *)FUSES_ROOM_TEMP_VAL_DEC_ADDR & FUSES_ROOM_TEMP_VAL_DEC_Msk) >> FUSES_ROOM_TEMP_VAL_DEC_Pos;
  float TL = TLI + convert_dec_to_frac(TLD);

  uint32_t THI = (*(uint32_t *)FUSES_HOT_TEMP_VAL_INT_ADDR & FUSES_HOT_TEMP_VAL_INT_Msk) >> FUSES_HOT_TEMP_VAL_INT_Pos;
  uint32_t THD = (*(uint32_t *)FUSES_HOT_TEMP_VAL_DEC_ADDR & FUSES_HOT_TEMP_VAL_DEC_Msk) >> FUSES_HOT_TEMP_VAL_DEC_Pos;
  float TH = THI + convert_dec_to_frac(THD);

  uint16_t VPL = (*(uint32_t *)FUSES_ROOM_ADC_VAL_PTAT_ADDR & FUSES_ROOM_ADC_VAL_PTAT_Msk) >> FUSES_ROOM_ADC_VAL_PTAT_Pos;
  uint16_t VPH = (*(uint32_t *)FUSES_HOT_ADC_VAL_PTAT_ADDR & FUSES_HOT_ADC_VAL_PTAT_Msk) >> FUSES_HOT_ADC_VAL_PTAT_Pos;
  uint16_t VCL = (*(uint32_t *)FUSES_ROOM_ADC_VAL_CTAT_ADDR & FUSES_ROOM_ADC_VAL_CTAT_Msk) >> FUSES_ROOM_ADC_VAL_CTAT_Pos;
  uint16_t VCH = (*(uint32_t *)FUSES_HOT_ADC_VAL_CTAT_ADDR & FUSES_HOT_ADC_VAL_CTAT_Msk) >> FUSES_HOT_ADC_VAL_CTAT_Pos;

  #endif
#ifdef TZ_WITH_DEBUG_CODE
  if (_debug) {
    _debugSerial->println(F("\n+++ Factory calibration parameters:"));
    _debugSerial->print(F("Room Temperature : "));
    _debugSerial->println(_roomTemperature, 1);
    _debugSerial->print(F("Hot Temperature  : "));
    _debugSerial->println(_hotTemperature, 1);
    _debugSerial->print(F("Room Reading     : "));
    _debugSerial->println(_roomReading);
    _debugSerial->print(F("Hot Reading      : "));
    _debugSerial->println(_hotReading);
    _debugSerial->print(F("Room Voltage ref raw / interpreted : "));
    _debugSerial->print(roomInt1vRefRaw);
    _debugSerial->print(F(" / "));
    _debugSerial->println(_roomInt1vRef, 4);
    _debugSerial->print(F("Hot Voltage ref raw / interpreted  : "));
    _debugSerial->print(hotInt1vRefRaw);
    _debugSerial->print(F(" / "));
    _debugSerial->println(_hotInt1vRef, 4);
    _debugSerial->print(F("Room Reading compensated : "));
    _debugSerial->println(_roomVoltageCompensated, 4);
    _debugSerial->print(F("Hot Reading compensated  : "));
    _debugSerial->println(_hotVoltageCompensated, 4);
  }
#endif
}


// Extra safe decimal to fractional conversion
float TemperatureZero::convertDecToFrac(uint8_t val) {
  if (val < 10) {
    return ((float)val/10.0);
  } else if (val <100) {
    return ((float)val/100.0);
  } else {
    return ((float)val/1000.0);
  }
}

// Set user calibration params, using two point linear interpolation for hot and cold measurements
void TemperatureZero::setUserCalibration2P(float userCalColdGroundTruth,
                                            float userCalColdMeasurement,
                                            float userCalHotGroundTruth,
                                            float userCalHotMeasurement,
                                            bool isEnabled) {
  _userCalOffsetCorrection = userCalColdMeasurement - userCalColdGroundTruth * (userCalHotMeasurement - userCalColdMeasurement) / (userCalHotGroundTruth - userCalColdGroundTruth);
  _userCalGainCorrection = userCalHotGroundTruth / (userCalHotMeasurement - _userCalOffsetCorrection);
  _isUserCalEnabled = isEnabled;
}

// Set user calibration params explixitly
void TemperatureZero::setUserCalibration(float userCalGainCorrection,
                                          float userCalOffsetCorrection,
                                          bool isEnabled) {
  _userCalOffsetCorrection = userCalOffsetCorrection;
  _userCalGainCorrection = userCalGainCorrection;
  _isUserCalEnabled = isEnabled;
}

void TemperatureZero::enableUserCalibration() {
  _isUserCalEnabled = true;
}

void TemperatureZero::disableUserCalibration() {
  _isUserCalEnabled = false;
}

// Get raw 12 bit adc reading
uint16_t TemperatureZero::readInternalTemperatureRaw() {

#ifdef _SAMD21_ // M0
  // Save ADC settings
  uint16_t oldReadResolution = ADC->CTRLB.reg;
  uint16_t oldSampling = ADC->SAMPCTRL.reg;
  uint16_t oldSampleAveraging = ADC->SAMPCTRL.reg;
  uint16_t oldReferenceGain = ADC->INPUTCTRL.bit.GAIN;
  uint16_t oldReferenceSelect = ADC->REFCTRL.bit.REFSEL;

  // Set to 12 bits resolution
  ADC->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT | ADC_CTRLB_PRESCALER_DIV256;
  // Wait for synchronization of registers between the clock domains
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  // Ensure we are sampling slowly
  ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(0x3f);
  while (ADC->STATUS.bit.SYNCBUSY == 1);
   // Set ADC reference to internal 1v
  ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;
  ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INT1V_Val;
  while (ADC->STATUS.bit.SYNCBUSY == 1);
   // Select MUXPOS as temperature channel, and MUXNEG  as internal ground
  ADC->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_TEMP_Val;
  ADC->INPUTCTRL.bit.MUXNEG = ADC_INPUTCTRL_MUXNEG_GND_Val; 
  while (ADC->STATUS.bit.SYNCBUSY == 1);
   // Enable ADC
  ADC->CTRLA.bit.ENABLE = 1;
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  // Start ADC conversion & discard the first sample
  ADC->SWTRIG.bit.START = 1;
  // Wait until ADC conversion is done, prevents the unexpected offset bug
  while (!(ADC->INTFLAG.bit.RESRDY));
   // Clear the Data Ready flag
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
  // perform averaging
  switch(_averaging) {
    case TZ_AVERAGING_1: 
      ADC->AVGCTRL.reg = 0;
      break;
    case TZ_AVERAGING_2: 
      ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_2 | ADC_AVGCTRL_ADJRES(0x1);
      break;
    case TZ_AVERAGING_4: 
      ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_4 | ADC_AVGCTRL_ADJRES(0x2);
      break;
    case TZ_AVERAGING_8: 
      ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_8 | ADC_AVGCTRL_ADJRES(0x3);
      break;
    case TZ_AVERAGING_16: 
      ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_16 | ADC_AVGCTRL_ADJRES(0x4);
      break;
    case TZ_AVERAGING_32: 
      ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_32 | ADC_AVGCTRL_ADJRES(0x4);
      break;
    case TZ_AVERAGING_64: 
      ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_64 | ADC_AVGCTRL_ADJRES(0x4);
      break;
    case TZ_AVERAGING_128: 
      ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_128 | ADC_AVGCTRL_ADJRES(0x4);
      break;
    case TZ_AVERAGING_256: 
      ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_256 | ADC_AVGCTRL_ADJRES(0x4);
      break;
  }
  while (ADC->STATUS.bit.SYNCBUSY == 1);
   // Start conversion again, since The first conversion after the reference is changed must not be used.
  ADC->SWTRIG.bit.START = 1;
   // Wait until ADC conversion is done
  while (!(ADC->INTFLAG.bit.RESRDY));
  while (ADC->STATUS.bit.SYNCBUSY == 1);
   // Get result
  uint16_t adcReading = ADC->RESULT.reg;
   // Clear result ready flag
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY; 
    while (ADC->STATUS.bit.SYNCBUSY == 1);
   // Disable ADC
  ADC->CTRLA.bit.ENABLE = 0; 
  while (ADC->STATUS.bit.SYNCBUSY == 1);
   // Restore pervious ADC settings
  ADC->CTRLB.reg = oldReadResolution;
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  ADC->SAMPCTRL.reg = oldSampling;
  ADC->SAMPCTRL.reg = oldSampleAveraging;
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  ADC->INPUTCTRL.bit.GAIN = oldReferenceGain;
  ADC->REFCTRL.bit.REFSEL = oldReferenceSelect;
  while (ADC->STATUS.bit.SYNCBUSY == 1); 

  return adcReading;
  #endif

 #ifdef __SAMD51__ // M4
  // enable and read 2 ADC temp sensors, 12-bit res
  volatile uint16_t ptat;
  volatile uint16_t ctat;

  SUPC->VREF.reg |= SUPC_VREF_TSEN | SUPC_VREF_ONDEMAND; // activate temperature sensor
  ADC0->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
  while (ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB); //wait for sync
  while ( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL ); //wait for sync
  ADC0->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_PTAT;
  while ( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
  ADC0->CTRLA.bit.ENABLE = 0x01;             // Enable ADC

  // Start conversion
  while ( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync

  ADC0->SWTRIG.bit.START = 1;

  // Clear the Data Ready flag
  ADC0->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  // Start conversion again, since The first conversion after the reference is changed must not be used.
  ADC0->SWTRIG.bit.START = 1;

  while (ADC0->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete
  ptat = ADC0->RESULT.reg;

  while ( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL ); //wait for sync
  ADC0->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_CTAT;
  // Start conversion
  while ( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync

  ADC0->SWTRIG.bit.START = 1;

  // Clear the Data Ready flag
  ADC0->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  // Start conversion again, since The first conversion after the reference is changed must not be used.
  ADC0->SWTRIG.bit.START = 1;

  while (ADC0->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete
  ctat = ADC0->RESULT.reg;


  while ( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
  ADC0->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
  while ( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync 

  return raw2temp(ptat, ctat);
  
  #endif
}