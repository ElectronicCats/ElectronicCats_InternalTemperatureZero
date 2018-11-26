/*
  TemperatureZero.h - Arduino library for internal temperature of the family SAMD -
  Copyright (c) 2018 Electronic Cats.  All right reserved.
  Based in the work of Mitchell Pontague https://github.com/arduino/ArduinoCore-samd/pull/277
*/

#include "Arduino.h"
#include "TemperatureZero.h"

TemperatureZero::TemperatureZero() {
}

void TemperatureZero::init()
{
  SYSCTRL->VREF.reg |= SYSCTRL_VREF_TSEN; // Enable the temperature sensor  

  while( ADC->STATUS.bit.SYNCBUSY == 1 ); // Wait for synchronization of registers between the clock domains
}


// Reads temperature using internal ADC channel
// Datasheet chapter 37.10.8 - Temperature Sensor Characteristics
float TemperatureZero::readInternalTemperature()
{
  // Save ADC settings
  uint16_t oldReadResolution = ADC->CTRLB.reg;
  uint16_t oldSampling = ADC->SAMPCTRL.reg;
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
   // Start ADC conversion
  ADC->SWTRIG.bit.START = 1;
   // Clear the Data Ready flag
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
  while (ADC->STATUS.bit.SYNCBUSY == 1);
   // Start conversion again, since The first conversion after the reference is changed must not be used.
  ADC->SWTRIG.bit.START = 1;
   // Wait until ADC conversion is done
  while (!(ADC->INTFLAG.bit.RESRDY));
  while (ADC->STATUS.bit.SYNCBUSY == 1);
   // Get result
  // This is signed so that the math later is done signed
  int32_t adcReading = ADC->RESULT.reg;
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
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  ADC->INPUTCTRL.bit.GAIN = oldReferenceGain;
  ADC->REFCTRL.bit.REFSEL = oldReferenceSelect;
  while (ADC->STATUS.bit.SYNCBUSY == 1);  
   // Factory room temperature readings
  uint8_t roomInteger = (*(uint32_t*)FUSES_ROOM_TEMP_VAL_INT_ADDR & FUSES_ROOM_TEMP_VAL_INT_Msk) >> FUSES_ROOM_TEMP_VAL_INT_Pos;
  uint8_t roomDecimal = (*(uint32_t*)FUSES_ROOM_TEMP_VAL_DEC_ADDR & FUSES_ROOM_TEMP_VAL_DEC_Msk) >> FUSES_ROOM_TEMP_VAL_DEC_Pos;
  int32_t roomReading = ((*(uint32_t*)FUSES_ROOM_ADC_VAL_ADDR & FUSES_ROOM_ADC_VAL_Msk) >> FUSES_ROOM_ADC_VAL_Pos);
  int32_t roomTemperature = 1000 * roomInteger + 100 * roomDecimal;
   // Factory hot temperature readings
  uint8_t hotInteger = (*(uint32_t*)FUSES_HOT_TEMP_VAL_INT_ADDR & FUSES_HOT_TEMP_VAL_INT_Msk) >> FUSES_HOT_TEMP_VAL_INT_Pos;
  uint8_t hotDecimal = (*(uint32_t*)FUSES_HOT_TEMP_VAL_DEC_ADDR & FUSES_HOT_TEMP_VAL_DEC_Msk) >> FUSES_HOT_TEMP_VAL_DEC_Pos;
  int32_t hotReading = ((*(uint32_t*)FUSES_HOT_ADC_VAL_ADDR & FUSES_HOT_ADC_VAL_Msk) >> FUSES_HOT_ADC_VAL_Pos);
  int32_t hotTemperature = 1000 * hotInteger + 100 * hotDecimal;
   // Linear interpolation of temperature using factory room temperature and hot temperature
  int32_t temperature = roomTemperature + ((hotTemperature - roomTemperature) * (adcReading - roomReading)) / (hotReading - roomReading);
  return temperature / 1000.0f;
}

