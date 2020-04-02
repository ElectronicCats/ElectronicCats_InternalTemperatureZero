#include <TemperatureZero.h>

TemperatureZero TempZero = TemperatureZero();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  TempZero.init();
  
  // Correct the output using a 2-point measurement. That is, a measurement at a known cold 
  // and at a known hot temperature. Ideally, at the edges of the range-of-interest.
  // In order to provide a clear example, we'll setup a case where the sensor mistakenly shifts
  // down by one degree and halves the temperature:
  //    Real cold temp = 10 degres, TempZero indicates (10 - 1) / 2 = 4.5
  //    Real hot temp = 100 degrees, TempZero indicates (100 - 1) / 2 = 49.5
  // We would now expect the user correction to double the temperature and shift it up 1 degree
  TempZero.setUserCalibration2P(10.0, 4.5, 100.0, 49.5, false);

  // As an alternative to the setup of a 2-point calibration measurement,
  // we could create a table with two temperature columns in a spreadsheet. One column for 
  // a known temperature, from a 'trusted' temperature sensor. and one for the measured 
  // TempZero temperature. Ideally this table would cover the range of interest evenly.
  // Then, using a spreadsheet program, we could create a 'linear fit' between both columns. 
  // Search for the function LINEST. This fit yields two numbers: A and B. These numbers 
  // are the best values for using the following formula:
  //    <Real temperature> = A * <measured TempZero temp> + B
  // These A and B values could be used directly in TempZero as follows:
  // TempZero.setUserCalibration(A, -B/A);
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println("\nUser calibration evaluation, expectation T x 2 + 1: ");

  // Below code takes two measurements.
  // One is displayed without user calibration and the other with it.
  // Since there are two separate measurements, the measured temperature could vary between measurements.
  // As the temperature sensor is a bit noisy, this effect means the the user calibration may not seem 100% 
  // accurate all the time. This could be resolved using a higher level of averaging (separate example)

  TempZero.disableUserCalibration();
  float temp = TempZero.readInternalTemperature();
  Serial.print("Internal Temperature is : ");
  Serial.print(temp, 1);
  
  TempZero.enableUserCalibration();
  float userTemp = TempZero.readInternalTemperature();
  Serial.print(", user corrected : ");
  Serial.println(userTemp, 1);
}
