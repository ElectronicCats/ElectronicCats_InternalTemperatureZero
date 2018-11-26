#include <TemperatureZero.h>

void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin(9600);
  //TemperatureZero.startTemperature();
}

void loop() {
  // put your main code here, to run repeatedly:
  float temperature = TemperatureZero.readInternalTemperature();
  SerialUSB.print("Internal Temperature is:");
  SerialUSB.println(temperature);
  delay(500);
}
