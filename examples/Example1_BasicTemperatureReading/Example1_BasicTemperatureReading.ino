/***************************************************************************
  This is a library for internal temperature of the family SAMD

  Electronic Cats invests time and resources providing this open source code,
  please support Electronic Cats and open-source hardware by purchasing products
  from Electronic Cats!

  Written by Andrés Sabas Electronic Cats.
  This code is beerware; if you see me (or any other Electronic Cats 
  member) at the local, and you've found our code helpful, 
  please buy us a round!
  Distributed as-is; no warranty is given.
 ***************************************************************************/

#include <TemperatureZero.h>

TemperatureZero TempZero = TemperatureZero();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  TempZero.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  float temperature = TempZero.readInternalTemperature();
  Serial.print("Internal Temperature is : ");
  Serial.println(temperature);
  delay(500);
}
