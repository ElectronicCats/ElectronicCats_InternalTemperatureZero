#include <TemperatureZero.h>
#include <RTCZero.h>

TemperatureZero TempZero = TemperatureZero();
RTCZero rtc;

void setup() {
  // put your setup code here, to run once:

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.begin(115200);

  rtc.begin();
  rtc.setTime(0, 0, 0);
  rtc.setDate(1, 1, 20);
  rtc.setAlarmTime(00, 00, 00);
  rtc.enableAlarm(rtc.MATCH_SS);  // Raise alarm every minute

  delay(5000);   //Delay for programming before sleep
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, HIGH);
  
  TempZero.init();
  delay(500); 
  printTemp();
  delay(500);
  TempZero.disable(); //saves ~60uA in standby
  
  digitalWrite(LED_BUILTIN, LOW);
  rtc.standbyMode();    // Sleep until next alarm match
}

void printTemp() {
  float temperature = TempZero.readInternalTemperature();
  Serial.print("Internal Temperature is : ");
  Serial.println(temperature);
}
