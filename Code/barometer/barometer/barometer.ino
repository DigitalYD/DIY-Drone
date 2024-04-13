#include <Adafruit_MPL3115A2.h>
#include <Wire.h>

Adafruit_MPL3115A2 baro;

float pressure;
float altitude;

void setup() {
  Serial.begin(9600);
  
  if (!baro.begin()) {
    Serial.println("Could not find sensor. Check wiring.");
    while(1);
  }

  baro.begin();

  baro.setSeaPressure(1013.26);
}

void loop() {

  
  baro.startOneShot();

  Serial.print("Pressure: ");
  pressure = baro.getPressure();
  Serial.println(pressure);

  Serial.print("Altitude: ");
  altitude = baro.getAltitude();
  Serial.println(altitude);

  delay(100);



}
