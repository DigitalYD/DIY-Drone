/* NOTIC: all of code for the receiver to connect to the teensy
  * from Carbon Aeronautics
  * https://github.com/CarbonAeronautics 
  */


#include <PulsePosition.h>
// #include <Adafruit_MPL3115A2.h>
#include <Adafruit_LSM6DSOX.h>
#include <Wire.h>

PulsePositionInput ReceiverInput(RISING);
float ReceiverValue[]={0, 0, 0, 0, 0, 0, 0, 0};
int ChannelNumber=0; 

Adafruit_MPL3115A2 baro;
float pressure;
float altitude;


void read_receiver(void){
  ChannelNumber = ReceiverInput.available();
  if (ChannelNumber > 0) {
      for (int i=1; i<=ChannelNumber;i++){
    ReceiverValue[i-1]=ReceiverInput.read(i);
    }
  }
}

void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT); 
  digitalWrite(13, HIGH);
  ReceiverInput.begin(15);

  if (!baro.begin()) {
    Serial.println("Could not find sensor. Check wiring.");
    while(1);
  }

  baro.begin();

  baro.setSeaPressure(1013.26);

}

void barometer() {
  baro.startOneShot();

  Serial.print("Pressure: ");
  pressure = baro.getPressure();
  Serial.println(pressure);

  Serial.print("Altitude: ");
  altitude = baro.getAltitude();
  Serial.println(altitude);

  // delay(100);
}

void controller() {
  read_receiver();
  Serial.print("Number of channels: ");
  Serial.print(ChannelNumber);
  Serial.print(" Roll [µs]: ");
  Serial.print(ReceiverValue[0]);
  Serial.print(" Pitch [µs]: "); 
  Serial.print(ReceiverValue[1]);
  Serial.print(" Throttle [µs]: "); 
  Serial.print(ReceiverValue[2]);
  Serial.print(" Yaw [µs]: "); 
  Serial.println(ReceiverValue[3]);
}

void loop() {
  
  controller();
  barometer();
  delay(50);
}
