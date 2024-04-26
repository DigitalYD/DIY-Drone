/* NOTIC: all of code for the receiver to connect to the teensy
  * from Carbon Aeronautics
  * https://github.com/CarbonAeronautics 
  */


#include <PulsePosition.h>
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_LSM6DSOX.h>

Adafruit_MPL3115A2 baro;
Adafruit_LSM6DSOX sox;

PulsePositionInput ReceiverInput(RISING);
float ReceiverValue[]={0, 0, 0, 0, 0, 0, 0, 0};
int ChannelNumber=0; 
int timer;

// Servo motor;
float InputThrottle;

void read_receiver(void){
  ChannelNumber = ReceiverInput.available();
  if (ChannelNumber > 0) {
      for (int i=1; i<=ChannelNumber;i++){
    ReceiverValue[i-1]=ReceiverInput.read(i);
    }
  }
}

void read_barometer(void){
  float altitude = baro.getAltitude();

  Serial.println("-----------------");
  Serial.print("altitude = ");
  Serial.print(altitude);
  Serial.println(" m");

  // delay(250);
}


// setting the motor control TX -> Teensy -> Motor
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


void setup() {

  // ************************************************************
  // Start Up Section 
  // ************************************************************
  // setting the initizing PIN 13 on board LED
  pinMode(13, OUTPUT); 
  digitalWrite(13, HIGH);

  // begin the baud rate at 57,600 
  Serial.begin(115200);

  // ************************************************************
  // Barometer Section 
  // ************************************************************
   while (!Serial);
  Serial.println("Adafruit_MPL3115A2 test!");

  if (!baro.begin()) {
    Serial.println("Could not find sensor. Check wiring.");
    while (1);
  }

  // use to set sea level pressure for current location
  // this is needed for accurate altitude measurement
  // STD SLP = 1013.26 hPa
  baro.setSeaPressure(1013.26);

  // ************************************************************
  // Physical Controller Section 
  // ************************************************************
  // start reading from the signal line of the reciever at PIN 15
  ReceiverInput.begin(15);

  // send PWM to the motor 

  // analogWriteFrequency(PIN, FREQ)
  // set the PIN to send out and the frequency of the signal
  analogWriteFrequency(1, 250);

  // analogWriteResolution(bits)
  analogWriteResolution(12);

  // delay 
  delay(250);

  // reading the throttle of the remote
  while (ReceiverValue[2] < 1020 || ReceiverValue[2] > 1050) {
    read_receiver();
    // delay(2);
  }

}

void loop() {
  
  
  
  // // read_receiver();
  // Serial.print("barometer");
  // Serial.println(timer);
  read_barometer();
  // delay(50);

  InputThrottle=ReceiverValue[2];
  analogWrite(1,1.024*InputThrottle);

  // Serial.print("TX");
  // Serial.println(timer);
  controller();
  
}
