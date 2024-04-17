/* NOTIC: all of code for the receiver to connect to the teensy
  * from Carbon Aeronautics
  * https://github.com/CarbonAeronautics 
  */


#include <PulsePosition.h>
#include <Wire.h>
// #include <Adafruit_LSM6DSOX.h>


PulsePositionInput ReceiverInput(RISING);
float ReceiverValue[]={0, 0, 0, 0, 0, 0, 0, 0};
int ChannelNumber=0; 

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

  // begin the baud rate at 57,600 
  Serial.begin(57600);

  // setting the initizing PIN 13 on board LED
  pinMode(13, OUTPUT); 
  digitalWrite(13, HIGH);

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
    delay(4);
  }

}

void loop() {
  
  InputThrottle=ReceiverValue[2];
  analogWrite(1,1.024*InputThrottle);

  controller();
  delay(50);
}
