#include <PulsePosition.h>
PulsePositionInput ReceiverInput(RISING);

float ReceiverValue[] = {0, 0, 0, 0, 0, 0, 0, 0};
int ChannelNumber = 0;

void read_receiver(void) {
  ChannelNumber = ReceiverInput.available();
  if(ChannelNumber > 0) {
    for(int i = 0; i < ChannelNumber; i++)
      ReceiverValue[i-1] = ReceiverInput.read(i);
  }
}


void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); 
  ReceiverInput.begin(15);

}

void loop() {
  read_receiver();
  Serial.print("Number of channels: ");
  Serial.print(ChannelNumber);
  Serial.print(" Roll [us]: ");
  Serial.print(ReceiverValue[0]);
  Serial.print(" Pitch [us]: ");
  Serial.print(ReceiverValue[1]);
  Serial.print(" Throttle [us]: ");
  Serial.print(ReceiverValue[2]);
  Serial.print(" Yaw [us]: ");
  Serial.println(ReceiverValue[3]);
  delay(50);

}
