/* Notice: this code is referenced from youtube and other materials
 * https://github.com/CarbonAeronautics
 * https://www.adafruit.com/product/4517
 * https://github.com/dbcarelli/teensy-kalman
 * https://www.youtube.com/watch?v=ruB917YmtgE
 * https://github.com/adafruit/Adafruit_MPL3115A2_Library
 */

#include <PulsePosition.h>
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_LSM6DSOX.h>

Adafruit_MPL3115A2 baro;
Adafruit_LSM6DSOX sox;

// TX/RX
PulsePositionInput ReceiverInput(RISING);
float ReceiverValue[] = {0, 0, 0, 0, 0, 0, 0, 0};
int ChannelNumber = 0;

// drone motors
float motor1, motor2, motor3, motor4;

// pid values
float RatePitch, RateRoll, RateYaw;
float CalibrateP, CalibrateR, CalibrateY;
int CalibrateNum;

uint32_t timer;

float DesiredPitch, DesiredRoll, DesiredYaw;
float ErrorPitch, ErrorRoll, ErrorYaw;
float Throttle, Pitch, Roll, Yaw;
float PrevErrorPitch, PrevErrorRoll, PrevErrorYaw;
float PrevIPitch, PrevIRoll, PrevIYaw;
float PIDReturn[] = {0, 0, 0};

float PRoll = 5;
float PPitch = PRoll;
float PYaw = 2;
float IRoll = 1.5;
float IPitch = IRoll;
float IYaw = 10;
float DRoll = 0.03;
float DPitch = DRoll;
float DYaw = 0;

// imu
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

// Kalman Filters
float Kr=0, Kr_uncertain=2*3;
float Ka=0, Ka_uncertain=2*3;
float K_result[]={0,0};

/* the kalman filter is a type of digital filter
 * where it would take in the input and based on that 
 * it would predict the next state 
 * with this filter it should reduce the noise caused by the motors
 * and other natural properties*/
void kalman_filter(float k_state, float k_uncertain, float k_in, float k_measured) {
  k_state=k_state+0.004*k_in;
  k_uncertain=k_uncertain + 0.004 * 0.004 * 4 * 4;
  float k_g=k_uncertain * 1/(1*k_uncertain + 3 * 3);
  k_state=k_state+k_g * (k_measured-k_state);
  k_uncertain=(1-k_g) * k_uncertain;
  K_result[0]=k_state; 
  K_result[1]=k_uncertain;
}

/* this is the function for reading in the remote commands */
void read_receiver(void)
{
  ChannelNumber = ReceiverInput.available();
  if (ChannelNumber > 0)
  {
    for (int i = 1; i <= ChannelNumber; i++)
    {
      ReceiverValue[i - 1] = ReceiverInput.read(i);
    }
  }

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

/* calling this function to get the altitude
 * abandoned due to no accurate altitude and 
 * causes a mysterious delay */
void read_barometer(void)
{
  float altitude = baro.getAltitude();

}

/* this is the function for reading the LSM6DSOX sensor
 * the value that are read is stored and calculates the 
 * AngleRoll and AnglePitch that is used for later */
void read_imu(void)
{
  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  sox.getEvent(&accel, &gyro, &temp);

  RateRoll=(float)gyro.gyro.x;
  RatePitch=(float)gyro.gyro.y;
  RateYaw=(float)gyro.gyro.z;

  AccX=(float)accel.acceleration.x;
  AccY=(float)accel.acceleration.y;
  AccZ=(float)accel.acceleration.z;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);

  Serial.print("Acceleration X [g]= ");
  Serial.print(AccX);
  Serial.print(" Acceleration Y [g]= ");
  Serial.print(AccY);
  Serial.print(" Acceleration Z [g]= ");
  Serial.println(AccZ);

  Serial.print("old AngleRoll: "); Serial.println(AngleRoll);
  Serial.print("old AnglePitch: "); Serial.println(AnglePitch);
  

}

void pid_controller(float Error, float P, float I, float D, float PrevError, float PrevIterm)
{
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;
  if (Iterm > 500)
    Iterm = 500;
  else if (Iterm < -500)
    Iterm = -500;
  float Dterm = D * (Error - PrevError) / 0.004;
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 500)
    PIDOutput = 500;
  else if (PIDOutput < -500)
    PIDOutput = -500;

  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void reset_pid(void)
{
  PrevErrorRoll = 0;
  PrevErrorPitch = 0;
  PrevErrorYaw = 0;
  PrevIRoll = 0;
  PrevIPitch = 0;
  PrevIYaw = 0;
}

void setup()
{
  // ************************************************************
  // Start Up Section
  // ************************************************************
  // begin the baud rate at 57,600
  Serial.begin(115200);

  // setting the initizing PIN 13 on board LED
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // ************************************************************
  // IMU Section
  // ************************************************************
  while (!Serial)
  {
    Serial.println("fail IMU");
  }

  Serial.println("Adafruit LSM6DSOX test!");

  if (!sox.begin_I2C())
  {
    while (1)
    {
      Serial.println("Could not find IMU. Check wiring.");
      delay(10);
    }
  }

  Serial.println("IMU Found!");

  // ************************************************************
  // Barometer Section
  // ************************************************************
  while (!Serial)
  {
    Serial.println("fail Barometer");
  }

  Serial.println("Adafruit_MPL3115A2 test!");

  if (!baro.begin())
  {
    Serial.println("Could not find Barometer. Check wiring.");
    while (1)
      ;
  }

  Serial.println("Barometer Found!");

  // baro.setAltitudeOffset(baro.getAltitudeOffset());

  // set sea level pressure = 1013.26 hPa
  baro.setSeaPressure(1013.26);

  // ************************************************************
  // Physical Controller Section
  // ************************************************************
  // start reading from the signal line of the reciever at PIN 15
  ReceiverInput.begin(15);

  =
  // set the PIN to send out and the frequency of the signal
  CalibrateR /= 2000;
  CalibrateP /= 2000;
  CalibrateY /= 2000;

  analogWriteFrequency(1, 250);
  analogWriteFrequency(2, 250);
  analogWriteFrequency(3, 250);
  analogWriteFrequency(4, 250);

  // analogWriteResolution(bits)
  analogWriteResolution(12);

  // reading the throttle of the remote
  while (ReceiverValue[2] < 1020 || ReceiverValue[2] > 1050)
  {
    read_receiver();
    delay(4);
  }

  timer = micros();
}

void loop()
{
  // the barometer has delay issues
  // read_barometer();
  read_imu();
  RateRoll -= CalibrateR;
  RatePitch -= CalibrateP;
  RateYaw -= CalibrateY;

  read_receiver();

  kalman_filter(Kr, Kr_uncertain, RateRoll, AngleRoll);
  Kr=K_result[0]; 
  Kr_uncertain=K_result[1];
  kalman_filter(Ka, Ka_uncertain, RatePitch, AnglePitch);
  Ka=K_result[0]; 
  Ka_uncertain=K_result[1];
  Serial.print("Roll Angle [°] ");
  Serial.print(Kr);
  Serial.print(" Pitch Angle [°] ");
  Serial.println(Ka);

  DesiredRoll = 0.15 * (ReceiverValue[0] - 1500);
  DesiredPitch = 0.15 * (ReceiverValue[1] - 1500);
  Throttle = ReceiverValue[2];
  DesiredYaw = 0.15 * (ReceiverValue[3] - 1500);

  ErrorRoll = DesiredRoll - RateRoll;
  ErrorPitch = DesiredPitch - RatePitch;
  ErrorYaw = DesiredYaw - RateYaw;

  pid_controller(ErrorRoll, PRoll, IRoll, DRoll, PrevErrorRoll, PrevIRoll);
  Roll = PIDReturn[0];
  PrevErrorRoll = PIDReturn[1];
  PrevIRoll = PIDReturn[2];
  pid_controller(ErrorPitch, PPitch, IPitch, DPitch, PrevIPitch, PrevIPitch);
  Pitch = PIDReturn[0];
  PrevErrorPitch = PIDReturn[1];
  PrevIPitch = PIDReturn[2];
  pid_controller(ErrorYaw, PYaw, IYaw, DYaw, PrevErrorYaw, PrevIYaw);
  Yaw = PIDReturn[0];
  PrevErrorYaw = PIDReturn[1];
  PrevIYaw = PIDReturn[2];

  if (Throttle > 1400){
    Throttle = 1400;
  }

  motor1 = 1.024 * (Throttle - Roll - Pitch - Yaw);
  motor2 = 1.024 * (Throttle - Roll + Pitch + Yaw);
  motor3 = 1.024 * (Throttle + Roll + Pitch - Yaw);
  motor4 = 1.024 * (Throttle + Roll - Pitch + Yaw);
  
  if (motor1 > 2000)
    motor1 = 1999;

  if (motor2 > 2000)
    motor2 = 1999;

  if (motor3 > 2000)
    motor3 = 1999;

  if (motor4 > 2000)
    motor4 = 1999;

  int IdleMode = 1180;

  if (motor1 < IdleMode)
    motor1 = IdleMode;

  if (motor2 < IdleMode)
    motor2 = IdleMode;

  if (motor3 < IdleMode)
    motor3 = IdleMode;

  if (motor4 < IdleMode)
    motor4 = IdleMode;

  int CutOff = 1000;

  if (ReceiverValue[2] < 1050)
  {
    motor1 = CutOff;
    motor2 = CutOff;
    motor3 = CutOff;
    motor4 = CutOff;
    reset_pid();
  }

  analogWrite(1, motor1);
  analogWrite(2, motor2);
  analogWrite(3, motor3);
  analogWrite(4, motor4);

  while(micros() - timer < 4000);
  timer = micros();
}
