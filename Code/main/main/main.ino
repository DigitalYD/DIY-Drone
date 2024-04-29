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

// TX/RX
PulsePositionInput ReceiverInput(RISING);
float ReceiverValue[] = {0, 0, 0, 0, 0, 0, 0, 0};
int ChannelNumber = 0;

// drone motors
float motor1, motor2, motor3, motor4;

// pid values
float RatePitch, RateRoll, RateYaw;
float CalibrationPitch, CalibrationRoll, CalibrationYaw;
int RateCalibrationNumber;

uint32_t timer;

float DesiredPitch, DesiredRoll, DesiredYaw;
float ErrorPitch, ErrorRoll, ErrorYaw;
float Throttle, Pitch, Roll, Yaw;
float PrevErrorPitch, PrevErrorRoll, PrevErrorYaw;
float PrevITermPitch, PrevITermRoll, PrevITermYaw;
float PIDReturn[] = {0, 0, 0};

float PRateRoll = 0.6;
float PRatePitch = PRateRoll;
float PRateYaw = 2;
float IRateRoll = 3.5;
float IRatePitch = IRateRoll;
float IRateYaw = 12;
float DRateRoll = 0.03;
float DRatePitch = DRateRoll;
float DRateYaw = 0;

// imu
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

// Kalman Filters
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

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
}

void read_barometer(void)
{
  float altitude = baro.getAltitude();

}

void read_imu(void)
{
  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  // there is a error in the template where the gyro and accelerometer values are swapped
  sox.getEvent(&accel, &gyro, &temp);

  int16_t GyroX=gyro.gyro.x;
  int16_t GyroY=gyro.gyro.y;
  int16_t GyroZ=gyro.gyro.z;

  RateRoll=(float)GyroX;
  RatePitch=(float)GyroY;
  RateYaw=(float)GyroZ;

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

// setting the motor control TX -> Teensy -> Motor
void controller(void)
{
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

void pid_controller(float Error, float P, float I, float D, float PrevError, float PrevIterm)
{
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;
  if (Iterm > 400)
    Iterm = 400;
  else if (Iterm < -400)
    Iterm = -400;
  float Dterm = D * (Error - PrevError) / 0.004;
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400)
    PIDOutput = 400;
  else if (PIDOutput < -400)
    PIDOutput = -400;
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void reset_pid(void)
{
  PrevErrorRoll = 0;
  PrevErrorPitch = 0;
  PrevErrorYaw = 0;
  PrevITermRoll = 0;
  PrevITermPitch = 0;
  PrevITermYaw = 0;
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

  // send PWM to the motor

  // analogWriteFrequency(PIN, FREQ)
  // set the PIN to send out and the frequency of the signal
  CalibrationRoll /= 2000;
  CalibrationPitch /= 2000;
  CalibrationYaw /= 2000;
  analogWriteFrequency(1, 250);
  analogWriteFrequency(2, 250);
  analogWriteFrequency(3, 250);
  analogWriteFrequency(4, 250);

  // analogWriteResolution(bits)
  analogWriteResolution(12);

  // delay
  // delay(250);

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
  RateRoll -= CalibrationRoll;
  RatePitch -= CalibrationPitch;
  RateYaw -= CalibrationYaw;

  // read_receiver();
  // controller();

  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  Serial.print("Roll Angle [°] ");
  Serial.print(KalmanAngleRoll);
  Serial.print(" Pitch Angle [°] ");
  Serial.println(KalmanAnglePitch);

  // DesiredRoll = 0.15 * (ReceiverValue[0] - 1500);
  // DesiredPitch = 0.15 * (ReceiverValue[1] - 1500);
  // Throttle = ReceiverValue[2];
  // DesiredYaw = 0.15 * (ReceiverValue[3] - 1500);

  // ErrorRoll = DesiredRoll - RateRoll;
  // ErrorPitch = DesiredPitch - RatePitch;
  // ErrorYaw = DesiredYaw - RateYaw;

  // pid_controller(ErrorRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRoll, PrevITermRoll);
  // Roll = PIDReturn[0];
  // PrevErrorRoll = PIDReturn[1];
  // PrevITermRoll = PIDReturn[2];
  // pid_controller(ErrorPitch, PRatePitch, IRatePitch, DRatePitch, PrevITermPitch, PrevITermPitch);
  // Pitch = PIDReturn[0];
  // PrevErrorPitch = PIDReturn[1];
  // PrevITermPitch = PIDReturn[2];
  // pid_controller(ErrorYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorYaw, PrevITermYaw);
  // Yaw = PIDReturn[0];
  // PrevErrorYaw = PIDReturn[1];
  // PrevITermYaw = PIDReturn[2];

  // if (Throttle > 1400){
  //   Throttle = 1400;
  // }

  // motor1 = 1.024 * (Throttle - Roll - Pitch - Yaw);
  // motor2 = 1.024 * (Throttle - Roll + Pitch + Yaw);
  // motor3 = 1.024 * (Throttle + Roll + Pitch - Yaw);
  // motor4 = 1.024 * (Throttle + Roll - Pitch + Yaw);
  
  // if (motor1 > 2000)
  //   motor1 = 1999;

  // if (motor2 > 2000)
  //   motor2 = 1999;

  // if (motor3 > 2000)
  //   motor3 = 1999;

  // if (motor4 > 2000)
  //   motor4 = 1999;

  // int ThrottleIdle = 1180;

  // if (motor1 < ThrottleIdle)
  //   motor1 = ThrottleIdle;

  // if (motor2 < ThrottleIdle)
  //   motor2 = ThrottleIdle;

  // if (motor3 < ThrottleIdle)
  //   motor3 = ThrottleIdle;

  // if (motor4 < ThrottleIdle)
  //   motor4 = ThrottleIdle;

  // int ThrottleCutOff = 1000;

  // if (ReceiverValue[2] < 1050)
  // {
  //   motor1 = ThrottleCutOff;
  //   motor2 = ThrottleCutOff;
  //   motor3 = ThrottleCutOff;
  //   motor4 = ThrottleCutOff;
  //   reset_pid();
  // }

  // analogWrite(1, motor1);
  // analogWrite(2, motor2);
  // analogWrite(3, motor3);
  // analogWrite(4, motor4);

  while(micros() - timer < 4000);
  timer = micros();
}
