
// LSM6DSOX sensor

#include <Adafruit_LSM6DSOX.h>

Adafruit_LSM6DSOX sox;

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

void read_IMU(void){
  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  // there is a error in the template where the gyro and accelerometer values are swapped
  sox.getEvent(&accel, &gyro, &temp);

  RateRoll = gyro.gyro.x;
  RatePitch = gyro.gyro.y;
  RateYaw = gyro.gyro.z;

  // Serial.print("\t\tTemperature ");
  // Serial.print(temp.temperature);
  // Serial.println(" deg C");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  /* Display the results (rotation is measured in rad/s) */
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();

  // delay(100);
}


void setup(void)
{
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit LSM6DSOX test!");

  if (!sox.begin_I2C())
  {
    while (1)
    {
      delay(10);
    }
  }

  Serial.println("LSM6DSOX Found!");

  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++)
  {
    read_IMU();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }

  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;   

   
}

void loop()
{
  read_IMU();
  
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;
  // Serial.print("Roll rate [°/s]= ");
  // Serial.print(RateRoll); 
  // Serial.print(" Pitch Rate [°/s]= ");
  // Serial.print(RatePitch);
  // Serial.print(" Yaw Rate [°/s]= ");
  // Serial.println(RateYaw);
  // delay(50);

  // Serial.print("Acceleration: ");
  // Serial.print(accel.acceleration.x);
  // Serial.print(","); Serial.print(accel.acceleration.y);
  // Serial.print(","); Serial.print(accel.acceleration.z);
  // Serial.print(",");

  // Serial.print("Gyroscope: ");
  // Serial.print(gyro.gyro.x);
  // Serial.print(","); Serial.print(gyro.gyro.y);
  // Serial.print(","); Serial.print(gyro.gyro.z);
  // Serial.println();

  delay(1000);
  //  delayMicroseconds(10000);
}

