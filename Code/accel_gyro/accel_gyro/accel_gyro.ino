// Basic demo for accelerometer & gyro readings from Adafruit
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
  sox.getEvent(&gyro, &accel, &temp);

  RateRoll = gyro.gyro.x;
  RatePitch = gyro.gyro.y;
  RateYaw = gyro.gyro.z;

  // Serial.print("\t\tTemperature ");
  // Serial.print(temp.temperature);
  // Serial.println(" deg C");

  /* Display the results (acceleration is measured in m/s^2) */
  // Serial.print("\t\tAccel X: ");
  // Serial.print(accel.acceleration.x);
  // Serial.print(" \tY: ");
  // Serial.print(accel.acceleration.y);
  // Serial.print(" \tZ: ");
  // Serial.print(accel.acceleration.z);
  // Serial.println(" m/s^2 ");

  /* Display the results (rotation is measured in rad/s) */
  // Serial.print("\t\tGyro X: ");
  // Serial.print(gyro.gyro.x);
  // Serial.print(" \tY: ");
  // Serial.print(gyro.gyro.y);
  // Serial.print(" \tZ: ");
  // Serial.print(gyro.gyro.z);
  // Serial.println(" radians/s ");
  // Serial.println();

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

    // sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    // Serial.print("Accelerometer range set to: ");
    // switch (sox.getAccelRange()) {
    // case LSM6DS_ACCEL_RANGE_2_G:
    //   Serial.println("+-2G");
    //   break;
    // case LSM6DS_ACCEL_RANGE_4_G:
    //   Serial.println("+-4G");
    //   break;
    // case LSM6DS_ACCEL_RANGE_8_G:
    //   Serial.println("+-8G");
    //   break;
    // case LSM6DS_ACCEL_RANGE_16_G:
    //   Serial.println("+-16G");
    //   break;
    // }

    // sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
    // Serial.print("Gyro range set to: ");
    // switch (sox.getGyroRange()) {
    // case LSM6DS_GYRO_RANGE_125_DPS:
    //   Serial.println("125 degrees/s");
    //   break;
    // case LSM6DS_GYRO_RANGE_250_DPS:
    //   Serial.println("250 degrees/s");
    //   break;
    // case LSM6DS_GYRO_RANGE_500_DPS:
    //   Serial.println("500 degrees/s");
    //   break;
    // case LSM6DS_GYRO_RANGE_1000_DPS:
    //   Serial.println("1000 degrees/s");
    //   break;
    // case LSM6DS_GYRO_RANGE_2000_DPS:
    //   Serial.println("2000 degrees/s");
    //   break;
    // case ISM330DHCX_GYRO_RANGE_4000_DPS:
    //   break; // unsupported range for the DSOX
    // }

    // sox.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
    // Serial.print("Accelerometer data rate set to: ");
    // switch (sox.getAccelDataRate()) {
    // case LSM6DS_RATE_SHUTDOWN:
    //   Serial.println("0 Hz");
    //   break;
    // case LSM6DS_RATE_12_5_HZ:
    //   Serial.println("12.5 Hz");
    //   break;
    // case LSM6DS_RATE_26_HZ:
    //   Serial.println("26 Hz");
    //   break;
    // case LSM6DS_RATE_52_HZ:
    //   Serial.println("52 Hz");
    //   break;
    // case LSM6DS_RATE_104_HZ:
    //   Serial.println("104 Hz");
    //   break;
    // case LSM6DS_RATE_208_HZ:
    //   Serial.println("208 Hz");
    //   break;
    // case LSM6DS_RATE_416_HZ:
    //   Serial.println("416 Hz");
    //   break;
    // case LSM6DS_RATE_833_HZ:
    //   Serial.println("833 Hz");
    //   break;
    // case LSM6DS_RATE_1_66K_HZ:
    //   Serial.println("1.66 KHz");
    //   break;
    // case LSM6DS_RATE_3_33K_HZ:
    //   Serial.println("3.33 KHz");
    //   break;
    // case LSM6DS_RATE_6_66K_HZ:
    //   Serial.println("6.66 KHz");
    //   break;
    // }

    // sox.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
    // Serial.print("Gyro data rate set to: ");
    // switch (sox.getGyroDataRate()) {
    // case LSM6DS_RATE_SHUTDOWN:
    //   Serial.println("0 Hz");
    //   break;
    // case LSM6DS_RATE_12_5_HZ:
    //   Serial.println("12.5 Hz");
    //   break;
    // case LSM6DS_RATE_26_HZ:
    //   Serial.println("26 Hz");
    //   break;
    // case LSM6DS_RATE_52_HZ:
    //   Serial.println("52 Hz");
    //   break;
    // case LSM6DS_RATE_104_HZ:
    //   Serial.println("104 Hz");
    //   break;
    // case LSM6DS_RATE_208_HZ:
    //   Serial.println("208 Hz");
    //   break;
    // case LSM6DS_RATE_416_HZ:
    //   Serial.println("416 Hz");
    //   break;
    // case LSM6DS_RATE_833_HZ:
    //   Serial.println("833 Hz");
    //   break;
    // case LSM6DS_RATE_1_66K_HZ:
    //   Serial.println("1.66 KHz");
    //   break;
    // case LSM6DS_RATE_3_33K_HZ:
    //   Serial.println("3.33 KHz");
    //   break;
    // case LSM6DS_RATE_6_66K_HZ:
    //   Serial.println("6.66 KHz");
    //   break;
  
}

void loop()
{
  read_IMU();
  
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;
  Serial.print("Roll rate [°/s]= ");
  Serial.print(RateRoll); 
  Serial.print(" Pitch Rate [°/s]= ");
  Serial.print(RatePitch);
  Serial.print(" Yaw Rate [°/s]= ");
  Serial.println(RateYaw);
  delay(50);
  //  // serial plotter friendly format

  //  Serial.print(temp.temperature);
  //  Serial.print(",");

  //  Serial.print(accel.acceleration.x);
  //  Serial.print(","); Serial.print(accel.acceleration.y);
  //  Serial.print(","); Serial.print(accel.acceleration.z);
  //  Serial.print(",");

  // Serial.print(gyro.gyro.x);
  // Serial.print(","); Serial.print(gyro.gyro.y);
  // Serial.print(","); Serial.print(gyro.gyro.z);
  // Serial.println();
  //  delayMicroseconds(10000);
}

// testing and getting it to work

// #include "LSM6DSOXSensor.h"

// // Declare LSM6DSOX sensor. Sensor address can have 2 values LSM6DSOX_I2C_ADD_L (corresponds to 0x6A I2C address) or LSM6DSOX_I2C_ADD_H (corresponds to 0x6B I2C address)
// // On Adafruit lsm6dsox sensor, LSM6DSOX_I2C_ADD_L is the default address
// LSM6DSOXSensor lsm6dsoxSensor = LSM6DSOXSensor(&Wire, LSM6DSOX_I2C_ADD_L);

// // float RateRoll, RatePitch, RateYaw;
// float RateRoll, RatePitch, RateYaw;
// float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
// int RateCalibrationNumber;

// void gyro_signals(void)
// {
//   // Read gyroscope
//   uint8_t gyroStatus;
//   lsm6dsoxSensor.Get_G_DRDY_Status(&gyroStatus);
//   if (gyroStatus == 1)
//   { // Status == 1 means a new data is available
//     int32_t rotation[3];
//     lsm6dsoxSensor.Get_G_Axes(rotation);
//     // Plot data for each axis in milli degrees per second

//     // RateRoll = (float)rotation[0] / 250;
//     // RatePitch = (float)rotation[1] / 250;
//     // RateYaw = (float)rotation[2] / 250;

//     RateRoll = (float)rotation[0];
//     RatePitch = (float)rotation[1];
//     RateYaw = (float)rotation[2];
//   }
// }

// void setup()
// {
//   Serial.begin(115200);
//   Wire.begin();

//   // Default clock is 100kHz. LSM6DSOX also supports 400kHz, let's use it
//   Wire.setClock(400000);

//   // Init the sensor
//   lsm6dsoxSensor.begin();

//   // Enable accelerometer and gyroscope, and check success
//   if (lsm6dsoxSensor.Enable_X() == LSM6DSOX_OK && lsm6dsoxSensor.Enable_G() == LSM6DSOX_OK)
//   {
//     Serial.println("Success enabling accelero and gyro");
//   }
//   else
//   {
//     Serial.println("Error enabling accelero and gyro");
//   }

//   // Read ID of device and check that it is correct
//   uint8_t id;
//   lsm6dsoxSensor.ReadID(&id);
//   if (id != LSM6DSOX_ID)
//   {
//     Serial.println("Wrong ID for LSM6DSOX sensor. Check that device is plugged");
//   }
//   else
//   {
//     Serial.println("Receviced correct ID for LSM6DSOX sensor");
//   }

//   // Set accelerometer scale at +- 2G. Available values are +- 2, 4, 8, 16 G
//   lsm6dsoxSensor.Set_X_FS(2);

//   // Set Accelerometer sample rate to 208 Hz. Available values are +- 12.0, 26.0, 52.0, 104.0, 208.0, 416.0, 833.0, 1667.0, 3333.0, 6667.0 Hz
//   lsm6dsoxSensor.Set_X_ODR(416.0f);

//   // Set gyroscope scale at +- 125 degres per second. Available values are +- 125, 250, 500, 1000, 2000 dps
//   lsm6dsoxSensor.Set_G_FS(125);

//   // Set Gyroscope sample rate to 208 Hz. Available values are +- 12.0, 26.0, 52.0, 104.0, 208.0, 416.0, 833.0, 1667.0, 3333.0, 6667.0 Hz
//   lsm6dsoxSensor.Set_G_ODR(416.0f);

//   for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++)
//   {
//     gyro_signals();
//     RateCalibrationRoll += RateRoll;
//     RateCalibrationPitch += RatePitch;
//     RateCalibrationYaw += RateYaw;
//     delay(1);
//   }
//   RateCalibrationRoll /= 2000;
//   RateCalibrationPitch /= 2000;
//   RateCalibrationYaw /= 2000;

//   delay(250);
// }

// void loop()
// {

//   // Read accelerometer
//   uint8_t acceleroStatus;
//   lsm6dsoxSensor.Get_X_DRDY_Status(&acceleroStatus);
//   if (acceleroStatus == 1)
//   { // Status == 1 means a new data is available
//     int32_t acceleration[3];
//     lsm6dsoxSensor.Get_X_Axes(acceleration);
//     // Plot data for each axis in mg
//     // Serial.print("AccelerationX="); Serial.print(acceleration[0]); Serial.print("mg, AccelerationY="); Serial.print(acceleration[1]); Serial.print("mg, AccelerationZ="); Serial.print(acceleration[2]); Serial.println("mg");
//   }

//   RateRoll -= RateCalibrationRoll;
//   RatePitch -= RateCalibrationPitch;
//   RateYaw -= RateCalibrationYaw;

//   Serial.print("RotationX= ");
//   Serial.print(RateRoll);
//   Serial.print("mdps, RotationY= ");
//   Serial.print(RatePitch);
//   Serial.print("mdps, RotationZ=");
//   Serial.print(RateYaw);
//   Serial.println("mdps");

//   delay(10);
// }
