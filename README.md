# DIY-Drone
This project is for ECE 1895 - Junior Design Fundamentals.

This project aims to use Teensy 4.1, LSM6DSOX, MPL3115A2, and a GPS module to make a DIY flight controller. All of the motors, electronic speed controller (ESC), power board, chassis, and Lithium Polymer (LiPO) battery are sponsored by the Robotics and Automation Society. All connections are through a single bus using I2C communication at PINs 18 and 19. The LSM6DSOX is a breakout board that contains sensors, an accelerometer, and a gyroscope. The MPL3115A2 is a breakout board that contains a barometer. The PID controller will use all the sensor data to make the drone flight with the RC controller. 
