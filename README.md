# DIY-Drone
This project is for ECE 1895 - Junior Design Fundamentals.

### Background
A quadacopter drone can move in the any direction and rotate on any axis. To start off the rotation: <br>
* X-axis is the pitch that would cause a up-down motion. 
* Y-axis is the roll that would cause a left to right motion.
* Z-axis is the yaw that would cause a tilting motion. <br>
![alt text](image.png) <center>Figure 1</center>

Depending on what you define as the axis the pitch, roll, and yaw may vary. In this project, x-axis is yellow, the red is y-axis, and z-axis is blue.

## Design Overview

### Introduction

This project aims to use Teensy 4.1, LSM6DSOX, MPL3115A2, and a GPS module to make a DIY flight controller. All of this is for developing a flight contoller that is more modular and flexible than pre-built ones for the Robotics and Automation Society (RAS) Aerial Project. The goal is to make one that can be used on a bigger platform.

Initially the design of this flight controller would have a reciever, gyroscope, accelerometer, barometer, GPS, and FPV camera attached to it so it can be used for flight and the pilot can see where the drone is pointing. Based on the sensors LSM6DSOX (gyroscope, accelerometer, temperature sensor) and the MPL3115A2 (barometer, temperature sensor), I can tell what direction and orientation the drone is at and how high the drone is. Also, being able to control the drone manually.

In the end, I was able to get the receiver, gyroscope, accelerometer, and barometer to work properly. In terms of getting the data from the sensors and able to calulate the date into something I can use.

Right after bop-it has finished, I started with getting some sort of data from the receiver and within the week I was able to get it to work properly and tested it. Then I waited for a week because I submitted the BOM a little late. Once I recieved all of my sensors and components I started out by testing the LSM6DSOX sensor and the MPL3115A2 sensor individually to see if the sensors are working properly. Both the LSM6DSOX and MPL3115A2 worked fine, then I implemented sample collection to calibration the sensors. Since both the LSM6DSOX and MPL3115A2 are the vital parts to a flight controller. With other classes assignments and exams, I was able to get a digital filter and some what of a PID controller working.

## Preliminary Design Vefificaiton 
I breadboarded all of the circuit for the flight controller.
![alt text](<Images and Video/IMG_8458.jpg>)<center>Figure 2</center>
On the breadboard is the from top to the bottom are the receiver, MPL3115A2, LSM6DSOX and Teensy 4.1. 

The test plan was to plug it into the Teensy and check if the values read in and check those values in the serial monitor from the Arduino IDE. 


## Design Implementation
The Final design of the drone project was all breadboarded because of the time constraints and getting everything to work together.
![alt text](<Images and Video\IMG_8442.jpg>)<center>Figure 3</center>

![alt text](<Images and Video\IMG_8441.jpg>)<center>Figure 4</center>

As shown in Fig. 3 the breadboad (flight controller) is zip tied to the cassy of the drone. All the electronic speed controllers (ESC) are connected to the pins 1-4, which are PWM pins. 

## Design Testing


## Summary, Conclusions and Future Work