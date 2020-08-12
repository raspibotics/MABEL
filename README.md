# MABEL (Multi Axis Balancer Electronically Levelled)
## Table of contents
 - [About MABEL](#About-MABEL)
   - [Features and design](#Features-and-design)
 - [Bill of Materials](Bill-of-materials)
   - [3D Printable](3D-Printable)
   - [Non 3D Printable](Non-3D-Printable)

## About MABEL
MABEL is an open source self balancing robot that is inspired by the famous [Boston Dynamics Handle robot](http://https://www.youtube.com/watch?v=-7xvqQeoA8c "Boston Dynamics Handle robot"). The robot is controlled via an Arduino that handles all of the PID calculations (based off of open source[ YABR](http://http://www.brokking.net/yabr_main.html " YABR") firmware) based on the angle received from an MPU-6050 Accelerometer/Gyro, whilst the pi manages Bluetooth and servo control, running an inverse kinematics algorithm to translate the robot legs perfectly in two axes.

> The goal of MABEL is to create an affordable legged balancing robot platform like the the Boston Dynamics Handle robot that can built on a hobby scale using cheap Amazon parts and components.

By having a balancing platform with articulated legs MABEL will be able to actively balance in multiple Axes and vary leg length depending on the surroundings to increase terrain and off-road performance.

MABEL has built on the open source [YABR](http://http://www.brokking.net/yabr_main.html "YABR") project for the PID controller but with the addition of servos and a pi that helps interface them and control everything.
### Features and design 

## Bill of materials
### 3D Printable
3D Printable files are found in /CAD/
### Non 3D Printable
