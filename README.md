# MABEL (Multi Axis Balancer Electronically Levelled)
![MABEL](https://i.imgur.com/ciOArSG.jpeg "MABEL")
## Table of contents
 - [About MABEL](#About-MABEL)
   - [Features and design](#features-and-design)
 - [Bill of Materials (BOM)](#bill-of-materials)
   - [3D Printable](#3d-printable)
   - [Non 3D Printable](#non-3d-printable)
     - [Electronic components](#electronic-components)
     - [Mechcanical components](#mechanical-components)
 - [Build instructions](#build-instructions)
     - [Mechanical assembly](#mechanical-assembly)
     - [Electronics assembly](#electronics-assembly)
 - [Installation](#installation)
 - [Usage](#usage)
     - [Inverse kinematics - Moving the legs](#iksolve---inverse-kinematics-for-mabel)
 - [Contact/Support](#contactsupport)

## About MABEL
MABEL is an open source self balancing robot that is inspired by the famous [Boston Dynamics Handle robot](http://https://www.youtube.com/watch?v=-7xvqQeoA8c "Boston Dynamics Handle robot"). The robot is controlled via an Arduino that handles all of the PID calculations (based off of open source [YABR](http://http://www.brokking.net/yabr_main.html " YABR") firmware) based on the angle received from an MPU-6050 Accelerometer/Gyro, whilst the pi manages Bluetooth and servo control, running an inverse kinematics algorithm to translate the robot legs perfectly in two axes.

> The goal of MABEL is to create an affordable legged balancing robot platform like the the Boston Dynamics Handle robot that can built on a hobby scale using cheap Amazon parts and components.

By having a balancing platform with articulated legs MABEL will be able to actively balance in multiple Axes and vary leg length depending on the surroundings to increase terrain and off-road performance.

MABEL has built on the open source [YABR](http://http://www.brokking.net/yabr_main.html "YABR") project for the PID controller but with the addition of servos and a pi that helps interface them and control everything.
### Features and design 
Some of the stand-out features that make MABEL different from other balancing robots are:
- **Movable Legs** (**Enhanced mobility**, **terrain** and **stabilisation** capabilities)
- **Inverse Kinematics** for each legs that enables accurate translation in (x, y) coordinates using the [IKSolve.py](https://github.com/raspibotics/MABEL/blob/master/raspi_code/IKSolve2.py) class
- **Raspberry Pi** enabled (for **Bluetooth control**, **wireless connectivity** and **Computer Vision** capabilities)
- **Common/cheap build materials** (All of the materials can be purchased off of Amazon/Ebay for a low cost)
- **Stepper Motors** (Accurate positioning and precise control)

## Bill of materials
### 3D Printable
3D Printable files are found in [/CAD/3D Models (To Print)](https://github.com/raspibotics/MABEL/tree/master/CAD/3D%20Models%20(To%20print))
- [BatteryPack](https://github.com/raspibotics/MABEL/blob/master/CAD/3D%20Models%20(To%20print)/BatteryPack.stl) (**Optional** - holds LiPo battery onto back of robot with M5 Bolts)
- **2x** [BodyPanel](https://github.com/raspibotics/MABEL/blob/master/CAD/3D%20Models%20(To%20print)/BodyPanel.stl) (Mounting for the 'Hip' servos and side panels of the body)
- [BodyTop](https://github.com/raspibotics/MABEL/blob/master/CAD/3D%20Models%20(To%20print)/BodyTop.stl)
- **4x** [DriverGear](https://github.com/raspibotics/MABEL/blob/master/CAD/3D%20Models%20(To%20print)/DriverGear.stl)
- [Housing](https://github.com/raspibotics/MABEL/blob/master/CAD/3D%20Models%20(To%20print)/Housing.stl) (**LOWER** body housing for MABEL)
- **2x** [LowerLeg](https://github.com/raspibotics/MABEL/blob/master/CAD/3D%20Models%20(To%20print)/LowerLeg.stl)
- **2x** [UpperLeg](https://github.com/raspibotics/MABEL/blob/master/CAD/3D%20Models%20(To%20print)/UpperLeg.stl)
- [PanBracketHead](https://github.com/raspibotics/MABEL/blob/master/CAD/3D%20Models%20(To%20print)/PanBracketHead.stl) (**Optional** Pan servo bracket for head assembly)
- [TiltBracketHead](https://github.com/raspibotics/MABEL/blob/master/CAD/3D%20Models%20(To%20print)/TiltBracketHead.stl) (**Optional** Tilt servo bracket for head assembly)
- [UpperBody](https://github.com/raspibotics/MABEL/blob/master/CAD/3D%20Models%20(To%20print)/UpperBody.stl) (**UPPER** body housing for MABEL)
- **2x** [Wheel](https://github.com/raspibotics/MABEL/blob/master/CAD/3D%20Models%20(To%20print)/Wheel.stl) (**Optional** You can 3D Print your own set of wheels, or buy wheels)

### Non 3D Printable
Here are the Non 3D printable materials to build MABEL that **must be either purchased or sourced**. **This includes all of the electronics, mechanical hardware and fixings.** It is **recommended to overbuy the nuts and bolts fixings, as the exact number can change between builds.** This [amazon list](https://www.amazon.co.uk/gp/registry/wishlist/1K7SOU8MRG2K7/ref=cm_wl_huc_view) contains a rough idea of what needs to be purchased.

### Electronic components
- **Raspberry Pi** Zero W
- **PCA9865** Servo Controller
- **Variable voltage regulator** (**Optionally** 2x regulator to supply servos with a higher voltage than the 5V required for the  pi)
- **Arduino** Uno 
- **6x** MG996R metal gear servos
- **2x** 38mm NEMA17 stepper motors
- **2x** A4988 (or **DRV8825**) stepper motor drivers 
- Arduino CNC Shield
- MPU-6050 gyro/accelerometer
- 11.1V 2800mAh 3S LiPo (**LiPo battery charger is required**)

### Mechanical components
- **Optional** Grippy rubber material for tyre tread if you are using the [Wheel](https://github.com/raspibotics/MABEL/blob/master/CAD/3D%20Models%20(To%20print)/Wheel.stl) provided.
- **6x** Aluminium servo horns (for **MG996R** servos)
- **8x** 	10mm diameter bearings (5mm internal diameter) x 4mm depth 
- **12x** 10mm M3 bolts
- **12** 15mm M5 bolts
- **4x** 30mm M5 bolts
- **16x** 15mm M4 bolts
- **20x** M5 locknuts and washers

## Build instructions
***This section is yet to be completed...*** 
### Mechanical assembly
***This section will contain instructions about the construction of the robot chassis and  frame***
### Electronics Assembly
***This section will contain instructions about the construction of the robot electronics***

## Installation
***This section will contain instructions about how to download and setup MABELs code and required libraries/dependencies***

## Usage 
### IKSolve - (Inverse Kinematics for MABEL)
**IKSolve** is the class that handles the **inverse kinematics** functionality for MABEL [(IKSolve.py)](https://github.com/raspibotics/MABEL/blob/master/raspi_code/IKSolve.py) and allows for the legs to be translated using **(x, y)** coordinates. It's really simple to use, all that you need to specify are the **home values of each servo** (these are the angles that when passed over to your servos, make the legs point **directly and straight downwards** at **90 degrees**). 

 
 *The code below is taken directly from [IKDemo.py](https://github.com/raspibotics/MABEL/blob/master/raspi_code/IKDemo.py)*

```python
from IKSolve import IKSolve

# Servo Home Values (degrees)
ru_home = 50   # Right leg upper joint home position (servo_angle[0])
rl_home = 109   # Right leg lower joint home position (servo_angle[1])
lu_home = 52   # Left leg upper joint home position (servo_angle[2])
ll_home = 23   # Left leg lower joint home position (servo_angle[3])
 
IKSolve = IKSolve(ru_home, rl_home, lu_home, ll_home)  # Pass home positions to IK class
```
***The class can also be initialised with different leg length segments but these are best left to default unless you have changed the length of any of the leg components.***
```python
def __init__(self, ru_home, rl_home, lu_home,  
                 ll_home, upper_leg=92, lower_leg=75):
```
To recieve suitable angles for each servo to move to (x, y) you must use `translate_xy(self, x, y, flip=False)` (Flip inverts the direction that the legs bend, by default this should be set **False**)

- **x translates the robot leg vertically and y translates the leg horizontally**, each value should be in integer mm
  
 - **x is the distance between the upper leg pivot and wheel centre** (Vertically) - acceptable range for MABEL (160-98mm)
  
 - **y is the distance between the upper leg pivot and wheel centre** (Horizontally) - acceptable range for MABEL (-50, 50mm)
 
```python
  servo_angle = IKSolve.translate_xy(x, y, flip=False) 
  # The angles calculated are stored in a tuple E.g. servo_angles[0-3]
  # You can pass these values to your servo controller code
```


## Contact/Support
Thanks for taking an interest in MABEL and its development! If you have any questions or need information whilst building MABEL, please feel free to mention me on twitter [@raspibotics](https://twitter.com/raspibotics) or send me an email at **raspibotics@gmail.com**. ***Whilst this page is still undergoing development, you can find updates and more information on my blog https://raspibotics.wixsite.com/pibotics-blog, where I have been posting about the development of MABEL.***
