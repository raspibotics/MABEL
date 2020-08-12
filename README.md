# MABEL (Multi Axis Balancer Electronically Levelled)
![MABEL](https://i.imgur.com/ciOArSG.jpeg "MABEL")
## Table of contents
 - [About MABEL](#About-MABEL)
   - [Features and design](#Features-and-design)
 - [Bill of Materials (BOM)](#Billofmaterials)
   - [3D Printable](#3D-Printable)
   - [Non 3D Printable](#Non-3D-Printable)
     - [Electronic components](#Electronic-components)
     - [Mechcanical components](#Mechanical-components)
 - [Build instructions](#Build-instructions)
     - [Mechanical assembly](#Mechanical-assembly)
     - [Electronics assembly](#Electronics-assembly)
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


## Contact/Support
Thanks for taking an interest in MABEL and its development! If you have any questions or need information whilst building MABEL, please feel free to mention me on twitter [@raspibotics](https://twitter.com/raspibotics) or send me an email at **raspibotics@gmail.com**. ***Whilst this page is still undergoing development, you can find updates and more information on my blog https://raspibotics.wixsite.com/pibotics-blog, where I have been posting about the development of MABEL.***
