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
     - [Dependencies](#required-librariesdependencies)
 - [Usage](#usage)
     - [Inverse kinematics - Moving the legs](#iksolve---inverse-kinematics-for-mabel)
     - [Driving controls and serial communication](#driving-controls)
 - [Contact/Support](#contactsupport)

## About MABEL
MABEL is an open source self balancing robot that is inspired by the famous [Boston Dynamics Handle robot](https://www.youtube.com/watch?v=-7xvqQeoA8c "Boston Dynamics Handle robot"). The robot is controlled via an Arduino that handles all of the PID calculations (based off of open source [YABR](http://www.brokking.net/yabr_main.html " YABR") firmware) based on the angle received from an MPU-6050 Accelerometer/Gyro, whilst a Raspberry pi (code in python) manages Bluetooth and servo control, running an inverse kinematics algorithm to translate the robot legs perfectly in two axes.

> The goal of MABEL is to create an affordable legged balancing robot platform like the the Boston Dynamics Handle robot that can built on a hobby scale using cheap Amazon parts and components.

By having a balancing platform with articulated legs MABEL will be able to actively balance in multiple Axes and vary leg length depending on the surroundings to increase terrain and off-road performance.

MABEL has built on the open source [YABR](http://www.brokking.net/yabr_main.html "YABR") project for the PID controller but with the addition of servos and a pi that helps interface them and control everything.
### Features and design 
Some of the stand-out features that make MABEL different from other balancing robots are:
- **Movable Legs** (**Enhanced mobility**, **terrain** and **stabilisation** capabilities)
- **Inverse Kinematics** for each legs that enables accurate translation in (x, y) coordinates using the [IKSolve.py](https://github.com/raspibotics/MABEL/blob/master/raspi_code/IKSolve.py) class
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
## Mechanical assembly
---

<img src="https://i.imgur.com/iYMrObD.jpg" alt="push fit the bearings and secure with a nut and bolt" title="Push fit bearings" width="240" height="210" ALIGN="right" HSPACE="65"/>

### **Step 1:** Press the bearings into the joints 

 Each leg section ([UpperLeg](https://github.com/raspibotics/MABEL/blob/master/CAD/3D%20Models%20(To%20print)/UpperLeg.stl)  and [LowerLeg](https://github.com/raspibotics/MABEL/blob/master/CAD/3D%20Models%20(To%20print)/LowerLeg.stl)) requires **two bearings (thats 8x bearings in total) to push fit on opposing sides** to ensure smooth rotation. The bearings can be quite tricky to fit so it's **advisable to either apply slow, even pressure with a bench vice**, or to **soften the plastic with a hairdryer** to make it easier to push in by hand. Once you've fitted the bearings, you need to **push an M5 (30mm) bolt through** the hole left and **secure with a locknut**.
 [*More reference images for Step 1*](https://github.com/raspibotics/MABEL/tree/master/docs/images/Mech_Step_1)

---

<img src="https://i.imgur.com/Y1E7Eal.jpg" alt="Attach servos with M4 bolt" title="Attach servos" width="270" height="200" ALIGN="right" HSPACE="65"/>

### **Step 2:** Attach **4x** Servos to the [UpperLeg](https://github.com/raspibotics/MABEL/blob/master/CAD/3D%20Models%20(To%20print)/UpperLeg.stl)(s) and [BodyPanel](https://github.com/raspibotics/MABEL/blob/master/CAD/3D%20Models%20(To%20print)/BodyPanel.stl)(s) using **M4 (15mm)** bolts 
**4x MG996R servos** must be secured to **2x BodyPanel and 2x UpperLeg** using **4x M4 (15mm) bolts (16x in total)**. The servos **must sit flat** against these parts with the **shaft facing towards the outside of the robot**. *Your **servos may have a small rib** that you can **easily sand or file off** to get the servo to sit in this orientation.* [*More reference images for Step 2*](https://github.com/raspibotics/MABEL/tree/master/docs/images/Mech_Step_2)


---

<img src="https://i.imgur.com/08lZZj5.jpg" alt="push fit servo horns and attach gear to servo" title="Push fit bearings" width="270" height="200" ALIGN="right" HSPACE="65"/>

### **Step 3:** Push fit and screw the **servo horns** into the [DriverGear](https://github.com/raspibotics/MABEL/blob/master/CAD/3D%20Models%20(To%20print)/DriverGear.stl)(s) and attach the assemblies to the servos 

Take your **4x DriverGear parts** and **4x Aluminium Servo horns** and push it into the recess on the bottom of the gear. Then **secure the servo horns to each of the gears using the screws** that (should) come with servo horns. Once the gear assembly has been completed, screw it on to the **servo shaft** using **one M3 (10mm) bolt per servo**. [*More reference images for Step 3*](https://github.com/raspibotics/MABEL/tree/master/docs/images/Mech_Step_3)

---




***UNDER CONSTRUCTION: This section will contain instructions about the construction of the robot chassis and  frame***

## Electronics Assembly
***This section will contain instructions about the construction of the robot electronics***

## Installation
### Required Libraries/Dependencies
The **MABEL** project relies on multiple open-source and independent libraries, whilst MABELs own classses and functions may run natively, certain libararies are required for full functionality.

- Adafruit ServoKit PCA9865 servo controller library - [Installation guide](https://learn.adafruit.com/adafruit-16-channel-servo-driver-with-raspberry-pi/overview)
- python3 Cwiid (**Optional** - If you want to use a Wiimote controller) - [Installation guide](https://automaticaddison.com/how-to-make-a-remote-controlled-robot-using-raspberry-pi/)
- [approxeng.input](https://approxeng.github.io/approxeng.input/) (**Optional** - If you want to use a different controller - **recommended**)
  
***The controller code is still under development, however the approxeng.input library is recommended to develop controller code with - Versions for both the approxeng.input libary and Cwiid will be released eventually.***

Aftere following the installation instructions for the [ServoKit](https://learn.adafruit.com/adafruit-16-channel-servo-driver-with-raspberry-pi/overview) and controller library of your choice, you should be able to clone to the Raspberry Pi with `git clone https://www.github.com/raspibotics/MABEL`, all controller code, and other scripts should be written in the [MABEL/raspi_code/](https://github.com/raspibotics/MABEL/tree/master/raspi_code) directory so that the code will be able to reference the code already included.



## Usage 
### IKSolve - (Inverse Kinematics for MABEL)
**IKSolve** is the class that handles the **inverse kinematics** functionality for MABEL [(IKSolve.py)](https://github.com/raspibotics/MABEL/blob/master/raspi_code/IKSolve.py) and allows for the legs to be translated using **(x, y)** coordinates. It's really simple to use, all that you need to specify are the **home values of each servo** (these are the angles that when passed over to your servos, make the legs point **directly and straight downwards** at **90 degrees**). Keep in mind that the values below are just my servo home positions, and **yours will be different**. 

 
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
### Driving controls
**MABEL** is designed to work by listening to commands on the Arduino (PID contoller) end that are sent to it by the raspberry pi over serial (using [pySerial](https://pyserial.readthedocs.io/en/latest/)). This allows easy control using bluetooth and web based solutions that are much more difficult to get working on an Arduino. The code snippets below show how you could modify/write your own controller code by sending specific bytes over serial to the arduino and [YABR](http://www.brokking.net/yabr_main.html) based firmware.

In order to use serial commands to drive MABEL around, you first need to find what serial port that the USB connection between the raspberry pi and arduino is called. In my case it was ```/dev/ttyACM0``` but yours could be different. To find what serial port your arduino is connected to, type ```ls /dev/tty*``` into the terminal **without the arduino connected**. Then **reconnect the arduino**, run ```ls /dev/tty*``` again, and make a note of which serial port appears to be listed that was not previously there - **this is your serial port.**

*Once you know what serial port the arduino is connected on, you should initialise the serial class with it:*
```python
import serial
from struct import pack
from time import sleep

ser = serial.Serial(port='/dev/ttyACM0', # ls /dev/tty* to find your port
		baudrate = 9600,
		parity = serial.PARITY_NONE,
		stopbits = serial.STOPBITS_ONE,
		bytesize = serial.EIGHTBITS,
		timeout = 1)
```
The YABR code on the arduino accepts bytes [0, 1, 2, 4, 8] as driving input commands, with **0b00000000 being 'No movement/ remain stationary'**:
```python
# sendByte code
# 00000000 - Do nothing
# 00000001 - Turn left
# 00000010 - Turn right
# 00000100 - Move forwards
# 00001000 - Move backwards
```
We can then send these bytes over the serial connection, and tell **MABEL** in which direction to move - Here's an example of how to write to the arduino serial port, with a bit of **controller pseudocode** for clarity:
```python
while True:
    sendByte |= 0b00000000 
    if controller.direction == 'LEFT': # Controller pseudocode 
        sendByte |= 0b00000001 # This could be any sendByte
    val = pack("B", sendByte)
    ser.write(val) # Send the value over to the arduino 
    sleep(0.04) # Sleep for this period before sending next command
```



## Contact/Support
Thanks for taking an interest in MABEL and its development! If you have any questions or need information whilst building MABEL, please feel free to mention me on twitter [@raspibotics](https://twitter.com/raspibotics) or send me an email at **raspibotics@gmail.com**. ***Whilst this page is still undergoing development, you can find updates and more information on my blog [https://raspibotics.wixsite.com/pibotics-blog](https://raspibotics.wixsite.com/pibotics-blog), where I have been posting about the development of MABEL.***
