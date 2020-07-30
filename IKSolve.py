# IKSolve.py is an inverse kinematics module for MABEL to move the wheel (effector) in 2D Cartesian coordinates.
# Contributors: (@raspibotics)

# ***USAGE GUIDANCE***
# x translates the robot leg vertically and y translates the leg horizontally, each value should be in integer mm
# x is the distance between the upper leg pivot and wheel centre (Vertically) - acceptable range for MABEL (160-98mm)
# y is the distance between the upper leg pivot and wheel centre (Horizontally) - acceptable range for MABEL (-50, 50mm)

import math
import piconzero as pz

# Initialise HAT
pz.init()
# Servo home positions (legs point directly downwards (90 degrees) and straight at these positions)
servo0_home = 50
servo1_home = 109
servo2_home = 52
servo3_home = 23
# Servo Pins for HAT
servo0 = 0  # Right (has battery cable hole) Leg Top Joint Servo
servo1 = 1  # Right (has battery cable hole) Leg Lower Joint Servo
servo2 = 2  # Left leg top joint servo
servo3 = 3  # Left leg lower joint servo
# Configure Output to Servo Mode
pz.setOutputConfig(servo0, 2)
pz.setOutputConfig(servo1, 2)
pz.setOutputConfig(servo2, 2)
pz.setOutputConfig(servo3, 2)


def home_servos():  # home_servos() resets all servos to home position
    pz.setOutput(servo0, servo0_home)
    pz.setOutput(servo1, servo1_home)
    pz.setOutput(servo2, servo2_home)
    pz.setOutput(servo3, servo3_home)


class IKSolve:  # IKSolve - Inverse Kinematics solver for MABEL
    def __init__(self, upper_leg=92, lower_leg=75):  # IKSolve Constructor, Values are default leg section lengths in mm
        self.a_const_0 = (upper_leg ** 2) - (lower_leg ** 2)  # b^2 - a^2
        self.a_const_1 = 2 * upper_leg  # 2*b (c is unknown)

        self.b_const_0 = (upper_leg ** 2) + (lower_leg ** 2)  # b^2 + c^2
        self.b_const_1 = 2 * upper_leg * lower_leg  # 2bc

    def translate_xy(self, x, y):  # translate_xy(x, y) - Calculates the required servo angles to move to (x, y) mm
        # A = Cos^-1((b^2+c^2-a^2)/2bc)
        angle_a = math.degrees(math.acos(((self.a_const_0 + (x ** 2)) /
                                          (self.a_const_1 * x))))
        angle_a += math.degrees(math.atan2(y, x))  # Tan^-1(y/x)
        angle_b = math.degrees(math.acos((self.b_const_0 - (x ** 2 + y ** 2)) /  # A = Cos^-1((b^2+c^2-a^2)/2bc)
                                         self.b_const_1))
        angle_b = 180 - angle_b
        return round(angle_a), round(angle_b)  # Return angle of joints required to move effector to (x, y)


IKSolve = IKSolve()  # Create an instance of the IKSolve class
home_servos()  # Reset Servos to default position

while True:
    try:
        x_val = int(input("x:"))  # Input x as mm value required between upper leg pivot and wheel centre (Vertically)
        y_val = int(input("y:"))  # Input y as mm value required between upper leg pivot and wheel centre (Horizontally)
        if x_val == 0 and y_val == 0:
            home_servos()
        else:
            offset_angle = IKSolve.translate_xy(x_val, y_val)  # Offset angles is calculated and returned as a tuple
            pz.setOutput(servo0, servo0_home + offset_angle[0])
            pz.setOutput(servo1, servo1_home - offset_angle[1])
            pz.setOutput(servo2, servo2_home - offset_angle[0])
            pz.setOutput(servo3, servo3_home + offset_angle[1])
    except KeyboardInterrupt:
        pz.cleanup()  # Cleanup servo outputs
        exit()
