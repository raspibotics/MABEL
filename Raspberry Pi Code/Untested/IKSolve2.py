# IKSolve.py is an inverse kinematics module for MABEL to move the wheel (effector) in 2D Cartesian coordinates.
# Contributors: (@raspibotics)

# ***USAGE GUIDANCE***
# x translates the robot leg vertically and y translates the leg horizontally, each value should be in integer mm
# x is the distance between the upper leg pivot and wheel centre (Vertically) - acceptable range for MABEL (160-98mm)
# y is the distance between the upper leg pivot and wheel centre (Horizontally) - acceptable range for MABEL (-50, 50mm)

import math


class IKSolve:  # IKSolve - Inverse Kinematics solver for MABEL

    def __init__(self, ru_home, rl_home, lu_home,  # IKSolve Constructor, Values are default leg section lengths in mm
                 ll_home, upper_leg=92, lower_leg=75):
        self.ru_home = ru_home
        self.rl_home = rl_home
        self.lu_home = lu_home
        self.ll_home = ll_home

        self.a_const_0 = (upper_leg ** 2) - (lower_leg ** 2)  # b^2 - a^2
        self.a_const_1 = 2 * upper_leg  # 2*b (c is unknown)

        self.b_const_0 = (upper_leg ** 2) + (lower_leg ** 2)  # b^2 + c^2
        self.b_const_1 = 2 * upper_leg * lower_leg  # 2bc

    def translate_xy(self, x, y, flip=False):  # translate_xy(x, y) -Calculates the required angles to move to (x, y) mm
        if x == 0 and y == 0:
            return self.ru_home, self.rl_home, self.lu_home, self.ll_home
        else:
            angle_a = math.degrees(math.acos(((self.a_const_0 + (x ** 2)) /  # A = Cos^-1((b^2+c^2-a^2)/2bc)
                                              (self.a_const_1 * x))))
            angle_a += math.degrees(math.atan2(y, x))  # Tan^-1(y/x)
            angle_b = math.degrees(math.acos((self.b_const_0 - (x ** 2 + y ** 2)) /  # A = Cos^-1((b^2+c^2-a^2)/2bc)
                                             self.b_const_1))
            angle_b = 180 - angle_b
        if flip is not False:
            return (self.ru_home - round(angle_a)), (self.rl_home + round(angle_b)), (
                    self.lu_home + round(angle_a)), (self.ll_home - round(angle_b))
        else:
            return (self.ru_home + round(angle_a)), (self.rl_home - round(angle_b)), (
                    self.lu_home - round(angle_a)), (self.ll_home + round(angle_b))
