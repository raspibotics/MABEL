# IKSolve.py is an inverse kinematics module for MABEL to move the wheel (effector) in 2D Cartesian coordinates.
# Contributors: (@raspibotics)

# ***USAGE GUIDANCE***
# x translates the robot leg vertically and y translates the leg horizontally, each value should be in integer mm
# x is the distance between the upper leg pivot and wheel centre (Vertically) - acceptable range for MABEL (160-98mm)
# y is the distance between the upper leg pivot and wheel centre (Horizontally) - acceptable range for MABEL (-50, 50mm)

from math import acos, degrees, atan2


class IKSolve:  # IKSolve - Inverse Kinematics solver for MABEL

    def __init__(self, ru_home, rl_home, lu_home,  # IKSolve Constructor, Values are default leg section lengths in mm
                 ll_home, upper_leg=92, lower_leg=75):

        self.ru_home, self.rl_home = ru_home, rl_home  # Right leg servo home positions
        self.lu_home, self.ll_home = lu_home, ll_home  # Left leg servo home positions

        self.a_const_0 = (upper_leg ** 2) - (lower_leg ** 2)  # b^2 - a^2
        self.a_const_1 = 2 * upper_leg  # 2*b (c is unknown)

        self.b_const_0 = (upper_leg ** 2) + (lower_leg ** 2)   # b^2 + c^2
        self.b_const_1 = 2 * upper_leg * lower_leg  # 2bc

    def translate_xy(self, x, y, flip=False):  # translate_xy(x, y, flip) Calculates the required angles to move...
        # ...(x, y) mm - flip is used to invert the direction of the legs
        if x == 0 and y == 0:  # (0, 0) resets servos to home position
            return self.ru_home, self.rl_home, self.lu_home, self.ll_home
        else:
            try:
                angle_a = degrees(acos(((self.a_const_0 + (x ** 2)) /  # A = Cos^-1((b^2+c^2-a^2)/2bc)
                                        (self.a_const_1 * x))))
                angle_a += degrees(atan2(y, x))  # Tan^-1(y/x)
                angle_b = 180 - degrees(acos((self.b_const_0 - (x ** 2 + y ** 2)) /  # A = Cos^-1((b^2+c^2-a^2)/2bc)
                                             self.b_const_1))
            except ValueError:
                print('Value specified is outside range of capable movement. Please specify a different value...')
                return
        if flip is not False:
            return (self.ru_home - round(angle_a)), (self.rl_home + round(angle_b)), (
                    self.lu_home + round(angle_a)), (self.ll_home - round(angle_b))
        else:
            return (self.ru_home + round(angle_a)), (self.rl_home - round(angle_b)), (
                    self.lu_home - round(angle_a)), (self.ll_home + round(angle_b))
