import math
import piconzero as pz
 
 
# Initialise Piconzero HAT
pz.init()
# Leg component length constants
thigh_length = 92
shin_length = 75
# Servo home positions (legs point directly downwards and straight at these positions)
servo0_home = 50
servo1_home = 109
servo2_home = 52
servo3_home = 23
# Servo Pins for Piconzero HAT
servo0 = 0
servo1 = 1
servo2 = 2
servo3 = 3
# Configure Output to Servo Mode
pz.setOutputConfig(servo0, 2)
pz.setOutputConfig(servo1, 2)
pz.setOutputConfig(servo2, 2)
pz.setOutputConfig(servo3, 2)
 
def home_servos():
    pz.setOutput(servo0, servo0_home)
    pz.setOutput(servo1, servo1_home)
    pz.setOutput(servo2, servo2_home)
    pz.setOutput(servo3, servo3_home)
    
 
class IKSolve:  # IKSolve - Inverse Kinematics solver for Raspibotics' Balancing Robot
 
    def __init__(self, upper_leg=92, lower_leg=75):  # 92mm and 75mm are default leg segment lengths
        self.a_const_0 = (upper_leg**2) - (lower_leg**2)  # b^2 - a^2
        self.a_const_1 = 2*upper_leg  # 2*b (c is unknown)
 
        self.b_const_0 = (upper_leg**2) + (lower_leg**2)  # b^2 + c^2
        self.b_const_1 = 2*upper_leg*lower_leg  # 2bc
 
    def translate_xy(self, x, y):  # translate_xy(x, y) - Calculates the required angles of two joints to move to (x, y)
        # A = Cos^-1((b^2+c^2-a^2)/2bc)
        angle_a = math.degrees(math.acos(((self.a_const_0 + (x ** 2)) /
                                          (self.a_const_1 * x))))
        angle_a += math.degrees(math.atan2(y, x))  # Tan^-1(y/x)
        angle_b = math.degrees(math.acos((self.b_const_0 - (x ** 2 + y ** 2)) /  # A = Cos^-1((b^2+c^2-a^2)/2bc)
                                         self.b_const_1))
        angle_b = 180-angle_b
        return round(angle_a), round(angle_b)  # Return angle of joints required to move effector to (x, y)
 
 
IKSolve = IKSolve()
home_servos()  # Reset Servos to default position
 
 
while True:
    try:
        x = int(input("x:"))
        y = int(input("y:"))
        if x==0 and y==0:
            home_servos()
        else:
            angle = IKSolve.translate_xy(x, y)
            pz.setOutput(servo0, servo0_home+angle[0])
            pz.setOutput(servo1, servo1_home-angle[1])
            pz.setOutput(servo2, servo2_home-angle[0])
            pz.setOutput(servo3, servo3_home+angle[1])
    except KeyboardInterrupt:
        pz.cleanup()
        exit()
