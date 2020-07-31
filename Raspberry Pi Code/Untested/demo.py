from IKSolve2 import IKSolve

# Servo Home Values (degrees)
ru_home = 50   # Right leg upper joint home position (servo_angle[0])
rl_home = 109   # Right leg lower joint home position (servo_angle[1])
lu_home = 52   # Left leg upper joint home position (servo_angle[2])
ll_home = 23   # Left leg lower joint home position (servo_angle[3])

IKSolve = IKSolve(ru_home, rl_home, lu_home, ll_home)  # Pass home positions to IK class

servo_angle = IKSolve.translate_xy(120, -10)  # servo_angles[0] refers to the right leg upper servo etc

print(servo_angle)

