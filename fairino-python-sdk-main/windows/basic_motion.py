from fairino import Robot
import time
# Establish a connection with the robot controller and return a robot object if the connection is successful
robot = Robot.RPC('192.168.58.2')
#start = robot.GetSystemClock()[1]
robot.RobotEnable(1)
robot.ServoMoveStart()
time.sleep(1)
robot.Mode(0)

j1 = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
j2 = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]
j3 = [-29.777, -84.536, 109.275, -114.075, -86.655, 74.257]
j4 = [-31.154, -95.317, 94.276, -88.079, -89.740, 74.256]
desc_pos1 = [-400, -130, 50, 178.118, 0.314, 3.833]
desc_pos2 = [-321.222, 185.189, 70, 179.030, -1.284, -29.869]
desc_pos3 = [-300, 100, 100, 176.600, 0.268, -14.061] #Keep roll, pitch, yaw of this
desc_pos4 = [-443.165, 147.881, 110, 179.511, -0.775, -15.409]
desc_pos5  = [-390.214, -150.332,  45.215, 178.520, -0.842, -10.314]
desc_pos6  = [-382.118,   95.884,  62.377, 179.002,  0.625, -18.774]
desc_pos7  = [-370.665,  175.292,  88.562, 177.661, -1.112,  25.447]
desc_pos8  = [-360.431, -110.558,  79.134, 180.000,  0.441,  12.890]
desc_pos9  = [-349.832,   55.477, 105.886, 176.231, -0.228, -30.511]
desc_pos10 = [-338.214,  200.155,  70.265, 178.941, -1.328, -5.876]
desc_pos11 = [-327.521,  -80.493,  52.187, 177.410,  0.974,  8.312]
desc_pos12 = [-315.937,  140.841,  96.755, 179.654, -0.714, -22.445]
desc_pos13 = [-295.613,   10.268,  37.991, 178.212,  0.581,  15.921]
desc_pos14 = [-265.842, -130.128, 108.449, 176.903, -0.985, -17.334]
offset_pos = [0, 0, 0, 0, 0, 0]
epos = [0, 0, 0, 0]
tool = 0
user = 0
vel = 100.0
acc = 100.0
ovl = 100.0
blendT = 1.0
blendR = 0.0
flag = 0
search = 0

robot.SetSpeed(20)
#rtn = robot.MoveJ(joint_pos=j1, tool=tool, user=user, vel=vel, blendT=blendT)
#print(f"movej errcode: {rtn}")
#rtn = robot.MoveL(desc_pos=desc_pos2, tool=tool, user=user, vel=vel, blendR=blendR)
#print(f"movel errcode: {rtn}")
'''rtn = robot.MoveC(desc_pos_p=desc_pos3, tool_p=tool, user_p=user, desc_pos_t=desc_pos4, tool_t=tool, user_t=user, blendR=blendR)
print(f"movec errcode: {rtn}")
rtn = robot.MoveJ(joint_pos=j2, tool=tool, user=user, vel=vel, blendT=blendT)
print(f"movej errcode: {rtn}")
rtn = robot.Circle(desc_pos_p=desc_pos3, tool_p=tool, user_p=user, desc_pos_t=desc_pos1, tool_t=tool, user_t=user)
print(f"circle errcode: {rtn}")'''
#end = robot.GetSystemClock()[1]
rtn = robot.MoveCart(desc_pos=desc_pos1, tool=tool, user=user, blendT=blendT)

rtn = robot.MoveCart(desc_pos=desc_pos2, tool=tool, user=user, blendT=blendT)
rtn = robot.MoveCart(desc_pos=desc_pos3, tool=tool, user=user, blendT=blendT)
rtn = robot.MoveCart(desc_pos=desc_pos4, tool=tool, user=user, blendT=blendT)
rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
rtn = robot.MoveCart(desc_pos=desc_pos6, tool=tool, user=user, blendT=blendT)
rtn = robot.MoveCart(desc_pos=desc_pos7, tool=tool, user=user, blendT=blendT)
rtn = robot.MoveCart(desc_pos=desc_pos8, tool=tool, user=user, blendT=blendT)
rtn = robot.MoveCart(desc_pos=desc_pos9, tool=tool, user=user, blendT=blendT)
rtn = robot.MoveCart(desc_pos=desc_pos10, tool=tool, user=user, blendT=blendT)
rtn = robot.MoveCart(desc_pos=desc_pos11, tool=tool, user=user, blendT=blendT)
rtn = robot.MoveCart(desc_pos=desc_pos12, tool=tool, user=user, blendT=blendT)
rtn = robot.MoveCart(desc_pos=desc_pos13, tool=tool, user=user, blendT=blendT)
rtn = robot.MoveCart(desc_pos=desc_pos14, tool=tool, user=user, blendT=blendT)

print(f"MoveCart errcode: {rtn}")
#elapsed_ms = end - start
#print(f"Elapsed time: {elapsed_ms / 1000:.3f} seconds")
robot.ServoMoveEnd()
robot.RobotEnable(0)
robot.CloseRPC()