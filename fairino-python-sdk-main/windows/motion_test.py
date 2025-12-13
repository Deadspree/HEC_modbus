from fairino import Robot
import time

robot = Robot.RPC('192.168.58.2')
robot.RobotEnable(1)
robot.ServoMoveStart()
time.sleep(1)
robot.Mode(0)
robot.SetSpeed(20)

tool = 0
user = 0
vel = 40.0
blendT = 0.0

# Go to safe position
j1 = [0, -90, 90, 0, 90, 0]
rtn = robot.MoveJ(joint_pos=j1, tool=tool, user=user, vel=vel, blendT=blendT)
print("MoveJ return:", rtn)

print(robot.GetActualTCPPose())
#desc_pos = [-350.0, -70.0, 300.0, -180, 0.50, 4]
#print(type(desc_pos), desc_pos)
#rtn = robot.MoveL(desc_pos=desc_pos, tool=0, user=0, vel = 40.0, blendR = 0.0)
#print("MoveL return:", rtn)
robot.ServoMoveEnd()

robot.CloseRPC()