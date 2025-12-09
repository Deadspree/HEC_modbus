from fairino import Robot
import time
# Establish a connection with the robot controller and return a robot object if the connection is successful
robot = Robot.RPC('192.168.58.2')
print(robot.GetActualTCPPose())
robot.CloseRPC()