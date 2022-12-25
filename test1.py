import math
from pulseapi import  RobotPulse, pose, position,jog
import time

host = "http://10.10.10.20:8081"  # replace with a valid robot address
robot1 = RobotPulse(host)  # create an instance of the API wrapper class
# create motion targets
home_pose = pose([0, -90, 0, -90, -90, 0])
start_pose = pose([0, -90, 90, -90, -90, 0])
pose_targets = [
    pose([10, -90, 90, -90, -90, 0]),
    pose([10, -90, 0, -90, -90, 0]),
]
position_target = position([-0.42, -0.12, 0.35], [math.pi, 0, 0])
position_targets = [
    position([-0.37, -0.12, 0.35], [math.pi, 0, 0]),
    position([-0.42, -0.12, 0.35], [math.pi, 0, 0]),
    position([-0.42, -0.17, 0.35], [math.pi, 0, 0]),
    position([-0.37, -0.17, 0.35], [math.pi, 0, 0]),
]

# set the desired speed (controls both motor velocity and acceleration)
SPEED = 20
# set the desired motor velocity
VELOCITY = 10
# set the desired motor acceleration
ACCELERATION = 20
# set the desired tcp velocity
TCP_VELOCITY_1CM = 0.01
TCP_VELOCITY_10CM = 0.1
robot1.freeze()
#robot1.set_pose(home_pose, speed=SPEED)

# checks every 0.1 s whether the motion is finished
#robot1.set_position(position_target, velocity=VELOCITY, acceleration=ACCELERATION)
#robot1.await_stop()


robot1.jogging(jog(x=-0.1, y=-0.1))
time.sleep(2)
robot1.freeze()
robot1.jogging(jog())