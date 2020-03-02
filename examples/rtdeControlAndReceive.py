import rtde_receive
import rtde_control
from robot_control.robot import Robot

robot_ip = "192.168.100.4"
rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
rtde_c = rtde_control.RTDEControlInterface(robot_ip)

start_pose = rtde_r.getActualTCPPose()
print("Start pose is {}".format(start_pose))

print(type(start_pose))
pose = list(start_pose)


speed = 0.5
acceleration = 0.5


# Be aware this will move the robot TCP withing a 1 cm radius from its current position
for i in range(3):
    for j in range(3):
        pose[i]+=0.01
        rtde_c.moveL(pose, speed, acceleration)
        pose[i]-=0.01
        rtde_c.moveL(pose, speed, acceleration)
        



