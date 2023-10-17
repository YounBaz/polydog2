#!/usr/bin/env python3
#Author: lnotspotl

import rospy

from sensor_msgs.msg import Joy,Imu
import RobotController
import robot_IK
from std_msgs.msg import Float64

USE_IMU = False
RATE = 60

rospy.init_node("Robot_Controller")

# Robot geometry
body = [0.286, 0.205]
legs = [0.0, 0.30, 0.1295, 0.125] 

notspot_robot = RobotController.Robot(body, legs, USE_IMU)
inverseKinematics = robot_IK.InverseKinematics(body, legs)

command_topics = ["/notspot_controller/FR1_joint/command",
                  "/notspot_controller/FR2_joint/command",
                  "/notspot_controller/FR3_joint/command",
                  "/notspot_controller/FL1_joint/command",
                  "/notspot_controller/FL2_joint/command",
                  "/notspot_controller/FL3_joint/command",
                  "/notspot_controller/RR1_joint/command",
                  "/notspot_controller/RR2_joint/command",
                  "/notspot_controller/RR3_joint/command",
                  "/notspot_controller/RL1_joint/command",
                  "/notspot_controller/RL2_joint/command",
                  "/notspot_controller/RL3_joint/command"]

publishers = []
for i in range(len(command_topics)):
    publishers.append(rospy.Publisher(command_topics[i], Float64, queue_size = 10))

if USE_IMU:
    rospy.Subscriber("notspot_imu/base_link_orientation",Imu,notspot_robot.imu_orientation)
rospy.Subscriber("notspot_joy/joy_ramped",Joy,notspot_robot.joystick_command)

rate = rospy.Rate(RATE)

del body
del legs
del command_topics
del USE_IMU
del RATE

while not rospy.is_shutdown():
    leg_positions = notspot_robot.run()
    notspot_robot.change_controller()

    dx = notspot_robot.state.body_local_position[0]
    dy = notspot_robot.state.body_local_position[1]
    dz = notspot_robot.state.body_local_position[2]
    
    roll = notspot_robot.state.body_local_orientation[0]
    pitch = notspot_robot.state.body_local_orientation[1]
    yaw = notspot_robot.state.body_local_orientation[2]

    try:
        joint_angles = inverseKinematics.inverse_kinematics(leg_positions,
                               dx, dy, dz, roll, pitch, yaw)

        for i in range(len(joint_angles)):
            if i == 2 or i == 3 or i == 4 or i == 5 or i == 8 or i == 9 or i == 10 or i == 11:
                publishers[i].publish(-joint_angles[i])
            if i == 0 or i == 1 or i == 6 or i == 7 :
                publishers[i].publish(-joint_angles[i])
    except:
        pass

    rate.sleep()
