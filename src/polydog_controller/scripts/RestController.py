#!/usr/bin/env python3

import rospy
import numpy as np
from Transformations import rotxyz
from PIDController import PID_controller

class RestController(object):
    def __init__(self, default_stance):
        self.def_stance = default_stance

        # TODO: tune kp, ki and kd
        #                                     kp     ki    kd
        self.pid_controller = PID_controller(0.75, 2.29, 0.0)
        self.use_imu = False
        self.use_button = True
        self.pid_controller.reset()
        
    def updateStateCommand(self, msg, state, command):
        # local body position
        state.body_local_position[0] = msg.linear.z * 0.04
        #state.body_local_position[1] = msg.axes[6] * 0.03
        #if msg.linear.z < 0: 
        #    state.body_local_position[2] = msg.linear.z * 0.09
        #if msg.linear.z > 0: 
        #    state.body_local_position[2] = msg.linear.z * 0.04
        #if msg.linear.z == 0: 
        #    state.body_local_position[2] = msg.linear.z 
        # local body orientation
        #state.body_local_orientation[0] = msg.angular.y * 0.4
        #state.body_local_orientation[1] = msg.angular.x * 0.5
        #state.body_local_orientation[2] = msg.angular.z * 0.08



    @property
    def default_stance(self):
        return self.def_stance

    def step(self, state, command):
        temp = self.default_stance
        temp[2] = [command.robot_height] * 4

        # roll and pitch compensation
        # if self.use_imu == True, the robot tries to keep its body horizontal
        # using a PID controller
        if self.use_imu:
            compensation = self.pid_controller.run(state.imu_roll, state.imu_pitch)
            roll_compensation = -compensation[0]
            pitch_compensation = -compensation[1]

            rot = rotxyz(roll_compensation,pitch_compensation,0)
            temp = np.matmul(rot,temp)

        return temp

    def run(self, state, command):
        state.foot_locations = self.step(state, command)
        return state.foot_locations
