#!/usr/bin/env python3

import rospy
import numpy as np

class StandController(object):
    def __init__(self, default_stance):
        self.def_stance = default_stance
        self.max_reach = 0.065

        self.FR_X = 0.
        self.FR_Y = 0.
        self.FL_X = 0.
        self.FL_Y = 0.

    def updateStateCommand(self,msg,state,command):
        state.body_local_position[0] = msg.linear.x * 0.15;
        self.FR_X = msg.linear.y
        self.FR_Y = msg.linear.z

        self.FL_X = msg.angular.x
        self.FL_Y = msg.angular.y

    @property
    def default_stance(self):
        a = np.copy(self.def_stance)
        return a

    def run(self,state,command):
        temp = self.default_stance
        temp[2] = [command.robot_height] * 4
        
        temp[1][0] += self.FR_Y * self.max_reach
        temp[0][0] += self.FR_X * self.max_reach

        temp[1][1] += self.FL_Y * self.max_reach
        temp[0][1] += self.FL_X * self.max_reach
            
        state.foot_locations = temp
        return state.foot_locations
