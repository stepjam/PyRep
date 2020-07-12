#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul  9 03:15:00 2020

@author: nanbaima
"""
from pyrep.robots.legs.leg import Leg

class LeggedBase(Leg):
    """Base class representing a legged mobile robot with multi leg support.
    """

    def __init__(self,
                 count: int,
                 num_legs: int,
                 root_name: str,
                 num_joints: int,
                 base_name: str = None):
        """Init.

        :param count: used for multiple copies of the same robot.
        :param num_legs: number of actuated legs. (i.e.: biped, hexapod ...)
        :param root_name: string with the robot's root leg's hierarchy name.
        :param num_joints: number of joints in each leg
        :param base_name: string with the robot name (same as base in CoppeliaSim
                                                      model).
        """

        leg_names = ['%s%d' % (root_name, i) for i in range(1, self.num_legs)]

        super().__init__(count, leg_names, num_joints, base_name)
