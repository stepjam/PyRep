#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul  9 02:47:19 2020

@author: nanbaima
"""

from pyrep.robots.robot_component import RobotComponent


class Leg(RobotComponent):
    """Base class representing a robot leg with multiple joints.
    """

    def __init__(self, count: int, leg_names: str, num_joints: int,
                 base_name: str = None):
        """Count is used for when we have multiple copies of Robots"""
        suffix = '' if count == 0 else '#%d' % (count - 1)
        joint_names = [
            '%s_joint%d%s' % (leg_names, i+1, suffix) for i in range(num_joints)]
        super().__init__(count, leg_names, joint_names, base_name)

