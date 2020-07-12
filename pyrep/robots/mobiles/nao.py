#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul  9 04:16:23 2020

@author: nanbaima
"""

from pyrep.robots.legs.leg import Leg
from pyrep.robots.arms.arm import Arm


class NAOLeftLeg(Leg):

    def __init__(self, count: int = 0):
        super().__init__(count, 1, 'NAO_leftLeg', 6, base_name='NAO')


class NAORightLeg(Leg):

    def __init__(self, count: int = 0):
        super().__init__(count, 1, 'NAO_rightLeg', 6, base_name='NAO')


class NAOLeftArm(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'NAO_leftArm', 5, base_name='NAO')


class NAORightArm(Arm):

    def __init__(self, count: int = 0):
        super().__init__(count, 'NAO_rightArm', 5, base_name='NAO')