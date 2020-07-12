#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul  9 02:56:54 2020

@author: nanbaima
"""

from pyrep.robots.legs.leg import Leg


class NAOLeftLeg(Leg):

    def __init__(self, count: int = 0):
        super().__init__(count, 'NAO_leftLeg', 6, base_name='NAO')


class NAORightLeg(Leg):

    def __init__(self, count: int = 0):
        super().__init__(count, 'NAO_rightLeg', 6, base_name='NAO')