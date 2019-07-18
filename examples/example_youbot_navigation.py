"""
A Kuka youBot reaches for 4 randomly places targets.
This script contains examples of:
    - Nonlinear mobile paths with an omnidirectional robot to reach a target
"""
from os.path import dirname, join, abspath
from pyrep import PyRep
from pyrep.robots.mobiles.youBot import youBot
from pyrep.objects.shape import Shape
from pyrep.const import PrimitiveShape
from pyrep.errors import ConfigurationPathError
import numpy as np
import math
import time

LOOPS = 4
SCENE_FILE = join(dirname(abspath(__file__)), 'youbot.ttt')
pr = PyRep()
pr.launch(SCENE_FILE, headless=False)
pr.start()
agent = youBot()

# We could have made this target in the scene, but lets create one dynamically
target = Shape.create(type=PrimitiveShape.SPHERE,
                      size=[0.05, 0.05, 0.05],
                      color=[1.0, 0.1, 0.1],
                      static=True, respondable=False)

position_min, position_max = [-0.5, 1.4, 0.1], [1.0, 0.5, 0.1]

starting_pose = agent.get_2d_pose()

for i in range(LOOPS):
    agent.set_2d_pose(starting_pose)

    # Get a random position within a cuboid and set the target position
    pos = list(np.random.uniform(position_min, position_max))
    target.set_position(pos)

    path = agent.get_linear_path(position=pos, angle=0)

    path.visualize()

    done = False
    while not done:
        _, done = path.step()
        pr.step()

    path.clear_visualization()

    print('Reached target %d!' % i)

pr.stop()
pr.shutdown()