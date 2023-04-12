"""
A turtlebot reaches for 4 randomly places targets.
This script contains examples of:
    - Non-linear mobile paths to reach a target with collision avoidance
"""
from os.path import dirname, join, abspath
from pyrep import PyRep
from pyrep.robots.mobiles.turtlebot import TurtleBot
from pyrep.objects.shape import Shape
from pyrep.const import PrimitiveShape
import numpy as np

LOOPS = 4
SCENE_FILE = join(dirname(abspath(__file__)), 'scene_turtlebot_navigation.ttt')
pr = PyRep()
pr.launch(SCENE_FILE, headless=False)
pr.start()
agent = TurtleBot()

# We could have made this target in the scene, but lets create one dynamically
target = Shape.create(type=PrimitiveShape.SPHERE,
                      size=[0.05, 0.05, 0.05],
                      color=[1.0, 0.1, 0.1],
                      static=True, respondable=False)

position_min, position_max = [-0.5, 1, 0.1], [0.5, 1.5, 0.1]

starting_pose = agent.get_2d_pose()

agent.set_motor_locked_at_zero_velocity(True)

for i in range(LOOPS):
    agent.set_2d_pose(starting_pose)

    # Get a random position within a cuboid and set the target position
    pos = list(np.random.uniform(position_min, position_max))
    target.set_position(pos)

    path = agent.get_nonlinear_path(position=pos, angle=0)

    path.visualize()
    done = False

    while not done:
        done = path.step()
        pr.step()

    path.clear_visualization()

    print('Reached target %d!' % i)

pr.stop()
pr.shutdown()
