from os import path
from os.path import dirname, join, abspath
from pyrep import PyRep
from pyrep.robots.mobiles.quadricopter import Quadricopter
from pyrep.objects.shape import Shape
from pyrep.const import PrimitiveShape
import numpy as np

LOOPS = 10
SCENE_FILE = join(dirname(abspath(__file__)), 'example_drone.ttt')
pr = PyRep()
pr.launch(SCENE_FILE, headless=False)
pr.start()

agent = Quadricopter()

# create a Cuboid
target = Shape.create(type=PrimitiveShape.CUBOID,
                      size=[0.8, 0.8, 0.8],
                      color=[1.0, 0.1, 0.1],
                      static=False, respondable=True)

position_min, position_max = [-0.4, -0.4, 1.5], [0.4, 0.4, 1.5]

for i in range(LOOPS):

    # Get a random position for the drone
    pos = list(np.random.uniform(position_min, position_max))
    agent.set_3d_pose(pos+[0.0,0.0,0.0]) # The orientation here is all zeros

    #Set the position for the cuboid
    target.set_position([0.0,0.0,0.4])

    for j in np.arange(100):

        # The velcities set here are just rubbish.
        # YOU SHOULD DEVELOP YOUR OWN VELOCITY CONTROL METHOD
        agent.set_propller_velocity(5.5, 5.5, 5.5, 5.5)

        # Power each propeller
        agent.control_propeller_thrust(1)
        agent.control_propeller_thrust(2)
        agent.control_propeller_thrust(3)
        agent.control_propeller_thrust(4)

        pr.step()


pr.stop()
pr.shutdown()