from os import path
from os.path import dirname, join, abspath
from pyrep import PyRep
from pyrep.robots.mobiles.quadricopter import Quadricopter
from pyrep.objects.shape import Shape
from pyrep.const import PrimitiveShape
import numpy as np

LOOPS = 10
SCENE_FILE = join(dirname(abspath(__file__)), 'cooperative_transportation_uav.ttt')
pr = PyRep()
pr.launch(SCENE_FILE, headless=False)
pr.start()

agent = Quadricopter()
# We could have made this target in the scene, but lets create one dynamically
target = Shape.create(type=PrimitiveShape.CUBOID,
                      size=[0.8, 0.8, 0.8],
                      color=[1.0, 0.1, 0.1],
                      static=False, respondable=True)

position_min, position_max = [-0.4, -0.4, 1.5], [0.4, 0.4, 1.5]


#robot_DIR = path.join(path.dirname(path.abspath(__file__)), 'models')
#m = pr.import_model(path.join(robot_DIR, 'Quadricopter_rope.ttm'))


for i in range(LOOPS):

    # Get a random position within a cuboid and set the target position
    pos = list(np.random.uniform(position_min, position_max))
    agent.set_3d_pose(pos+[0.0,0.0,0.0])
    target.set_position([0.0,0.0,0.4])

    for j in np.arange(100):
        agent.set_propller_velocity(5.5, 5.5, 5.5, 5.5)

        agent.control_propeller_thrust(1)
        agent.control_propeller_thrust(2)
        agent.control_propeller_thrust(3)
        agent.control_propeller_thrust(4)

        pr.step()


pr.stop()
pr.shutdown()