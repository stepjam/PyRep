"""
A Franka Panda inverse kinematics example
This script contains examples of:
    - IK calculations.
"""
from os.path import dirname, join, abspath
from pyrep import PyRep
from pyrep.errors import IKError
from pyrep.robots.arms.panda import Panda

SCENE_FILE = join(dirname(abspath(__file__)), 'scene_panda_reach_target.ttt')
pr = PyRep()
pr.launch(SCENE_FILE, headless=False, responsive_ui=True)
pr.start()
agent = Panda()

starting_joint_positions = agent.get_joint_positions()
(x, y, z), q = agent.get_tip().get_position(), agent.get_tip().get_quaternion()

# Try solving via linearisation
new_joint_pos = agent.solve_ik_via_jacobian([x, y, z - 0.01], quaternion=q)
new_joint_pos = agent.solve_ik_via_jacobian([x, y, z - 0.05], quaternion=q)
new_joint_pos = agent.solve_ik_via_jacobian([x, y, z - 0.1], quaternion=q)
new_joint_pos = agent.solve_ik_via_jacobian([x, y, z - 0.2], quaternion=q)

# This will fail because the distance between start and goal is too far
try:
    new_joint_pos = agent.solve_ik_via_jacobian([x, y, z - 0.4], quaternion=q)
except IKError:
    # So let's swap to an alternative IK method...
    # This returns 'max_configs' number of joint positions
    input('Press key to run solve_ik_via_sampling...')
    new_joint_pos = agent.solve_ik_via_sampling([x, y, z - 0.4], quaternion=q)[0]

# Because the arm is in Forxe/Torque mode, we need to temporarily disable
# dynamics in order to instantaneously move joints.
agent.set_joint_positions(new_joint_pos, disable_dynamics=True)
input('Press any key to finish...')

pr.stop()
pr.shutdown()
