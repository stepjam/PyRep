"""
Facebook/CMU LoCoBot drives to a cube, picks it up, and takes it to a target.
This script contains examples of:
    - Linear mobile paths on a non-holonomic platform
    - How to use the combination of a mobile base, manipulator, and gripper.
"""
from os.path import dirname, join, abspath
from pyrep import PyRep
from pyrep.robots.mobiles.locobot import LoCoBot
from pyrep.robots.arms.locobot_arm import LoCoBotArm
from pyrep.robots.end_effectors.locobot_gripper import LoCoBotGripper
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy

SCENE_FILE = join(dirname(abspath(__file__)), 'scene_locobot_stack_cube.ttt')
pr = PyRep()
pr.launch(SCENE_FILE, headless=False)
pr.start()

base = LoCoBot()
arm = LoCoBotArm()
gripper = LoCoBotGripper()

base.set_motor_locked_at_zero_velocity(True)


def drive_to_position(position, orientation):
    base_path = base.get_linear_path(position, orientation)
    base_path.visualize()
    done = False
    while not done:
        done = base_path.step()
        pr.step()
    pr.step()


def move_arm(position, quaternion, ignore_collisions=False):
    arm_path = arm.get_path(position,
                            quaternion=quaternion,
                            ignore_collisions=ignore_collisions)
    arm_path.visualize()
    done = False
    while not done:
        done = arm_path.step()
        pr.step()
    arm_path.clear_visualization()


cuboid = Shape('cuboid')
goal = Shape('goal')
grasp_point = Dummy('grasp_point')

drive_pos = cuboid.get_position()
drive_pos[1] -= 0.3

print('Driving to cube ...')
drive_to_position(drive_pos, 0)

grasp_point_raised = grasp_point.get_position()
grasp_point_raised[2] += 0.075

print('Move arm above cube ...')
move_arm(grasp_point_raised, grasp_point.get_quaternion())

print('Arm approach cube ...')
move_arm(grasp_point.get_position(), grasp_point.get_quaternion(), True)

print('Closing gripper ...')
while not gripper.actuate(0.0, 0.4):
    pr.step()
gripper.grasp(cuboid)

print('Lift cube ...')
move_arm(grasp_point_raised, grasp_point.get_quaternion(), True)

drive_pos = goal.get_position()
drive_pos[1] -= 0.35

print('Driving to goal ...')
drive_to_position(drive_pos, 0)

goal_point_raised = goal.get_position()
goal_point_raised[2] += 0.05

print('Move arm above goal ...')
move_arm(goal_point_raised, grasp_point.get_quaternion())

print('Drop cube ...')
gripper.release()
while not gripper.actuate(1.0, 0.4):
    pr.step()

print('Done!')

pr.stop()
pr.shutdown()
