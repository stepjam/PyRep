"""
A baxter picks up a cup with its left arm and then passes it to its right arm.
This script contains examples of:
    - Path planning (linear and non-linear).
    - Using multiple arms.
    - Using a gripper.
"""
from os.path import dirname, join, abspath
from pyrep import PyRep
from pyrep.robots.arms.baxter import BaxterLeft, BaxterRight
from pyrep.robots.end_effectors.baxter_gripper import BaxterGripper
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape

SCENE_FILE = join(dirname(abspath(__file__)), 'scene_baxter_pick_and_pass.ttt')
pr = PyRep()

pr.launch(SCENE_FILE, headless=False)
pr.start()

baxter_left = BaxterLeft()
baxter_right = BaxterRight()
baxter_gripper_left = BaxterGripper(0)
baxter_gripper_right = BaxterGripper(1)

cup = Shape('Cup')
waypoints = [Dummy('waypoint%d' % i) for i in range(7)]

print('Planning path for left arm to cup ...')
path = baxter_left.get_path(position=waypoints[0].get_position(),
                            quaternion=waypoints[0].get_quaternion())
path.visualize()  # Let's see what the path looks like
print('Executing plan ...')
done = False
while not done:
    done = path.step()
    pr.step()
path.clear_visualization()

print('Planning path closer to cup ...')
path = baxter_left.get_path(position=waypoints[1].get_position(),
                            quaternion=waypoints[1].get_quaternion())
print('Executing plan ...')
done = False
while not done:
    done = path.step()
    pr.step()

print('Closing left gripper ...')
while not baxter_gripper_left.actuate(0.0, 0.4):
    pr.step()
baxter_gripper_left.grasp(cup)

print('Planning path to lift cup ...')
path = baxter_left.get_path(position=waypoints[2].get_position(),
                            quaternion=waypoints[2].get_quaternion(),
                            ignore_collisions=True)
print('Executing plan ...')
done = False
while not done:
    done = path.step()
    pr.step()

print('Planning path for right arm to cup ...')
path = baxter_right.get_path(position=waypoints[3].get_position(),
                             quaternion=waypoints[3].get_quaternion())
print('Executing Plan ...')
done = False
while not done:
    done = path.step()
    pr.step()

print('Planning path closer to cup ...')
path = baxter_right.get_path(position=waypoints[4].get_position(),
                             quaternion=waypoints[4].get_quaternion())
print('Executing plan ...')
done = False
while not done:
    done = path.step()
    pr.step()

print('Closing right gripper ...')
while not baxter_gripper_right.actuate(0.0, 0.4):
    pr.step()

print('Opening left gripper ...')
while not baxter_gripper_left.actuate(1.0, 0.4):
    pr.step()

# Left gripper releases, right gripper grasps.
baxter_gripper_left.release()
baxter_gripper_right.grasp(cup)
pr.step()

print('Planning path for left arm to home position ...')
path_l = baxter_left.get_path(position=waypoints[5].get_position(),
                              quaternion=waypoints[5].get_quaternion())

print('Planning path for right arm to home position ...')
path_r = baxter_right.get_path(position=waypoints[6].get_position(),
                               quaternion=waypoints[6].get_quaternion())

print('Executing plan on both arms ...')
done_l = done_r = False
while not done_l or not done_r:
    if not done_l:
        done_l = path_l.step()
    if not done_r:
        done_r = path_r.step()
    pr.step()

print('Done ...')
input('Press enter to finish ...')
pr.stop()
pr.shutdown()
