from pyrep.backend import vrep, utils
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint
from pyrep.robots.robot_component import RobotComponent
from pyrep.objects.cartesian_path import CartesianPath
from pyrep.robots.mobiles.configuration_path import ConfigurationPath
from pyrep.errors import ConfigurationPathError
from pyrep.const import ConfigurationPathAlgorithms as Algos
from pyrep.const import ObjectType as OT
from pyrep.const import PYREP_SCRIPT_TYPE
from contextlib import contextmanager
from typing import List
import sys
import os
import io
from math import pi, sqrt

class Mobile(RobotComponent):
    """Base class representing a robot mobile base with path planning support.
    """

    def __init__(self, count: int, name: str, type_: str, base_name: str = None,
                 max_velocity: float = 4, max_velocityRot: float = 6, max_acceleration: float = 0.035, distance_from_target: float = 0):
        """Count is used for when we have multiple copies of mobile bases"""

        suffix = '' if count == 0 else '#%d' % (count - 1)
        self.type_ = type_

        if type_ is "omnidirectional":
            self.num_wheels = 4 #Actuated wheels
            self.paramP = 20
            self.paramO = 30
            self.previousForwBackVel=0
            self.previousLeftRightVel=0
            self.previousRotVel=0
            self.accelF = max_acceleration
            self.maxV = max_velocity
            self.maxVRot = max_velocityRot
            self.dist1 = distance_from_target

            joint_slipping_names = ['%s_slipping_m_joint%s%s' %(name,str(i+1),suffix) for i in range(self.num_wheels)]
            self.joints_slipping = [Joint(jsname)
                                    for jsname in joint_slipping_names]

        elif type_ is "two_wheels":
            self.num_wheels = 2
            self.paramP = 0.1
            self.paramO = 0.8
            self.previousForwBackVel=0
            self.previousRotVel=0
            self.accelF = max_acceleration
            self.maxV = max_velocity
            self.maxVRot = max_velocityRot
            self.dist1 = distance_from_target

        joint_names = ['%s_m_joint%s' %(name,str(i+1)) for i in range(self.num_wheels)]
        super().__init__(count, name, joint_names, base_name)

        wheel_names = ['%s_wheel%s%s' %(name,str(i+1),suffix) for i in range(self.num_wheels)]
        self.wheels = [Shape(wname)
                       for wname in wheel_names]

        # Robot parameters and handle
        self.base_object = Shape(name + suffix)
        self.z_pos = self.base_object.get_position()[2]
        self.wheel_size = self.wheels[0].get_bounding_box()[1] * 2
        self.wheel_sep = abs(self.wheels[0].get_position()[1] - self.wheels[1].get_position()[1])/2


        # Motion planning handles
        self.intermediate_target_base = Dummy('%s_intermediate_target_base%s' % (name, suffix))
        self.target_base = Dummy('%s_target_base%s' % (name, suffix))
        self.base_ref = Dummy('%s_base_ref%s' % (name, suffix))
        self._collision_collection = vrep.simGetCollectionHandle(
            '%s_base%s' % (name, suffix))

        # Make sure dummies are orphan if loaded with ttm
        self.intermediate_target_base.set_parent(None)
        self.target_base.set_parent(None)

    def assess_collision(self):
        """ Silent detection of the robot base with all other entities present in the scene.

        :return: True if collision is detected
        """
        return vrep.simCheckCollision(self._collision_collection, vrep.sim_handle_all) == 1

    def set_cartesian_position(self, position: List[float]):
        """ Set a delta target position (x,y) and rotation position

        :param position: length 3 list containing x and y position, and angle position

        NOTE: not supported for two wheel robot yet.
        """
        vel = [0,0,0]
        vel[-1] = position[-1]
        for i in range(2):
            vel[i] = position[i]/(0.05*self.wheel_size/2) #"0.05 is dt"

        self.set_base_angular_velocites(vel)

    def _reset_wheel(self):
        """ Required to achieve desired omnidirectional wheel effect.
        """
        [j.reset_dynamic_object() for j in self.wheels]

        p = [[-pi/4,0,0],[pi/4,0,pi],[-pi/4,0,0],[pi/4,0,pi]]

        for i in range(self.num_wheels):
            self.joints_slipping[i].set_position([0,0,0],relative_to=self.joints[i],reset_dynamics=False)
            self.joints_slipping[i].set_orientation(p[i],relative_to=self.joints[i],reset_dynamics=False)
            self.wheels[i].set_position([0,0,0],relative_to=self.joints[i],reset_dynamics=False)
            self.wheels[i].set_orientation([0,0,0],relative_to=self.joints[i],reset_dynamics=False)

    def set_base_angular_velocites(self,velocity: List[float]):
        """ This function has no effect for two_wheels robot. More control is required for omnidirectional robot.

        :param velocity: for two wheels robot: each wheel velocity, for omnidirectional robot forwardBackward, leftRight and rotation velocity
        """
        if self.type_ is "two_wheels":
            self.set_joint_target_velocities(velocity)

        elif  self.type_ is "omnidirectional":
            self._reset_wheel()
            fBVel=velocity[0]
            lRVel=velocity[1]
            rVel=velocity[2]

            self.set_joint_target_velocities([-fBVel-lRVel-rVel,-fBVel+lRVel-rVel,-fBVel-lRVel+rVel,-fBVel+lRVel+rVel])

    def set_base_position(self,position: List[float]):
        self.base_object.set_position([position[0],position[1],self.z_pos])

    def set_base_orientation(self,angle: 0):
        if self.type_ is "two_wheels":
            self.base_object.set_orientation([0,0,angle])

        elif self.type_ is "omnidirectional":
            self.base_object.set_orientation([-90*pi/180 if angle<0 else 90*pi/180, angle, -90*pi/180 if angle<0 else 90*pi/180])

    def get_base_position(self) -> List:
        return self.base_ref.get_position()[:2]

    def get_base_orientation(self) -> List:
        return self.base_ref.get_orientation()[-1]

    def get_linear_path(self, position: List[float], angle=0) -> ConfigurationPath:
        """ Let the controller solve the path with no collision check.  assess_collision may be
        added at every simulation step.

        For a more stable linear path with sampled points get_nonlinear_path may be
        called, if no obstable along the way, a linear trajectory will be returned.
        """
        position_base = self.base_ref.get_position()
        angle_base = self.base_ref.get_orientation()[-1]

        if self.type_ is "two_wheels":
            self.target_base.set_position([position[0],position[1],self.z_pos])
            self.target_base.set_orientation([0,0,angle])
            self.intermediate_target_base.set_position([position[0],position[1],self.z_pos])
            self.intermediate_target_base.set_orientation([0,0,angle])

            # Missing the dist1 for intermediate target

        elif self.type_ is "omnidirectional":
            self.target_base.set_position([position[0],position[1],self.z_pos])
            self.target_base.set_orientation([0,0,angle])

            handleBase = self.base_ref.get_handle()
            handleTargetBase = self.target_base.get_handle()
            __, ret_floats, _, _ = utils.script_call(
                'getBoxAdjustedMatrixAndFacingAngle@PyRep', PYREP_SCRIPT_TYPE,
                ints=[handleBase, handleTargetBase])

            m = ret_floats[:-1]
            angle = ret_floats[-1]
            self.target_base.set_position([m[3]-m[0]*self.dist1,m[7]-m[4]*self.dist1,self.z_pos])
            self.target_base.set_orientation([0,0,angle])
            self.intermediate_target_base.set_position([m[3]-m[0]*self.dist1,m[7]-m[4]*self.dist1,self.z_pos])
            self.intermediate_target_base.set_orientation([0,0,angle])

        path = [[position_base[0],position_base[1],angle_base], [position[0],position[1],angle]]

        return ConfigurationPath(self, path)

    def get_nonlinear_path(self, position: List[float],
                           angle=0,
                           boundaries=2,
                           path_pts=600,
                           ignore_collisions=False) -> ConfigurationPath:
        """Gets a non-linear (planned) configuration path given a target pose.

        :param position: The x, y, z position of the target.
        :param angle: The z orientation of the target (in radians).
        :param boundaries: A float defining the path search in x and y direction
        [[-boundaries,boundaries],[-boundaries,boundaries]].
        :param path_pts: number of sampled points returned from the computed path
        :param ignore_collisions: If collision checking should be disabled.

        :return: A non-linear path (x,y,angle) in the xy configuration space.
        """

        # Base dummy required to be parent of the robot tree
        self.base_ref.set_parent(None)
        self.base_object.set_parent(self.base_ref)

            # Missing the dist1 for intermediate target

        self.target_base.set_position([position[0],position[1],self.z_pos])
        self.target_base.set_orientation([0,0,angle])

        handleBase = self.base_ref.get_handle()
        handleTargetBase = self.target_base.get_handle()

        # Despite verbosity being set to 0, OMPL spits out a lot of text
        with suppress_std_out_and_err():
            _, ret_floats, _, _ = utils.script_call(
                'getNonlinearPathMobile@PyRep', PYREP_SCRIPT_TYPE,
                ints=[handleBase, handleTargetBase, self._collision_collection,
                      int(ignore_collisions), path_pts], floats=[boundaries])

        self.base_object.set_parent(None)
        self.base_ref.set_parent(self.base_object)

        if len(ret_floats) == 0:
            raise ConfigurationPathError('Could not create path.')

        path = []
        for i in range(0,len(ret_floats)//3):
            inst = ret_floats[3*i:3*i+3]
            if i > 0:
                dist_change = sqrt((inst[0]-prev_inst[0])**2 + (inst[1]-prev_inst[1])**2)
            else:
                dist_change = 0
            inst.append(dist_change)

            path.append(inst)

            prev_inst = inst

        return ConfigurationPath(self,path)

    def _get_base_actuation(self):
        """ Controller for two wheels and omnidirectional robots.
        Based on a proportional controller. Used for motion planning.
        """

        handleBase = self.base_ref.get_handle()
        handleInterTargetBase = self.intermediate_target_base.get_handle()
        pos_v = self.target_base.get_position(relative_to=self.base_ref)
        or_v = self.target_base.get_orientation(relative_to=self.base_ref)

        if self.type_ is "two_wheels":

            __, ret_floats, _, _ = utils.script_call(
                'getAngleTwoWheel@PyRep', PYREP_SCRIPT_TYPE,
                ints=[handleBase, handleInterTargetBase])

            if sqrt((pos_v[0])**2 +(pos_v[1])**2) < 0.01: #and or_v[-1] < 0.4*pi/180: too hard to achieve orientation
                return [0,0], True

            v_des = self.paramP
            omega_des = self.paramO * ret_floats[0]
            v_R = v_des + self.wheel_sep * omega_des
            v_L = v_des - self.wheel_sep * omega_des

            omega_jointR = v_R/(self.wheel_size/2)
            omega_jointL = v_L/(self.wheel_size/2)

            return [omega_jointL, omega_jointR], False

        elif self.type_ is "omnidirectional":
            pos_inter = self.intermediate_target_base.get_position(relative_to=self.base_ref)
            or_inter = self.intermediate_target_base.get_orientation(relative_to=self.base_ref)

            if sqrt((pos_v[0])**2 +(pos_v[1])**2) < 0.001 and or_v[-1] < 0.2*pi/180:
                return [self.previousForwBackVel,self.previousLeftRightVel,self.previousRotVel], True

            ForwBackVel = pos_inter[1] * self.paramP
            LeftRightVel = pos_inter[0] * self.paramP
            RotVel = - or_inter[2] * self.paramO

            v = sqrt(ForwBackVel*ForwBackVel+LeftRightVel*LeftRightVel)
            if v>self.maxV:
                ForwBackVel = ForwBackVel*self.maxV/v
                LeftRightVel = LeftRightVel*self.maxV/v

            if (abs(RotVel)>self.maxVRot):
                RotVel=self.maxVRot*RotVel/abs(RotVel)

            df = ForwBackVel- self.previousForwBackVel
            ds = LeftRightVel - self.previousLeftRightVel
            dr = RotVel - self.previousRotVel

            if (abs(df)>self.maxV*self.accelF):
                df=abs(df)*(self.maxV*self.accelF)/df

            if (abs(ds)>self.maxV*self.accelF):
                ds=abs(ds)*(self.maxV*self.accelF)/ds

            if (abs(dr)>self.maxVRot*self.accelF):
                dr=abs(dr)*(self.maxVRot*self.accelF)/dr

            ForwBackVel = self.previousForwBackVel+df
            LeftRightVel = self.previousLeftRightVel+ds
            RotVel = self.previousRotVel+dr

            self.previousForwBackVel = ForwBackVel
            self.previousLeftRightVel = LeftRightVel
            self.previousRotVel = RotVel

            return [ForwBackVel, LeftRightVel, RotVel], False

    def get_tip(self) -> Dummy:
        # for mobile base tip corresponds to front of the base
        """Gets the center of the mobile robot.

        Each robot is required to have a tip for path planning.

        :return: The tip of the robot.
        """
        return self.base_ref

@contextmanager
def suppress_std_out_and_err():
    """Used for suppressing std out/err.

    This is needed because the OMPL plugin outputs logging info even when
    logging is turned off.
    """

    try:
        # If we are using an IDE, then this will fail
        original_stdout_fd = sys.stdout.fileno()
        original_stderr_fd = sys.stderr.fileno()
    except io.UnsupportedOperation:
        # Nothing we can do about this, just don't suppress
        yield
        return

    with open(os.devnull, "w") as devnull:

        devnull_fd = devnull.fileno()

        def _redirect_stdout(to_fd):
            sys.stdout.close()
            os.dup2(to_fd, original_stdout_fd)
            sys.stdout = io.TextIOWrapper(os.fdopen(original_stdout_fd, 'wb'))

        def _redirect_stderr(to_fd):
            sys.stderr.close()
            os.dup2(to_fd, original_stderr_fd)
            sys.stderr = io.TextIOWrapper(os.fdopen(original_stderr_fd, 'wb'))

        saved_stdout_fd = os.dup(original_stdout_fd)
        # saved_stderr_fd = os.dup(original_stderr_fd)

        try:
            _redirect_stdout(devnull_fd)
            # _redirect_stderr(devnull_fd)
            yield
            _redirect_stdout(saved_stdout_fd)
            # _redirect_stderr(saved_stderr_fd)
        finally:
            os.close(saved_stdout_fd)
            # os.close(saved_stderr_fd)
