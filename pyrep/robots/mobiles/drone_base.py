from pyrep.objects.force_sensor import ForceSensor
from pyrep.objects.object import Object
from pyrep.backend import sim
from pyrep.objects.shape import Shape
import numpy as np
from typing import List, Union
from pyrep.const import ObjectType


class Drone_base(Object):
    """ This class is based on the quadricopter model in Coppeliasim. To accelerate the learning, 
    the propellers (Joints) are removed. The components only contains the primitive shapes and 
    the force sensors. The wind particles have been removed """

    def __init__(self, count:int, num_propeller:int, name:str):

        force_sensor_names = ['%s_propeller%s' % (name, str(i + 1)) for i in range(num_propeller)]
        respondable_names = ['%s_propeller_respondable%s' % (name, str(i+1)) for i in range(num_propeller)]
        suffix = '' if count == 0 else '#%d' % (count - 1)

        #get the handle of the drone base
        super().__init__(name + suffix)

        self._num_propeller = num_propeller

        #get handles of force sensor
        self.force_sensors = [ForceSensor(fname + suffix).get_handle() for fname in force_sensor_names]
        #get the handles of propeller respondable
        self.respondables = [Shape(sname+suffix)._handle for sname in respondable_names]

        #add some simulation parameters
        self.particleCountPerSecond = 430
        self.particleDensity = 8500
        self.particleSize = 1
        self.notFullParticles = 0
        self.pre_v = 0  # previous size factor

    def get_3d_pose(self) -> np.ndarray:
        """Gets the ground truth 3D pose of the robot [x, y, z, yaw, pitch, roll].

        :return: A List containing the x, y, z, roll, pitch, yaw (in radians).
        """
        return np.r_[self.get_position()[:], self.get_orientation()[:]]
    
    def set_3d_pose(self, pose: Union[List[float], np.ndarray]) -> None:
        """Sets the 3D pose of the robot [x, y, z, yaw, pitch, roll]

        :param pose: A List containing the x, y, z, roll, pitch, yaw (in radians).
        """
        x, y, z, roll, pitch, yaw = pose
        self.set_position([x, y, z])
        self.set_orientation([roll, pitch, yaw])
    
    def control_propeller_thrust(self, num_p) -> None:
        """ set thrust for the particular propeller, num_p is the number of the propeller(1 to 4) """

        particleVelocity = self.velocities[num_p-1]
        s = sim.simGetObjectSizeFactor(self.force_sensors[num_p-1]) # current size factor
        if (s!= self.pre_v):      #When the size of the propeller changes
            self.particleSize = self.particleSize * 0.005 * s
            self.pre_v = s

        self.ts = sim.simGetSimulationTimeStep() 
    
        m = sim.simGetObjectMatrix(self.force_sensors[num_p-1], -1)

        requiredParticleCnt = self.particleCountPerSecond * self.ts + self.notFullParticles
        self.notFullParticles = requiredParticleCnt % 1
        requiredParticleCnt = np.floor(requiredParticleCnt)

        totalExertedForce = requiredParticleCnt * self.particleDensity * particleVelocity * np.pi * self.particleSize * self.particleSize * self.particleSize/(6*self.ts)
        force = [0,0,totalExertedForce]
        m[3] = 0
        m[7] = 0
        m[11] = 0
        force = sim.simMultiplyVector(m,force)
        
        torque = [0, 0, pow(-1,num_p)*0.002 * particleVelocity]
        torque = sim.simMultiplyVector(m,torque)

        sim.simAddForceAndTorque(self.respondables[num_p-1], force, torque) 
        
    def set_propller_velocity(self,v1, v2, v3, v4) -> None:
        """ set the motor velocities for the propellers """
        self.velocities = [v1, v2, v3, v4]  
       
    def _get_requested_type(self) -> ObjectType:
        """Gets the type of the object.

        :return: Type of the object.
        """
        return ObjectType(sim.simGetObjectType(self.get_handle()))
    


    