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

        self.particlesTargetVelocities = [0,0,0,0]
        self.pParam = 2
        self.iParam = 0
        self.dParam = 0
        self.vParam = -2

        self.cumul = 0
        self.lastE = 0
        self.pAlphaE = 0
        self.pBetaE = 0
        self.psp2 = 0
        self.psp1 = 0
        
        self.prevEuler = 0 

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

    def simple_controller(self, target_loc):
        '''simple PID controller'''
        
        sp = [0.0,0.0,0.0]
        euler = [0.0,0.0,0.0]
        l = []
        angu = []
    
        targetPos = target_loc
        #print("target position is: ",target_pos)
        pos = self.get_3d_pose()[:3]

        l, angu = sim.simGetVelocity(self._handle)
        e = (targetPos[2]-pos[2])
        self.cumul = self.cumul + e
        pv = self.pParam*e
        thrust = 5.335 + pv + self.iParam*self.cumul + self.dParam * (e-self.lastE) + l[2]*self.vParam
        self.lastE = e
    
        for i in range(3):
            sp[i] = targetPos[i]-pos[i]
        
        m= sim.simGetObjectMatrix(self._handle,-1)
        vx = [1,0,0]
        vx = sim.simMultiplyVector(m,vx)
        vy = [0,1,0]
        vy = sim.simMultiplyVector(m,vy)

        alphaE = (vy[2]-m[11])
        alphaCorr = 0.25*alphaE + 2.1*(alphaE-self.pAlphaE)
        betaE = (vx[2]-m[11])
        betaCorr = -0.25*betaE-2.1*(betaE-self.pBetaE)
        self.pAlphaE = alphaE
        self.pBetaE = betaE
        alphaCorr = alphaCorr + sp[1]*0.005 + 1*(sp[1]-self.psp2)
        betaCorr = betaCorr-sp[0]*0.005 - 1*(sp[0]-self.psp1)
        self.psp2 = sp[1]
        self.psp1 = sp[0]
    
        eulert1 = self.get_3d_pose()[3:]
        eulert2 = [0.0,0.0,0.0]

        for i in range(3):
            euler[i] = eulert1[i]-eulert2[i]
        
        rotCorr = euler[2]*0.1 + 2*(euler[2]-self.prevEuler)
        self.prevEuler = euler[2]
    
        self.particlesTargetVelocities[0] = thrust*(1-alphaCorr+betaCorr+rotCorr)
        self.particlesTargetVelocities[1] = thrust*(1-alphaCorr-betaCorr-rotCorr)
        self.particlesTargetVelocities[2] = thrust*(1+alphaCorr-betaCorr+rotCorr)
        self.particlesTargetVelocities[3] = thrust*(1+alphaCorr+betaCorr-rotCorr)
    
        return self.particlesTargetVelocities

    