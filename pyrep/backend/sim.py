"""Backend"""

import threading
from ctypes import c_char_p


from pyrep.backend import lib
from pyrep.backend import bridge
from pyrep.backend.sim_const import sim_gui_all, sim_gui_headless, \
    sim_stringparam_verbosity, sim_stringparam_statusbarverbosity

sim_api = None

class SimBackend:

    _instance = None

    def __new__(cls):
        # Singleton pattern
        if cls._instance is None:
            print('Creating the object')
            cls._instance = super(SimBackend, cls).__new__(cls)
            # Put any initialization here.
        return cls._instance

    @property
    def sim_api(self):
        return self._sim

    @property
    def sim_ik_api(self):
        return self._sim_ik

    @property
    def sim_ompl_api(self):
        return self._sim_ompl

    @property
    def lib(self):
        return lib

    def simInitialize(self, appDir: str, verbosity: str):
        lib.simSetStringParam(sim_stringparam_verbosity, c_char_p(verbosity.encode('utf-8')))
        lib.simSetStringParam(sim_stringparam_statusbarverbosity, c_char_p(verbosity.encode('utf-8')))
        lib.simInitialize(c_char_p(appDir.encode('utf-8')), 0)
        bridge.load()
        # fetch CoppeliaSim API sim-namespace functions:
        self._sim = bridge.require('sim')
        self._sim_ik = bridge.require('simIK')
        self._sim_ompl = bridge.require('simOMPL')
        v = self._sim.getInt32Param(self._sim.intparam_program_full_version)
        version = '.'.join(str(v // 100 ** (3 - i) % 100) for i in range(4))
        print('CoppeliaSim version is:', version)
        return self._sim

    def create_ui_thread(self, headless: bool) -> threading.Thread:
        options = sim_gui_headless if headless else sim_gui_all
        ui_thread = threading.Thread(
            target=lib.simRunGui,
            args=(options,))
        ui_thread.daemon = True
        return ui_thread

    def simDeinitialize(self):
        lib.simDeinitialize()

    def simGetExitRequest(self) -> bool:
        return bool(lib.simGetExitRequest())

    def simLoop(self, step_phys: bool = False):
        # Second value toggles stepIfRunning
        lib.simLoop(None, int(not step_phys))

    def simStartSimulation(self):
        if self._sim.getSimulationState() == self._sim.simulation_stopped:
            self._sim.startSimulation()

    def simStep(self):
        if self._sim.getSimulationState() != self._sim.simulation_stopped:
            t = self._sim.getSimulationTime()
            while t == self._sim.getSimulationTime():
                lib.simLoop(None, 0)

    def simStopSimulation(self):
        while self._sim.getSimulationState() != self._sim.simulation_stopped:
            self._sim.stopSimulation()
            lib.simLoop(None, 0)
