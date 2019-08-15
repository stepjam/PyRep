from pyrep.backend import vrep, utils
from pyrep.objects.object import Object
from pyrep.objects.shape import Shape
from pyrep.textures.texture import Texture
from pyrep.errors import PyRepError
import os
import sys
import time
import threading
from threading import Lock
from typing import Tuple, List


class PyRep(object):
    """Used for interfacing with the V-REP simulation.

    Can be used for starting, stopping, and stepping the simulation. As well
    as getting, and creating scene objects and robots.
    """

    def __init__(self):
        self.running = False
        self._process = None
        self._robot_to_count = {}
        self.connected = False

        self._ui_thread = None
        self._responsive_ui_thread = None
        self._step_lock = Lock()

        self._init_thread_id = None
        self._shutting_down = False

        self._handles_to_objects = {}

        if 'VREP_ROOT' not in os.environ:
            raise PyRepError(
                'VREP_ROOT not defined. See installation instructions.')
        self._vrep_root = os.environ['VREP_ROOT']
        if not os.path.exists(self._vrep_root):
            raise PyRepError(
                'VREP_ROOT was not a correct path. '
                'See installation instructions')

    def _run_ui_thread(self, scene_file: str, headless: bool) -> None:
        # Need this otherwise extensions will not be loaded
        os.chdir(self._vrep_root)
        options = vrep.sim_gui_headless if headless else vrep.sim_gui_all
        vrep.simExtLaunchUIThread(options=options, scene=scene_file,
                                  pyrep_root=self._vrep_root)

    def _run_responsive_ui_thread(self) -> None:
        while True:
            if not self.running:
                with self._step_lock:
                    if self._shutting_down or vrep.simExtGetExitRequest():
                        break
                    vrep.simExtStep(False)
            time.sleep(0.01)
        # If the exit request was from the UI, then call shutdown, otherwise
        # shutdown caused this thread to terminate.
        if not self._shutting_down:
            self.shutdown()

    def launch(self, scene_file="", headless=False, responsive_ui=False,
               blocking=False) -> None:
        """Launches V-REP.

        Launches the UI thread, waits until the UI thread has finished, this
        results in the current thread becoming the simulation thread.

        :param scene_file: The scene file to load. Empty string for empty scene.
        :param headless: Run V-REP in simulation mode.
        :param responsive_ui: If True, then a separate thread will be created to
            asynchronously step the UI of V-REP. Note, that will reduce
            the responsiveness of the simulation thread.
        :param blocking: Causes V-REP to launch as if running the default c++
            client application. This is causes the function to block. For most
            users, this will be set to False.
        """
        if len(scene_file) > 0 and not os.path.isfile(
                os.path.abspath(scene_file)):
            raise PyRepError('Scene file does not exist: %s' % scene_file)
        cwd = os.getcwd()
        self._ui_thread = threading.Thread(target=self._run_ui_thread,
                                           args=(scene_file, headless))
        self._ui_thread.daemon = True
        self._ui_thread.start()

        while not vrep.simExtCanInitSimThread():
            time.sleep(0.1)

        vrep.simExtSimThreadInit()
        time.sleep(0.2)  # Stops V-REP crashing if it is restarted too quickly.

        if blocking:
            while not vrep.simExtGetExitRequest():
                vrep.simExtStep()
            self.shutdown()
        elif responsive_ui:
            self._responsive_ui_thread = threading.Thread(
                target=self._run_responsive_ui_thread)
            self._responsive_ui_thread.daemon = True
            try:
                self._responsive_ui_thread.start()
            except (KeyboardInterrupt, SystemExit):
                if not self._shutting_down:
                    self.shutdown()
                sys.exit()
            self.step()
        else:
            self.step()
        os.chdir(cwd)  # Go back to the previous cwd

    def script_call(self, function_name_at_script_name: str,
                    script_handle_or_type: int,
                    ints=(), floats=(), strings=(), bytes='') -> (
            Tuple[List[int], List[float], List[str], str]):
        """Calls a script function (from a plugin, the main client application,
        or from another script). This represents a callback inside of a script.

        :param function_name_at_script_name: A string representing the function
            name and script name, e.g. myFunctionName@theScriptName. When the
            script is not associated with an object, then just specify the
            function name.
        :param script_handle_or_type: The handle of the script, otherwise the
            type of the script.
        :param ints: The input ints to the script.
        :param floats: The input floats to the script.
        :param strings: The input strings to the script.
        :param bytes: The input bytes to the script (as a string).
        :return: Any number of return values from the called Lua function.
        """
        return utils.script_call(
            function_name_at_script_name, script_handle_or_type, ints, floats,
            strings, bytes)

    def shutdown(self) -> None:
        """Shuts down the V-REP simulation.
        """
        if self._ui_thread is None:
            raise PyRepError('V-REP has not been launched. Call launch first.')
        if self._ui_thread is not None:
            self._shutting_down = True
            self.stop()
            self.step_ui()
            vrep.simExtPostExitRequest()
            vrep.simExtSimThreadDestroy()
            self._ui_thread.join()
            if self._responsive_ui_thread is not None:
                self._responsive_ui_thread.join()
            # V-REP crashes if new instance opened too quickly after shutdown.
            # TODO: A small sleep stops this for now.
            time.sleep(0.1)
        self._ui_thread = None
        self._shutting_down = False

    def start(self) -> None:
        """Starts the physics simulation if it is not already running.
        """
        if self._ui_thread is None:
            raise PyRepError('V-REP has not been launched. Call launch first.')
        if not self.running:
            vrep.simStartSimulation()
            self.running = True

    def stop(self) -> None:
        """Stops the physics simulation if it is running.
        """
        if self._ui_thread is None:
            raise PyRepError('V-REP has not been launched. Call launch first.')
        if self.running:
            vrep.simStopSimulation()
            self.running = False
            # Need this so the UI updates
            [self.step() for _ in range(5)]

    def step(self) -> None:
        """Execute the next simulation step.

        If the physics simulation is not running, then this will only update
        the UI.
        """
        with self._step_lock:
            vrep.simExtStep()

    def step_ui(self) -> None:
        """Update the UI.

        This will not execute the next simulation step, even if the physics
        simulation is running.
        This is only applicable when PyRep was launched without a responsive UI.
        """
        with self._step_lock:
            vrep.simExtStep(False)

    def set_simulation_timestep(self, dt: float) -> None:
        """Sets the simulation time step. Default is 0.05.

        :param dt: The time step value in seconds.
        """
        vrep.simSetFloatParameter(vrep.sim_floatparam_simulation_time_step, dt)

    def set_configuration_tree(self, config_tree: bytes) -> None:
        """Restores configuration information previously retrieved.

        Configuration information (object relative positions/orientations,
        joint/path values) can be retrieved with
        :py:meth:`Object.get_configuration_tree`. Dynamically simulated
        objects will implicitly be reset before the command is applied
        (i.e. similar to calling :py:meth:`Object.reset_dynamic_object` just
        before).

        :param config_tree: The configuration tree to restore.
        """
        vrep.simSetConfigurationTree(config_tree)

    def group_objects(self, objects: List[Shape]) -> Shape:
        """Groups several shapes into a compound shape (or simple shape).

        :param objects: The list of shapes to group.
        :return: A single grouped shape.
        """
        handles = [o.get_handle() for o in objects]
        handle = vrep.simGroupShapes(handles)
        return Shape(handle)

    def merge_objects(self, objects: List[Shape]) -> Shape:
        """Merges several shapes into a compound shape (or simple shape).

        :param objects: The list of shapes to group.
        :return: A single merged shape.
        """
        handles = [o.get_handle() for o in objects]
        handle = vrep.simGroupShapes(handles, merge=True)
        return Shape(handle)

    def import_model(self, filename: str) -> Object:
        """	Loads a previously saved model.

        :param filename: model filename. The filename extension is required
            ("ttm"). An optional "@copy" can be appended to the filename, in
            which case the model's objects will be named/renamed as if an
            associated script was attached to the model.
        :return: The imported model.
        """
        handle = vrep.simLoadModel(filename)
        return utils.to_type(handle)

    def create_texture(self, filename: str, interpolate=True, decal_mode=False,
                       repeat_along_u=False, repeat_along_v=False
                       ) -> Tuple[Shape, Texture]:
        """Creates a planar shape that is textured.

        :param filename: Path to the texture to load.
        :param interpolate: Adjacent texture pixels are not interpolated.
        :param decal_mode: Texture is applied as a decal (its appearance
            won't be influenced by light conditions).
        :param repeat_along_u: Texture will be repeated along the U direction.
        :param repeat_along_v: Texture will be repeated along the V direction.
        :return: A tuple containing the textured plane and the texture.
        """
        options = 0
        if not interpolate:
            options |= 1
        if decal_mode:
            options |= 2
        if repeat_along_u:
            options |= 3
        if repeat_along_v:
            options |= 4
        handle = vrep.simCreateTexture(filename, options)
        s = Shape(handle)
        return s, s.get_texture()
