import numpy as np

from pyrep.backend import utils
from pyrep.backend.sim import SimBackend
from pyrep.backend.sim_const import sim_floatparam_simulation_time_step

# from pyrep.backend import sim, utils
from pyrep.const import Verbosity
from pyrep.objects.object import Object
from pyrep.objects.shape import Shape
from pyrep.textures.texture import Texture
from pyrep.errors import PyRepError

# from pyrep.backend import sim
import os
import sys
import time
import threading
from typing import Tuple, List
import warnings


class PyRep(object):
    """Used for interfacing with the CoppeliaSim simulation.

    Can be used for starting, stopping, and stepping the simulation. As well
    as getting, and creating scene objects and robots.
    """

    def __init__(self):
        self.running = False
        self._ui_thread = None
        self._handles_to_objects = {}
        self._step_lock = utils.step_lock
        self._sim_api = None  # check later
        self._shutting_down = False

        if "COPPELIASIM_ROOT" not in os.environ:
            raise PyRepError(
                "COPPELIASIM_ROOT not defined. See installation instructions."
            )
        self._coppeliasim_root = os.environ["COPPELIASIM_ROOT"]
        if not os.path.exists(self._coppeliasim_root):
            raise PyRepError(
                "COPPELIASIM_ROOT was not a correct path. "
                "See installation instructions"
            )

    def _run_responsive_ui_thread(self) -> None:
        while True:
            with utils.step_lock:
                if self._shutting_down or self._sim_backend.simGetExitRequest():
                    break
                self._sim_backend.simLoop()
            time.sleep(0.01)
        # If the exit request was from the UI, then call shutdown, otherwise
        # shutdown caused this thread to terminate.
        if not self._shutting_down:
            self.shutdown()

    def launch(
        self,
        scene_file: str = "",
        headless: bool = False,
        responsive_ui: bool = False,
        blocking: bool = False,
        verbosity: Verbosity = Verbosity.NONE,
    ) -> None:
        """Launches CoppeliaSim.

        Launches the UI thread, waits until the UI thread has finished, this
        results in the current thread becoming the simulation thread.

        :param scene_file: The scene file to load. Empty string for empty scene.
        :param headless: Run CoppeliaSim in simulation mode.
        :param responsive_ui: If True, then a separate thread will be created to
            asynchronously step the UI of CoppeliaSim. Note, that will reduce
            the responsiveness of the simulation thread.
        :param blocking: Causes CoppeliaSim to launch as if running the default
            c++ client application. This is causes the function to block.
            For most users, this will be set to False.
        :param verbosity: The verbosity level for CoppeliaSim.
            Usually Verbosity.NONE or Verbosity.LOAD_INFOS.
        """
        abs_scene_file = os.path.abspath(scene_file)
        if len(scene_file) > 0 and not os.path.isfile(abs_scene_file):
            raise PyRepError("Scene file does not exist: %s" % scene_file)
        self._sim_backend = SimBackend()
        self._ui_thread = self._sim_backend.create_ui_thread(headless)
        self._ui_thread.start()
        self._sim_api = self._sim_backend.simInitialize(
            self._coppeliasim_root, verbosity.value
        )
        if len(scene_file) > 0:
            self._sim_api.loadScene(abs_scene_file)

        if blocking:
            while not self._sim_backend.simGetExitRequest():
                self._sim_backend.simLoop(True)
            self.shutdown()
        elif responsive_ui:
            self._responsive_ui_thread = threading.Thread(
                target=self._run_responsive_ui_thread
            )
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

    def shutdown(self) -> None:
        """Shuts down the CoppeliaSim simulation."""
        if self._ui_thread is None:
            raise PyRepError("CoppeliaSim has not been launched. Call launch first.")
        if self._ui_thread is not None:
            # self._shutting_down = True
            self.stop()
            self.step_ui()
            self._sim_backend.simDeinitialize()
            # sim.simExtPostExitRequest()
            # sim.simExtSimThreadDestroy()
            # self._ui_thread.join()
            # if self._responsive_ui_thread is not None:
            #     self._responsive_ui_thread.join()
            # # CoppeliaSim crashes if new instance opened too quickly after shutdown.
            # # TODO: A small sleep stops this for now.
            # time.sleep(0.1)
        self._ui_thread = None
        # self._shutting_down = False

    def start(self) -> None:
        """Starts the physics simulation if it is not already running."""
        if self._ui_thread is None:
            raise PyRepError("CoppeliaSim has not been launched. Call launch first.")
        if not self.running:
            self._sim_backend.simStartSimulation()
            self.running = True

    def stop(self) -> None:
        """Stops the physics simulation if it is running."""
        if self._ui_thread is None:
            raise PyRepError("CoppeliaSim has not been launched. Call launch first.")
        if self.running:
            self._sim_backend.simStopSimulation()
            self.running = False

            # # Need this so the UI updates
            # [self.step() for _ in range(5)]  # type: ignore

    def step(self) -> None:
        """Execute the next simulation step.

        If the physics simulation is not running, then this will only update
        the UI.
        """
        with self._step_lock:
            self._sim_backend.simStep()

    def step_ui(self) -> None:
        """Update the UI.

        This will not execute the next simulation step, even if the physics
        simulation is running.
        This is only applicable when PyRep was launched without a responsive UI.
        """
        with self._step_lock:
            self._sim_backend.simLoop()

    def set_simulation_timestep(self, dt: float) -> None:
        """Sets the simulation time step. Default is 0.05.

        :param dt: The time step value in seconds.
        """
        self._sim_api.setFloatParameter(sim_floatparam_simulation_time_step, dt)
        if not np.allclose(self.get_simulation_timestep(), dt):
            warnings.warn(
                "Could not change simulation timestep. You may need "
                'to change it to "custom dt" using simulation '
                "settings dialog."
            )

    def get_simulation_timestep(self) -> float:
        """Gets the simulation time step.

        :return: The time step value in seconds.
        """
        return self._sim_api.getSimulationTimeStep()

    def group_objects(self, objects: List[Shape]) -> Shape:
        """Groups several shapes into a compound shape (or simple shape).

        :param objects: The list of shapes to group.
        :return: A single grouped shape.
        """
        handles = [o.get_handle() for o in objects]
        handle = self._sim_api.groupShapes(handles, False)
        return Shape(handle)

    def merge_objects(self, objects: List[Shape]) -> Shape:
        """Merges several shapes into a compound shape (or simple shape).

        :param objects: The list of shapes to group.
        :return: A single merged shape.
        """
        handles = [o.get_handle() for o in objects]
        # FIXME: self._sim_api.groupShapes(merge=True) won't return correct handle,
        # so we use name to find correct handle of the merged shape.
        name = objects[-1].get_name()
        self._sim_api.groupShapes(handles, True)
        return Shape(name)

    def export_scene(self, filename: str) -> None:
        """Saves the current scene.

        :param filename: scene filename. The filename extension is required
            ("ttt").
        """
        self._sim_api.saveScene(filename)

    def import_model(self, filename: str) -> Object:
        """Loads a previously saved model.

        :param filename: model filename. The filename extension is required
            ("ttm"). An optional "@copy" can be appended to the filename, in
            which case the model's objects will be named/renamed as if an
            associated script was attached to the model.
        :return: The imported model.
        """
        handle = self._sim_api.loadModel(filename)
        return utils.to_type(handle)

    def create_texture(
        self,
        filename: str,
        interpolate=True,
        decal_mode=False,
        repeat_along_u=False,
        repeat_along_v=False,
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
        handle, texid, res = self._sim_api.createTexture(filename, options)
        s = Shape(handle)
        return s, s.get_texture()

    def get_objects_in_tree(self, root_object=None, *args, **kwargs) -> List[Object]:
        """Retrieves the objects in a given hierarchy tree.

        :param root_object: The root object in the tree. Pass None to retrieve
            all objects in the configuration tree. :py:class:`Object` or `int`.
        :param object_type: The object type to retrieve.
            One of :py:class:`.ObjectType`.
        :param exclude_base: Exclude the tree base from the returned list.
        :param first_generation_only: Include in the returned list only the
            object's first children. Otherwise, entire hierarchy is returned.
        :return: A list of objects in the hierarchy tree.
        """
        return Object._get_objects_in_tree(root_object, *args, **kwargs)

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
        self._sim_api.setConfigurationTree(config_tree)
