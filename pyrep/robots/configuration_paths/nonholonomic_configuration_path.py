from pyrep.robots.configuration_paths.mobile_configuration_path import (
    MobileConfigurationPath)
from math import sqrt


class NonHolonomicConfigurationPath(MobileConfigurationPath):
    """A path expressed in joint configuration space.

    Paths are retrieved from an :py:class:`Mobile`, and are associated with the
    mobile base that generated the path.

    This class is used for executing motion along a path via the
    _get_base_actuation function employing a proportional controller.
    """

    def step(self) -> bool:
        """Make a step along the trajectory.

        Step forward by calling _get_base_actuation to get the velocity needed
        to be applied at the wheels.

        NOTE: This does not step the physics engine. This is left to the user.

        :return: If the end of the trajectory has been reached.

        """
        if self._path_done:
            raise RuntimeError('This path has already been completed. '
                               'If you want to re-run, then call set_to_start.')

        pos_inter = self._mobile.intermediate_target_base.get_position(
            relative_to=self._mobile)

        if len(self._path_points) > 2:  # Non-linear path
            if self.inter_done:
                self._next_i_path()
                self._set_inter_target(self.i_path)
                self.inter_done = False

            if sqrt((pos_inter[0]) ** 2 + (pos_inter[1]) ** 2) < 0.1:
                self.inter_done = True
                actuation, _ = self._mobile.get_base_actuation()
            else:
                actuation, _ = self._mobile.get_base_actuation()

            self._mobile.set_joint_target_velocities(actuation)

            if self.i_path == len(self._path_points) - 1:
                self._path_done = True

        else:
            actuation, self._path_done = self._mobile.get_base_actuation()
            self._mobile.set_joint_target_velocities(actuation)

        return self._path_done
