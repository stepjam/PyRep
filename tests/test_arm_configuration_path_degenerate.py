"""No simulator launch needed: exercise degenerate path interpolation."""
import unittest
from unittest.mock import Mock, patch

import numpy as np

from pyrep.robots.configuration_paths.arm_configuration_path import (
    ArmConfigurationPath, sim)


class TestDegenerateArmConfigurationPath(unittest.TestCase):

    def make_path(self, points):
        arm = Mock()
        arm.joints = [Mock(), Mock()]
        arm._num_joints = 2
        arm.get_joint_count.return_value = 2
        arm.get_joint_upper_velocity_limits.return_value = [100., 100.]
        arm.max_velocity = arm.max_acceleration = arm.max_jerk = 1.
        return ArmConfigurationPath(arm, np.asarray(points).flatten())

    def test_stationary_path_completes_without_reflexxes(self):
        for count in (1, 2, 50):
            with self.subTest(points=count):
                path = self.make_path([[.1, -.2]] * count)
                with patch.object(sim, 'simRMLPos') as create:
                    with np.errstate(all='raise'):
                        self.assertTrue(path.step())
                    create.assert_not_called()
                self.assertEqual(len(path), count)
                np.testing.assert_array_equal(
                    path.get_executed_joint_position_action(), [.1, -.2])
                np.testing.assert_array_equal(
                    path._arm.set_joint_target_positions.call_args[0][0],
                    [.1, -.2])
                with self.assertRaises(RuntimeError):
                    path.step()
                path.set_to_start()
                self.assertTrue(path.step())

    def test_single_point_slice_can_be_stepped(self):
        path = self.make_path([[0., 0.], [1., 0.]])[1]
        with patch.object(sim, 'simRMLPos') as create:
            self.assertTrue(path.step())
            create.assert_not_called()
        np.testing.assert_array_equal(
            path.get_executed_joint_position_action(), [1., 0.])

    def test_duplicate_segments_are_finite_in_both_interpolators(self):
        cases = [
            ([[0., 0.], [0., 0.], [1., 0.]], 0., [0., 0.]),
            ([[0., 0.], [1., 0.], [1., 0.], [2., 0.]], 1., [1., 0.]),
            ([[0., 0.], [1., 0.], [1., 0.]], 1., [1., 0.]),
            # Reflexxes can slightly overshoot; the last segment may be zero.
            ([[0., 0.], [1., 0.], [1., 0.]], 1.0001, [1., 0.]),
        ]
        for points, position, target in cases:
            with self.subTest(points=points, position=position):
                path = self.make_path(points)
                with patch.object(sim, 'simGetSimulationTimeStep', return_value=.05), \
                        patch.object(sim, 'simRMLPos', return_value=42), \
                        patch.object(sim, 'simRMLStep', return_value=(1, [position, 0., 0.])), \
                        patch.object(sim, 'simRMLRemove'):
                    with np.errstate(all='raise'):
                        self.assertEqual(path._get_rml_handle(), 42)
                        path._rml_handle = 42
                        self.assertEqual(path._step_motion(), 1)
                np.testing.assert_allclose(
                    path.get_executed_joint_position_action(), target)

    def test_nonzero_segment_interpolation_is_unchanged(self):
        for endpoint in (1., 1e-12):
            with self.subTest(endpoint=endpoint):
                path = self.make_path([[0., 0.], [endpoint, 0.]])
                path._rml_handle = 42
                with patch.object(sim, 'simGetSimulationTimeStep', return_value=.05), \
                        patch.object(sim, 'simRMLStep', return_value=(0, [endpoint / 2, 0., 0.])):
                    with np.errstate(all='raise'):
                        self.assertEqual(path._step_motion(), 0)
                np.testing.assert_array_equal(
                    path.get_executed_joint_position_action(), [endpoint / 2, 0.])


if __name__ == '__main__':
    unittest.main()
