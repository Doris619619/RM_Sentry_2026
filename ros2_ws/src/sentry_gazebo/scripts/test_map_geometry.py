"""Check physical dynamic-clearance invariance and profile alignment on both real map sets."""
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

from ament_index_python.packages import get_package_share_directory
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from autonomy_geometry import StaticSafetyGrid, DynamicSafetyGrid
from simulation_map import map_paths
from sentry_map_profile import load_profile


class GeometryTest(unittest.TestCase):
    # Both supported resolutions should protect the same physical distance from a measured obstacle.
    def test_dynamic_radius_is_in_metres(self):
        for resolution, cells in [(.05, 8), (.1, 4)]:
            base = StaticSafetyGrid(np.zeros((100, 100), dtype=bool), resolution, (0., 0.))
            with patch.object(StaticSafetyGrid, 'load', return_value=base), \
                 patch('autonomy_geometry.Image.open', return_value=np.zeros((100, 100), dtype=np.uint8)):
                dynamic = DynamicSafetyGrid()
            self.assertEqual(dynamic.clearance_cells, cells)
            dynamic.update(np.array([[2.025, 2.025, .5]]), 1.)
            self.assertFalse(dynamic.points_free([[2.025 + .3, 2.025]]))
            self.assertTrue(dynamic.points_free([[2.025 + .6, 2.025]]))
            dynamic.update(np.empty((0, 3)), 2.1)
            self.assertTrue(dynamic.points_free([[2.025, 2.025]]))

    # Switching provider directories must switch every consumer to the same raster limits.
    def test_new_and_legacy_profiles(self):
        planning = Path(get_package_share_directory('trajectory_generation'))
        scene = Path(get_package_share_directory('sentry_gazebo')) / 'maps/sentry_2024'
        for profile, size in [(planning, (400, 400)), (scene, (301, 161))]:
            loaded = load_profile(profile)
            self.assertEqual((loaded.width, loaded.height), size)
        with patch.dict(os.environ, {'SENTRY_SIM_MAP': str(scene)}):
            grid = StaticSafetyGrid.load()
            self.assertFalse(grid.points_free([[5., 17.]]))
            self.assertTrue(grid.points_free([[5., 8.]]))


if __name__ == '__main__':
    unittest.main()
