"""Exercise the public map profile with old/new rasters and rejected external configurations."""
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'python'))
from sentry_map_profile import load_profile


class MapProfileTest(unittest.TestCase):
    # Use real legacy assets so the retained map contract is exercised, not a mocked image header.
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        for name in ['occfinal.png', 'bevfinal.png', 'occtopo.png']:
            shutil.copy(PACKAGE / 'map' / name, self.root / name)
        shutil.copy(PACKAGE / 'config/map_metadata.yaml', self.root / 'map_metadata.yaml')

    # An existing package and an external profile must resolve identical map geometry.
    def test_external_profile_matches_legacy(self):
        legacy, external = load_profile(PACKAGE), load_profile(self.root)
        self.assertEqual((external.width, external.height), (400, 400))
        self.assertEqual(legacy.planner, external.planner)
        self.assertEqual(external.parameters('trajectory_tracking')['maps.directory'], str(self.root))

    # An apparently compatible topic is not sufficient when map geometry disagrees.
    def test_disagreeing_tracker_metadata_is_rejected(self):
        data = yaml.safe_load((self.root / 'map_metadata.yaml').read_text())
        data['trajectory_tracking']['ros__parameters']['map.lower_x'] += 1
        (self.root / 'map_metadata.yaml').write_text(yaml.safe_dump(data))
        with self.assertRaisesRegex(ValueError, 'disagree'):
            load_profile(self.root)

    # Reject the former 30x18 metadata for a 301x161 raster instead of accepting invisible free space.
    def test_raster_extent_is_not_silently_padded(self):
        data = yaml.safe_load((self.root / 'map_metadata.yaml').read_text())
        data['trajectory_generation']['ros__parameters']['planner.map_x_size'] = 21.
        data['trajectory_tracking']['ros__parameters']['map.x_size'] = 21.
        (self.root / 'map_metadata.yaml').write_text(yaml.safe_dump(data))
        with self.assertRaisesRegex(ValueError, 'extent'):
            load_profile(self.root)

    # A wrong manifest origin would otherwise misalign visualization and navigation.
    def test_manifest_origin_must_agree(self):
        (self.root / 'map_manifest.json').write_text(json.dumps({'lower': [0., 0.]}))
        with self.assertRaisesRegex(ValueError, 'origins'):
            load_profile(self.root)


if __name__ == '__main__':
    unittest.main()
