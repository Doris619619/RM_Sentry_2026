"""Validate the static OCC/BEV/Topo interface shared by planning, tracking and simulation."""
from dataclasses import dataclass
import json
import math
from pathlib import Path
import struct

import yaml


@dataclass(frozen=True)
class MapProfile:
    """Resolved files and validated ROS parameters; loading never rewrites caller data."""
    directory: Path
    metadata: Path
    files: dict
    planner: dict
    tracker: dict
    width: int
    height: int

    # Return fresh dictionaries so launch-specific limits cannot mutate the shared profile.
    def parameters(self, node):
        values = dict(self.planner if node == 'trajectory_generation' else self.tracker)
        values.update({'maps.directory': str(self.directory),
                       **{'maps.' + key: name for key, name in self.files.items()}})
        if node == 'trajectory_generation':
            values.update({'maps.expected_width': self.width, 'maps.expected_height': self.height})
        return values


# Read only the PNG header; full image decoding remains the algorithms' responsibility.
def png_size(path):
    with Path(path).open('rb') as stream:
        header = stream.read(24)
    if len(header) != 24 or header[:8] != b'\x89PNG\r\n\x1a\n' or header[12:16] != b'IHDR':
        raise ValueError(f'Expected a PNG map: {path}')
    return struct.unpack('>II', header[16:24])


# Support the existing planning package and externally supplied profile directories equally.
def load_profile(path):
    root = Path(path).expanduser().resolve(strict=True)
    legacy = (root / 'config/map_metadata.yaml').is_file()
    directory = root / 'map' if legacy else root
    metadata = root / ('config/map_metadata.yaml' if legacy else 'map_metadata.yaml')
    manifest_path = directory / 'map_manifest.json'
    manifest = json.loads(manifest_path.read_text()) if manifest_path.is_file() else {}
    files = manifest.get('map_files', {'occupancy': 'occfinal.png', 'bev': 'bevfinal.png', 'topology': 'occtopo.png'})
    if set(files) != {'occupancy', 'bev', 'topology'}:
        raise ValueError('Map profile must name occupancy, bev and topology PNG files')
    for name in files.values():
        resolved = (directory / name).resolve(strict=True)
        if not resolved.is_relative_to(directory):
            raise ValueError('Map filenames must stay inside the profile directory')
    sizes = {png_size(directory / name) for name in files.values()}
    if len(sizes) != 1:
        raise ValueError('OCC, BEV and Topo image dimensions differ')
    width, height = sizes.pop()
    data = yaml.safe_load(metadata.read_text())
    planner = data['trajectory_generation']['ros__parameters']
    tracker = data['trajectory_tracking']['ros__parameters']
    pairs = {'map_resolution': 'resolution', 'map_x_size': 'x_size', 'map_y_size': 'y_size',
             'map_z_size': 'z_size', 'map_lower_x': 'lower_x', 'map_lower_y': 'lower_y',
             'map_lower_z': 'lower_z', 'robot_radius': 'robot_radius',
             'search_height_min': 'search_height_min', 'search_height_max': 'search_height_max',
             'search_radius': 'search_radius', 'height_bias': 'height_bias',
             'height_interval': 'height_interval', 'height_threshold': 'height_threshold',
             'height_second_high_threshold': 'height_second_high_threshold'}
    for left, right in pairs.items():
        a, b = float(planner['planner.' + left]), float(tracker['map.' + right])
        if not math.isfinite(a) or not math.isfinite(b) or not math.isclose(a, b, abs_tol=1e-9):
            raise ValueError(f'Planning/tracking map metadata disagree: {left}/{right}')
    resolution = planner['planner.map_resolution']
    if resolution <= 0 or planner['planner.robot_radius'] <= 0 or planner['planner.map_z_size'] <= 0:
        raise ValueError('Map resolution, radius and height must be positive')
    for axis, count in [('x', width), ('y', height)]:
        if not math.isclose(planner[f'planner.map_{axis}_size'], count * resolution, abs_tol=1e-8):
            raise ValueError(f'Map {axis} extent must equal image cells × resolution ({count * resolution})')
    if 'resolution' in manifest and not math.isclose(manifest['resolution'], resolution, abs_tol=1e-9):
        raise ValueError('Manifest and metadata resolutions disagree')
    if 'shape' in manifest and manifest['shape'] != [height, width]:
        raise ValueError('Manifest and image dimensions disagree')
    if 'lower' in manifest and manifest['lower'] != [planner['planner.map_lower_x'], planner['planner.map_lower_y']]:
        raise ValueError('Manifest and metadata origins disagree')
    return MapProfile(directory, metadata, dict(files), dict(planner), dict(tracker), width, height)
