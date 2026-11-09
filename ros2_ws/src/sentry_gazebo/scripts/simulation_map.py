"""Locate the map selected by the simulation launch for all Python consumers."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def map_paths():
    profile = os.environ.get('SENTRY_SIM_MAP')
    if profile:
        root = Path(profile)
        return root, root, root / 'map_metadata.yaml', root / 'map_manifest.json'
    root = Path(get_package_share_directory('trajectory_generation'))
    manifest = Path(get_package_share_directory('sentry_gazebo')) / 'config/map_manifest.json'
    return root, root / 'map', root / 'config/map_metadata.yaml', manifest


def map_filenames(maps):
    """Return occupancy, BEV and skeleton filenames for the selected map profile."""
    manifest = Path(maps) / 'map_manifest.json'
    if manifest.is_file():
        import json
        return json.loads(manifest.read_text()).get('map_files', {
            'occupancy': 'occfinal.png', 'bev': 'bevfinal.png', 'topology': 'occtopo.png'})
    return {'occupancy': 'occfinal.png', 'bev': 'bevfinal.png', 'topology': 'occtopo.png'}
