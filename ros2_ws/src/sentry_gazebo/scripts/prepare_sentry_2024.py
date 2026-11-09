#!/usr/bin/env python3
"""Expand and verify the pinned upstream RMUC2024 Gazebo arena model."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from PIL import Image
import yaml


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def git_blob_sha(path):
    data = path.read_bytes()
    return hashlib.sha1(f'blob {len(data)}\0'.encode() + data).hexdigest()


def prepare(asset_dir):
    package = Path(__file__).resolve().parents[1]
    profile = package / 'maps/sentry_2024'
    sources = json.loads((profile / 'sources.json').read_text())
    scene = yaml.safe_load((profile / 'scene.yaml').read_text())
    metadata = yaml.safe_load((profile / 'map_metadata.yaml').read_text())

    expected_shape = None
    for key in ('occupancy', 'bev', 'topology'):
        name = scene['map_files'][key]
        image_path = profile / name
        with Image.open(image_path) as image:
            if expected_shape is None:
                expected_shape = image.size
            elif image.size != expected_shape:
                raise ValueError(f'Upstream map dimensions differ: {name} has {image.size}, expected {expected_shape}')
        source = sources['files'][name]
        if git_blob_sha(image_path) != source['git_blob_sha']:
            raise ValueError(f'Upstream map bytes changed: {name}')
    if expected_shape != (301, 161):
        raise ValueError(f'Unexpected upstream map dimensions: {expected_shape}')
    if metadata['trajectory_generation']['ros__parameters']['planner.map_resolution'] != 0.1:
        raise ValueError('Unexpected upstream map resolution')

    compressed = profile / 'source/RMUC2024_field_base_link.STL.gz'
    source_meta = sources['files']['source/RMUC2024_field_base_link.STL.gz']
    if sha256(compressed) != source_meta['compressed_sha256']:
        raise ValueError('Compressed upstream arena source changed')
    output = Path(asset_dir) / 'models/sentry_2024_arena'
    mesh = output / 'meshes/base_link.STL'
    mesh.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(compressed, 'rb') as source:
        original = source.read()
    if hashlib.sha256(original).hexdigest() != source_meta['sha256']:
        raise ValueError('Decompressed RMUC2024 arena SHA-256 does not match the pinned source')
    if hashlib.sha1(f'blob {len(original)}\0'.encode() + original).hexdigest() != source_meta['git_blob_sha']:
        raise ValueError('Decompressed RMUC2024 arena does not match its pinned upstream Git blob')
    triangle_count = int.from_bytes(original[80:84], 'little')
    if len(original) != 84 + triangle_count * 50:
        raise ValueError('Upstream arena mesh has an unexpected binary STL layout')
    mesh.write_bytes(original)

    (output / 'model.config').write_text(
        '<?xml version="1.0"?>\n'
        '<model><name>sentry_2024_arena</name><version>1.0</version>'
        '<sdf version="1.8">model.sdf</sdf></model>\n')
    (output / 'model.sdf').write_text(
        '<?xml version="1.0"?>\n'
        '<sdf version="1.8"><model name="sentry_2024_arena"><static>true</static>'
        '<link name="terrain"><collision name="terrain_collision"><geometry><mesh>'
        '<uri>meshes/base_link.STL</uri></mesh></geometry></collision>'
        '<visual name="terrain_visual"><geometry><mesh><uri>meshes/base_link.STL</uri>'
        '</mesh></geometry><material><ambient>0.38 0.43 0.48 1</ambient>'
        '<diffuse>0.48 0.54 0.59 1</diffuse></material></visual></link></model></sdf>\n')

    visual = Path(asset_dir) / 'models/sentry_2024_arena_visual'
    visual.mkdir(parents=True, exist_ok=True)
    (visual / 'model.config').write_text(
        '<?xml version="1.0"?>\n'
        '<model><name>sentry_2024_arena_visual</name><version>1.0</version>'
        '<sdf version="1.8">model.sdf</sdf></model>\n')
    (visual / 'model.sdf').write_text(
        '<?xml version="1.0"?>\n'
        '<sdf version="1.8"><model name="sentry_2024_arena_visual"><static>true</static>'
        '<link name="terrain"><visual name="terrain_visual"><geometry><mesh>'
        '<uri>model://sentry_2024_arena/meshes/base_link.STL</uri>'
        '</mesh></geometry><material><ambient>0.38 0.43 0.48 1</ambient>'
        '<diffuse>0.48 0.54 0.59 1</diffuse></material></visual></link></model></sdf>\n')

    worlds = Path(asset_dir) / 'worlds'
    worlds.mkdir(parents=True, exist_ok=True)
    for physical in (False, True):
        template = package / 'worlds' / ('planning_physics.sdf' if physical else 'planning.sdf')
        tree = ET.parse(template)
        world = tree.getroot().find('world')
        world.set('name', 'sentry_2024')
        for include in list(world.findall('include')):
            uri = include.findtext('uri', '')
            if 'planning_map' in uri:
                world.remove(include)
            elif include.findtext('name') == 'sentry':
                x, y, heading = scene['spawn']
                z = 0.1 if physical else 0.0
                include.find('pose').text = f'{x} {y} {z} 0 0 {heading}'
        arena = ET.Element('include')
        ET.SubElement(arena, 'uri').text = 'model://sentry_2024_arena' if physical else 'model://sentry_2024_arena_visual'
        ET.SubElement(arena, 'name').text = 'RMUC2024_arena'
        world.append(arena)
        ground = world.find("model[@name='ground']")
        if ground is not None:
            ground.find('pose').text = '15.0 8.0 -0.05 0 0 0' if physical else '15.0 8.0 -0.03 0 0 0'
            for size in ground.findall('.//box/size'):
                size.text = '32 20 0.1' if physical else '32 20 0.05'
        camera = world.find("gui/plugin[@filename='MinimalScene']/camera_pose")
        if camera is not None:
            # Place the orbit camera far enough away to show the whole 30x16 m field.
            camera.text = '15 -24 30 0 0.75 1.57079632679'
        ET.indent(tree)
        tree.write(worlds / ('sentry_2024_physics.sdf' if physical else 'sentry_2024.sdf'),
                   encoding='unicode', xml_declaration=True)

    config = Path(asset_dir) / 'config'
    config.mkdir(parents=True, exist_ok=True)
    rviz = yaml.safe_load((package / 'config/autonomy.rviz').read_text())
    view = rviz['Visualization Manager']['Views']['Current']
    view['Distance'] = 42
    view['Pitch'] = 0.95
    view['Yaw'] = 1.57
    view['Focal Point'] = {'X': 15.0, 'Y': 8.0, 'Z': 0.0}
    rviz['Visualization Manager']['Displays'].append({
        'Class': 'rviz_default_plugins/Marker',
        'Name': 'Upstream RMUC2024 topology skeleton',
        'Enabled': True,
        'Topic': {'Value': '/sim/road_topology', 'Durability Policy': 'Transient Local',
                  'Reliability Policy': 'Reliable'},
    })
    (config / 'sentry_2024.rviz').write_text(yaml.safe_dump(rviz, sort_keys=False))

    result = {
        'source_commit': sources['commit'],
        'map_shape_width_height': list(expected_shape),
        'arena_mesh_git_blob_sha': git_blob_sha(mesh),
        'arena_mesh_sha256': sha256(mesh),
        'arena_mesh_bytes': mesh.stat().st_size,
        'arena_triangles': triangle_count,
        'generated_worlds': [str(worlds / 'sentry_2024.sdf'), str(worlds / 'sentry_2024_physics.sdf')],
        'asset_directory': str(Path(asset_dir).resolve()),
    }
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--asset-dir', type=Path, required=True)
    args = parser.parse_args()
    prepare(args.asset_dir)
