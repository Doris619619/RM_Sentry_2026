#!/usr/bin/env python
"""ROS1 copy of the deterministic largest-free-component baseline selector."""

from __future__ import print_function

import argparse
import json
import os
from collections import deque

from PIL import Image


RESOLUTION = 0.05
LOWER_X = -13.394
LOWER_Y = -12.079
ROBOT_RADIUS = 0.35


def _load_gray(path):
    image = Image.open(path).convert('L')
    if image.size != (400, 400):
        raise ValueError('expected 400x400 map, got {}'.format(image.size))
    return image


def _largest_component(occ, topo):
    width, height = occ.size
    inflation = int(ROBOT_RADIUS / RESOLUTION)
    occupied = [[occ.getpixel((column, row)) > 10 for column in range(width)] for row in range(height)]
    inflated = [row[:] for row in occupied]
    offsets = [(dr, dc) for dr in range(-inflation, inflation + 1)
               for dc in range(-inflation, inflation + 1)
               if dr * dr + dc * dc <= inflation * inflation]
    for row in range(height):
        for column in range(width):
            if not occupied[row][column]:
                continue
            for dr, dc in offsets:
                next_row, next_column = row + dr, column + dc
                if 0 <= next_row < height and 0 <= next_column < width:
                    inflated[next_row][next_column] = True
    candidates = set()
    for row in range(height):
        for column in range(width):
            if not inflated[row][column] and topo.getpixel((column, row)) > 10:
                candidates.add((row, column))
    if not candidates:
        raise RuntimeError('no free occtopo cells after inflation')
    remaining = set(candidates)
    largest = []
    while remaining:
        root = min(remaining)
        remaining.remove(root)
        queue = deque([root])
        component = [root]
        while queue:
            row, column = queue.popleft()
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    neighbour = (row + dr, column + dc)
                    if neighbour in remaining:
                        remaining.remove(neighbour)
                        queue.append(neighbour)
                        component.append(neighbour)
        if len(component) > len(largest):
            largest = component
    return set(largest), inflation


def _farthest(component, source):
    queue = deque([source])
    distance = {source: 0}
    best = source
    while queue:
        current = queue.popleft()
        if distance[current] > distance[best] or (distance[current] == distance[best] and current < best):
            best = current
        row, column = current
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                neighbour = (row + dr, column + dc)
                if neighbour in component and neighbour not in distance:
                    distance[neighbour] = distance[current] + 1
                    queue.append(neighbour)
    return best, distance[best]


def _coord(cell):
    row, column = cell
    return [round((column + 0.5) * RESOLUTION + LOWER_X, 12),
            round((400 - row - 0.5) * RESOLUTION + LOWER_Y, 12)]


def select_points(map_directory):
    occ = _load_gray(os.path.join(map_directory, 'occfinal.png'))
    topo = _load_gray(os.path.join(map_directory, 'occtopo.png'))
    component, inflation = _largest_component(occ, topo)
    first, _ = _farthest(component, min(component))
    second, separation = _farthest(component, first)
    return {'algorithm': 'largest_8_connected_inflated_occtopo_two_bfs',
            'component_size': len(component), 'inflation_cells': inflation,
            'start': _coord(first), 'goal': _coord(second),
            'graph_separation_cells': separation}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map-dir', required=True)
    parser.add_argument('--output')
    arguments = parser.parse_args()
    result = select_points(arguments.map_dir)
    if arguments.output:
        with open(arguments.output, 'w') as output:
            json.dump(result, output, indent=2, sort_keys=True)
            output.write('\n')
    else:
        print(json.dumps(result, sort_keys=True))


if __name__ == '__main__':
    main()
