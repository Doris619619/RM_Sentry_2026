#!/usr/bin/env python3
"""Check the saved deterministic ROS1/ROS2 Planning baseline artifacts."""

import argparse
import json
import math


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('ros1')
    parser.add_argument('ros2')
    arguments = parser.parse_args()
    with open(arguments.ros1, encoding='utf-8') as source:
        ros1 = json.load(source)
    with open(arguments.ros2, encoding='utf-8') as source:
        ros2 = json.load(source)
    assert ros1['selection'] == ros2['selection'], 'ROS1/ROS2 selected different free-space pair'
    assert ros1['selection']['component_size'] > 0
    assert ros1['selection']['graph_separation_cells'] > 0
    assert ros1['segments'] > 0 and ros2['segments'] > 0
    assert all(math.isfinite(value) for value in ros1['end'] + ros2['end'])
    end_error = math.hypot(ros1['end'][0] - ros2['end'][0], ros1['end'][1] - ros2['end'][1])
    length_error = abs(ros1['length'] - ros2['length']) / ros1['length']
    time_error = abs(ros1['total_time'] - ros2['total_time']) / ros1['total_time']
    assert end_error <= 0.10, end_error
    assert length_error <= 0.02, length_error
    assert time_error <= 0.02, time_error
    print('PASS end_error={:.9f} length_error={:.9f} time_error={:.9f}'.format(
        end_error, length_error, time_error))


if __name__ == '__main__':
    main()
