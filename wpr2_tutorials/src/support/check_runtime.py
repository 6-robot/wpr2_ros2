#!/usr/bin/env python3
"""Check installed runtime dependencies without opening hardware devices."""
import argparse
import importlib.util
import sys
from pathlib import Path
from ament_index_python.packages import PackageNotFoundError, get_package_prefix


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--group', choices=['all', 'base', 'lidar', 'camera', 'face', 'navigation', 'speech'],
                        default='all')
    group = parser.parse_args().group
    groups = {
        'base': ['wpr2_bringup', 'wpr2_description', 'robot_state_publisher', 'joy'],
        'lidar': ['sllidar_ros2'],
        'camera': ['orbbec_camera'],
        'face': ['cv_bridge'],
        'navigation': ['slam_toolbox', 'nav2_bringup', 'wp_map_tools'],
        'speech': ['wp_speech'],
    }
    missing = []
    for name, packages in groups.items():
        if group not in ('all', name):
            continue
        for package in packages:
            try:
                print('OK  ', package, get_package_prefix(package))
            except PackageNotFoundError:
                print('MISS', package)
                missing.append(package)
    if group in ('all', 'face'):
        for module in ['rclpy', 'cv2', 'face_recognition']:
            if importlib.util.find_spec(module) is None:
                print('MISS Python module:', module)
                missing.append(module)
            else:
                print('OK   Python module:', module)
    if group in ('all', 'speech'):
        for module in ['vosk', 'sherpa_onnx']:
            if importlib.util.find_spec(module) is None:
                print('MISS Python module:', module)
                missing.append(module)
            else:
                print('OK   Python module:', module)
        models = {
            'Vosk Chinese': (Path.home() / '.cache/vosk/vosk-model-small-cn-0.22',
                             ['am/final.mdl', 'conf/model.conf', 'conf/mfcc.conf']),
            'Vosk English': (Path.home() / '.cache/vosk/vosk-model-small-en-us-0.15',
                             ['am/final.mdl', 'conf/model.conf', 'conf/mfcc.conf']),
            'Melo TTS': (Path.home() / '.cache/sherpa-onnx/vits-melo-tts-zh_en',
                         ['model.onnx', 'tokens.txt', 'lexicon.txt', 'date.fst',
                          'number.fst', 'phone.fst', 'dict']),
        }
        for name, (directory, files) in models.items():
            complete = directory.is_dir() and all(
                (directory / f).is_dir() if f == 'dict' else
                (directory / f).is_file() and (directory / f).stat().st_size > 0
                for f in files)
            if complete:
                print('OK   model:', name, directory)
            else:
                print('MISS model:', name, directory)
                missing.append(name)
    if missing:
        print('Missing dependencies: ' + ', '.join(missing))
        print('See wpr2_tutorials/README.md for optional drivers and runtime dependencies. No hardware was started.')
    return 1 if missing else 0


if __name__ == '__main__':
    sys.exit(main())
