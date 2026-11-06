"""Static checks that need no ROS install: XML/YAML well-formedness, xacro expansion, syntax."""

from pathlib import Path
import py_compile
import sys
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parent.parent
errors = []


def check(path, func):
    try:
        func(path)
    except Exception as exc:  # noqa: BLE001 - report every problem, not just the first
        errors.append(f'{path.relative_to(ROOT)}: {exc}')


def check_xml(path):
    ET.parse(path)


def check_yaml(path):
    with open(path) as f:
        yaml.safe_load(f)


def check_py(path):
    py_compile.compile(str(path), doraise=True)


def check_xacro(path):
    import xacro
    doc = xacro.process_file(str(path))
    root = ET.fromstring(doc.toxml())
    links = {e.get('name') for e in root.iter('link')}
    joints = {e.get('name') for e in root.iter('joint')}
    for need in ('base_footprint', 'base_link', 'lidar_link', 'camera_link',
                 'camera_optical_link', 'imu_link'):
        if need not in links:
            raise ValueError(f'missing link {need}')
    for need in ('left_wheel_joint', 'right_wheel_joint'):
        if need not in joints:
            raise ValueError(f'missing joint {need}')
    # every joint parent/child must exist
    for joint in root.iter('joint'):
        for tag in ('parent', 'child'):
            if joint.find(tag).get('link') not in links:
                raise ValueError(f'joint {joint.get("name")} references unknown link')


for p in ROOT.rglob('package.xml'):
    check(p, check_xml)
for p in ROOT.rglob('*.sdf'):
    check(p, check_xml)
for p in ROOT.rglob('*.yaml'):
    if '.github' not in p.parts:
        check(p, check_yaml)
for p in ROOT.rglob('*.rviz'):
    check(p, check_yaml)
for p in ROOT.rglob('*.py'):
    check(p, check_py)
for p in ROOT.rglob('*.xacro'):
    check(p, check_xacro)

if errors:
    print('\n'.join(errors))
    sys.exit(1)
print('All static checks passed.')
