import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohammed Al Jaf',
    maintainer_email='Mrmoha01@users.noreply.github.com',
    description='Colour + LiDAR object detection that places objects on the map.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector = amr_perception.object_detector:main',
            'yolo_detector = amr_perception.yolo_detector:main',
        ],
    },
)
