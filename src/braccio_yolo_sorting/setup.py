from setuptools import setup
from glob import glob
import os

package_name = 'braccio_yolo_sorting'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='MoveIt-based YOLO sorting for Braccio',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector_node.py = braccio_yolo_sorting.yolo_detector_node:main',
            'braccio_moveit_sorting_controller.py = braccio_yolo_sorting.braccio_moveit_sorting_controller:main',
    
        ],
    },
)