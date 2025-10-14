from setuptools import setup
import os
from glob import glob

package_name = 'lela_drop_system'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sakazu01',
    maintainer_email='sakazu01@example.com',
    description='LELA Drop System',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_detector.py = lela_drop_system.color_detector:main',
            'mission_monitor.py = lela_drop_system.mission_monitor:main',
            'state_manager.py = lela_drop_system.state_manager:main',
            'servo_controller.py = lela_drop_system.servo_controller:main',
        ],
    },
)
