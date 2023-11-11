from setuptools import setup
from glob import glob
import os

package_name = 'perceptions'

setup(
    name=package_name,
    version='0.0.0',
    # packages=[package_name],
    packages=[package_name],
    data_files=[
        # ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/**')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fishros',
    maintainer_email='fishros@foxmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "yolo=perceptions.yolo:main",
            "aruco_detect=perceptions.aruco_detect:main",
            "camera_test=perceptions.camera_test:main"
        ],
    },
)