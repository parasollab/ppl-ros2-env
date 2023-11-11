# Copyright 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from glob import glob

from setuptools import setup
import os

package_name = "ur_demo"

setup(
    name=package_name,
    version="3.16.0",
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/**')),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Scott Lee",
    author_email="sl148@illinois.edu",
    maintainer="Scott Lee",
    maintainer_email="sl148@illinois.edu",
    keywords=["ROS"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="Demo nodes for showing and testing functionalities of ros2_control framework.",
    long_description="""\
Demo nodes for showing and testing functionalities of the ros2_control framework.""",
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "publisher_joint_trajectory_controller = \
                ur_demo.publisher_joint_trajectory_controller:main",
            "publisher_ik = \
                ur_demo.publisher_ik:main",
            "publisher_gpg = \
                ur_demo.publisher_gpg:main",
            "pick_and_place = \
                ur_demo.pick_and_place:main",
            "handoff_demo = ur_demo.handoff_demo:main",
        ],
    },
)
