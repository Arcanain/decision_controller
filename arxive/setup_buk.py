from setuptools import setup
from setuptools import find_packages
import os
from glob import glob

package_name = "decision_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=['arxive']),
    py_modules=[
        'python_programs.decision_control_node',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),  # launchファイルを含める
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="your_email@example.com",
    description="A ROS2 package for decision control node",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "decision_control_node = python_programs.decision_control_node:main"
        ],
    },
)
