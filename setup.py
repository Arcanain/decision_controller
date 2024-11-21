from setuptools import setup
from setuptools import find_packages
from glob import glob
import os

package_name = "decision_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="your_email@example.com",
    description="A ROS2 package for decision control node",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
    'console_scripts': [
        'decision_control_node = decision_controller.decision_control_node:main'
        ],
    },
)
