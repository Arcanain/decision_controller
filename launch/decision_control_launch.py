import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'decision_controller'

    # decision_control_node ノードの設定
    decision_control_node = Node(
        package=package_name,
        executable='decision_control_node',
        output="screen",
        )

    return LaunchDescription([decision_control_node])