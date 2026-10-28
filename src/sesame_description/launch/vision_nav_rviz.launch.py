import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('sesame_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'sesame_ros2.urdf')
    rviz_config_path = os.path.join(pkg_share, 'config', 'vision_nav.rviz')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher reads /joint_states and publishes internal link TFs
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # RViz2 configured with Fixed Frame: map, Planned Path, Obstacle Markers, and RobotModel
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node
    ])
