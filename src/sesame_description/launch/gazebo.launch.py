import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('sesame_description')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    urdf_path = os.path.join(pkg_share, 'urdf', 'sesame_gazebo.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'chartpaper_arena.world')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Ensure Gazebo model path includes the workspace share directory and ~/.gazebo/models
    models_parent = os.path.abspath(os.path.join(pkg_share, '..'))
    user_models = os.path.expanduser('~/.gazebo/models')
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_path = f"{models_parent}:{user_models}:{existing_model_path}"

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path
    )

    # 1. Gazebo Server & Client (loads chartpaper_arena.world which includes Sesame robot directly)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path, 'verbose': 'true'}.items()
    )

    # 2. Robot State Publisher (publishes TF tree and robot_description topic)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    return LaunchDescription([
        set_model_path,
        gazebo,
        robot_state_publisher
    ])
