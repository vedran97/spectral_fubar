from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    world_file = PathJoinSubstitution(
        [FindPackageShare("spectral_fubar"),
        "worlds",
        "outdoor.world"]
    )

    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("husky_gazebo"),
        "launch",
        "gazebo.launch.py"],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("spectral_fubar"),
        "rviz",
        "default.rviz"]
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments={'world_path': world_file}.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    ld = LaunchDescription()
    ld.add_action(gazebo_sim)
    ld.add_action(rviz_node)

    return ld