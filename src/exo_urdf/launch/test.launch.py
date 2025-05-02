from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    urdf_path = PathJoinSubstitution([
        get_package_share_directory('exo_urdf'),
        'urdf',
        'exo.urdf.xacro'
    ])

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            {'update_rate': 100},
            {'use_sim_time': True}
        ],
        output='screen',
        emulate_tty=True,
    )

    # Use ExecuteProcess to run the standalone Gazebo Fortress executable
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-topic', 'robot_description'],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        controller_manager,
        gazebo,
        spawn_entity,
    ])
