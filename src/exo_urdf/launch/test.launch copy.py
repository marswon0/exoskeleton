from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_path, get_package_share_directory

    
def generate_launch_description():

    # urdf_path = PathJoinSubstitution([
    #     get_package_share_path('exo_urdf'),  # Finds your package
    #     'urdf',                              # Subfolder
    #     'exo.urdf'                           # Filename
    # ])

    urdf_path = str(get_package_share_path('exo_urdf')) + '/urdf/exo.urdf.xacro'
    
    robot_description = {
                'robot_description': ParameterValue(
                    Command(['xacro ', str(urdf_path)]),
                    value_type=str
                )
    }
    
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{
            'robot_description': robot_description, 
            'update_rate': 100,
            'use_sim_time': True
        }],
        emulate_tty=True,
    )


    joint_state_publisher = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[robot_description]
    )

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description]
    )

    gazebo_spawn_srv = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'exoskeleton_leg', 
                      '-topic', 'robot_description',
                      '-z', '0.5'],  # Spawn above ground
            output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        controller_manager,
        gazebo_spawn_srv
    ])