from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_path

    
def generate_launch_description():

    # urdf_path = PathJoinSubstitution([
    #     get_package_share_path('exo_urdf'),  # Finds your package
    #     'urdf',                              # Subfolder
    #     'exo.urdf'                           # Filename
    # ])

    urdf_path = str(get_package_share_path('exo_urdf')) + '/urdf/exo.xacro'
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', str(urdf_path)]),
                    value_type=str
                )
            }]
        ),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'exoskeleton_leg', 
                      '-topic', 'robot_description',
                      '-z', '0.5'],  # Spawn above ground
            output='screen'
        )
    ])