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
    
    robot_description = {
                'robot_description': ParameterValue(
                    Command(['xacro ', str(urdf_path)]),
                    value_type=str
                ),
                
            }

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'update_rate' : 200}],
        emulate_tty=True,
    )

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description]
    )

    gazebo_spawn = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'exoskeleton_leg', 
                      '-topic', 'robot_description',
                      '-z', '0.5'],  # Spawn above ground
            output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        gazebo_spawn
    ])