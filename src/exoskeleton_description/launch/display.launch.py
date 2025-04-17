from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch argument for the URDF file
    urdf_file = PathJoinSubstitution([
        FindPackageShare('exoskeleton_description'),
        'urdf',
        'exo.urdf'
    ])
    
    robot_description_content = Command(['xacro ', urdf_file])
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            name='urdf_file',
            default_value=urdf_file,
            description='Path to URDF file'
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    robot_description_content,
                    value_type=str
                ),
                'use_sim_time': False
            }]
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{
                'robot_description': ParameterValue(
                    robot_description_content,
                    value_type=str
                )
            }]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('exoskeleton_description'),
                'rviz',
                'urdf_robot.rviz'
            ])],
            parameters=[{
                'use_sim_time': False
            }]
        )
    ])