import launch, xacro, os
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    show_gui = LaunchConfiguration('gui')
    
    share_dir = get_package_share_directory('exo_urdf')
    xacro_file = os.path.join(share_dir, 'urdf', 'exo.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_state_pub_node = Node(
        package ='robot_state_publisher',
        executable ='robot_state_publisher',
        name='robot_state_publisher',
        parameters= [
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[robot_urdf]
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{
            'robot_description': robot_urdf, 
            'update_rate': 100,
            'use_sim_time': True
        }],
        emulate_tty=True,
    )

    gazebo_node = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'),
        launch_arguments={'world': 'empty.world'}.items()
    )
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'exoskeleton', '-topic', 'robot_description'],
        output='screen',
        emulate_tty=True,
    )

    # 返回启动描述
    return launch.LaunchDescription([
        # gui_arg,
        # gazebo_node,
        robot_state_pub_node,
        joint_state_publisher,
        # controller_manager,
        # spawn_entity,

    ])
