import os

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    # Check if we're told to use sim time
    # use_sim_time = LaunchConfiguration('use_sim_time')
    # use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Process the URDF file
    pkg_share = FindPackageShare(package='lawnbot_description').find('lawnbot_description')
    default_model_path = os.path.join(pkg_share, 'src', 'description', 'lawnbot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')

    # Create a robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, 
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    # {'use_ros2_control': use_ros2_control}
                    ],
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig'),{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        # DeclareLaunchArgument(
        #     'use_ros2_control',
        #     default_value='true',
        #     description='Use ros2_control if true'),
        DeclareLaunchArgument(
            name='rvizconfig', 
            default_value=default_rviz_config_path, 
            description='Absolute path to rviz config file'),
        DeclareLaunchArgument(
            name='model', 
            default_value=default_model_path, 
            description='Absolute path to robot model file'),

        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,

    ])