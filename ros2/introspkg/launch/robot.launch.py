'''
Starts 5 tf2 broadcaster nodes
Starts husky launch file
TODO: find husky_gazebo alternative implementation in ros2 
'''

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # bringup_dir = get_package_share_directory('husky_gazebo')
    # launch_path = os.path.join(bringup_dir, 'launch/spawn_husky.launch')

    node1 = Node(package="tf2_ros",
                executable="static_transform_publisher",
                name="robot_link",
                arguments=['0', '0', '-0.7', '0', '0', '0', '1', 'velodyne', 'base_link'],
                output='log',
            )

    node2 = Node(package="tf2_ros",
                executable="static_transform_publisher",
                name="front_left_wheel_link",
                arguments=['0.25', '-0.25', '0', '0', '0', '0', '1', 'base_link', 'front_left_wheel_link'],
                output='log',
            )

    node3 = Node(package="tf2_ros",
                executable="static_transform_publisher",
                name="front_right_wheel_link",
                arguments=['0.25', '0.25', '0', '0', '0', '0', '1', 'base_link', 'front_right_wheel_link'],
                output='log',
            )

    node4 = Node(package="tf2_ros",
                executable="static_transform_publisher",
                name="rear_left_wheel_link",
                arguments=['-0.25', '-0.25', '0', '0', '0', '0', '1', 'base_link', 'rear_left_wheel_link'],
                output='log',
            )

    node5 = Node(package="tf2_ros",
                executable="static_transform_publisher",
                name="rear_right_wheel_link",
                arguments=['-0.25', '0.25', '0', '0', '0', '0', '1', 'base_link', 'rear_right_wheel_link'],
                output='log',
            )
    # TODO: find out where to set use_sim_time 
    # use_sim_time = LaunchConfiguration('use_sim_time')
    # declare_use_sim_time = DeclareLaunchArgument('use_sim_time',
    #                                              default_value='true')
    
    # Create launch config variables and declare them 
    # Note: kinect is not supported anymore
    laser_enabled = LaunchConfiguration('laser_enabled')
    declare_laser_enabled = DeclareLaunchArgument('laser_enabled',
                                                   default_value='true')
    # Include husky launch file
    # husky_launch = IncludeLaunchDescription(
    #                     PythonLaunchDescriptionSource(launch_path),
    #                     launch_arguments={'laser_enabled': laser_enabled
    #                     }.items()
    #                 )

    
    ld = LaunchDescription()
    # Add tf2 broadcaster nodes
    ld.add_action(node1)
    ld.add_action(node2)
    ld.add_action(node3)
    ld.add_action(node4)
    ld.add_action(node5)
    
    # Declare the launch options
    # ld.add_action(declare_use_sim_time)
    ld.add_action(declare_laser_enabled)
    # Add launchfile to run husky nodes
    # ld.add_action(husky_launch)

    return ld