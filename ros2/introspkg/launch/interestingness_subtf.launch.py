'''
Main launch file. Starts the other launch files and rviz2.
You can change the rviz2 config file path here.
'''

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    int_ros_share = get_package_share_directory('interestingness_ros')

    # TODO: find out where to set use_sim_time
    # use_sim_time = LaunchConfiguration('use_sim_time')
    # declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    image_topic_list = LaunchConfiguration('image_topic_list')
    declare_image_topic_list = DeclareLaunchArgument('image_topic_list', 
                                                     default_value='[/rs_front/color/image]')

    interestingness_launch = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    os.path.join(int_ros_share, "interestingness.launch.py"),
                                ),
                                launch_arguments={'image_topic_list':image_topic_list}.items()
                             )

    subtf_bags_launch = IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                    os.path.join(int_ros_share, "subtf_bags.launch.py")
                            )
                        )
 
    robot_launch = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                                    os.path.join(int_ros_share, "robot.launch.py")
                        )
                   )

    rviz_node = Node(package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    arguments=['-d', os.path.join(int_ros_share, 'subtf2.rviz')]
                )

    ld = LaunchDescription()
    # ld.add_action(declare_use_sim_time)
    ld.add_action(declare_image_topic_list)
    ld.add_action(interestingness_launch)
    ld.add_action(subtf_bags_launch)
    # ld.add_action(robot_launch) # husky_gazebo is not in ROS2 yet
    ld.add_action(rviz_node)

    return ld