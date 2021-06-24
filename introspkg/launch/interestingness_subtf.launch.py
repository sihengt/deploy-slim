import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    bringup_dir = get_package_share_directory('interestingness_ros')

    # TODO: find out where to set use_sim_time
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default='true')

    # TODO: pass this to interesting_launch to interestingness_node
    image_topic_list = LaunchConfiguration('image_topic_list')
    declare_image_topic_list = DeclareLaunchArgument('image_topic_list', 
                                                     default='[/rs_front/color/image]')

    interestingness_launch = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    os.path.join(bringup_dir, "/launch/interestingness.launch.py")
                                )
                             )

    subtf_bags_launch = IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                    os.path.join(bringup_dir, "/launch/subtf_bags.launch.py")
                            )
                        )

    robot_launch = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                                    os.path.join(bringup_dir, "/launch/robot.launch.py")
                        )
                   )

    rviz_node = Node(package="rviz",
                executable="rviz",
                name="rviz",
                arguments=['-d', os.path.join(bringup_dir, '/rviz/subtf.rviz')]
                )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_image_topic_list)
    ld.add_action(interestingness_launch)
    ld.add_action(subtf_bags_launch)
    ld.add_action(robot_launch)
    ld.add_action(rviz_node)

    return ld