'''
TODO: Test node and process, formatting of rosbag args in ROS2.
TODO: add the remaining rosbags locations in.
'''


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default='true')

    datalocation = LaunchConfiguration('datalocation')
    declare_datalocation = DeclareLaunchArgument('datalocation', default='/data/datasets')

    # SubT0 = f"
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-16-32-27_0.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-16-34-13_1.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-16-35-56_2.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-16-37-39_3.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-16-39-21_4.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-16-41-03_5.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-16-42-46_6.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-16-44-30_7.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-16-46-13_8.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-16-47-55_9.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-16-49-37_10.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-16-51-18_11.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-16-52-53_12.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-16-54-25_13.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-16-55-47_14.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-16-57-05_15.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-16-58-39_16.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-16-59-54_17.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-17-01-14_18.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-17-02-57_19.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-17-04-28_20.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-17-05-42_21.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-17-06-59_22.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-17-08-15_23.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-17-09-31_24.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-17-10-46_25.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-17-12-11_26.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-17-13-55_27.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-17-15-39_28.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-17-17-23_29.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-17-19-06_30.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-17-20-56_31.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-17-22-46_32.bag
    # {datalocation}/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-17-24-36_33.bag"

    rosbag_node = Node(package="bag",
                        executable="play",
                        name="rosbag",
                        arguments=['--clock', '-r', '3', SubT1],
                        parameters=[
                            {"use_sim_time":use_sim_time},
                        ]
                  )

    rosbag = ExecuteProcess(cmd=[SubT1, 'ros2', 'bag', '--clock', '1', '--rate', '3']) # clock: publishes to /clock at 1hz. rate: Rate of msg playback

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_datalocation)
    ld.add_action(rosbag_node)

    return ld

