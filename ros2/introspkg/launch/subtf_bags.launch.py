'''
TODO: Find a way to access the launch arguments to change datalocation
TODO: add the remaining rosbags locations in
TODO: test if bags play properly in sequence
topics in bag according to ros2 bag info:
    /rs_front/color/camera_info
    /rs_front/color/image
    /rs_left/color/camera_info
    /rs_left/color/image
    /rs_right/color/camera_info
    /rs_right/color/image
    /tf
    /velodyne_cloud_registered  
'''

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # use_sim_time = LaunchConfiguration('use_sim_time')
    # declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default='true')

    # datalocation = LaunchConfiguration('datalocation')
    # declare_datalocation = DeclareLaunchArgument('datalocation', default_value='/root/ros2_ws/src/interestingness_ros/bags', description='dir containing ros2 bags')
    datalocation = '/root/ros2_ws/src/interestingness_ros/bags'

    SubT0 = [
        '0817-ugv0-tunnel0/objdet_2019-08-17-16-32-27_0',
        '0817-ugv0-tunnel0/objdet_2019-08-17-16-34-13_1',
        '0817-ugv0-tunnel0/objdet_2019-08-17-16-35-56_2',
        '0817-ugv0-tunnel0/objdet_2019-08-17-16-37-39_3',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-16-39-21_4',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-16-41-03_5',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-16-42-46_6',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-16-44-30_7',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-16-46-13_8',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-16-47-55_9',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-16-49-37_10',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-16-51-18_11',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-16-52-53_12',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-16-54-25_13',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-16-55-47_14',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-16-57-05_15',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-16-58-39_16',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-16-59-54_17',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-17-01-14_18',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-17-02-57_19',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-17-04-28_20',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-17-05-42_21',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-17-06-59_22',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-17-08-15_23',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-17-09-31_24',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-17-10-46_25',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-17-12-11_26',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-17-13-55_27',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-17-15-39_28',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-17-17-23_29',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-17-19-06_30',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-17-20-56_31',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-17-22-46_32',
        # '0817-ugv0-tunnel0/objdet_2019-08-17-17-24-36_33',
    ]

    SubT1 = [
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-24-41_0',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-26-13_1',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-27-44_2',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-29-14_3',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-30-43_4',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-32-12_5',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-33-42_6',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-35-11_7',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-36-38_8',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-38-07_9',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-39-36_10',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-41-06_11',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-42-35_12',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-44-05_13',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-45-33_14',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-46-48_15',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-48-19_16',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-49-50_17',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-51-11_18',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-52-31_19',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-54-02_20',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-55-31_21',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-56-51_22',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-58-14_23',
        '0817-ugv1-tunnel0/objdet_2019-08-17-16-59-16_24',
        '0817-ugv1-tunnel0/objdet_2019-08-17-17-00-24_25',
        '0817-ugv1-tunnel0/objdet_2019-08-17-17-01-29_26',
        '0817-ugv1-tunnel0/objdet_2019-08-17-17-02-29_27',
        '0817-ugv1-tunnel0/objdet_2019-08-17-17-04-00_28',
        '0817-ugv1-tunnel0/objdet_2019-08-17-17-05-32_29',
        '0817-ugv1-tunnel0/objdet_2019-08-17-17-07-03_30',
        '0817-ugv1-tunnel0/objdet_2019-08-17-17-08-34_31',
        '0817-ugv1-tunnel0/objdet_2019-08-17-17-10-03_32',
        '0817-ugv1-tunnel0/objdet_2019-08-17-17-11-33_33',
        '0817-ugv1-tunnel0/objdet_2019-08-17-17-13-05_34',
        '0817-ugv1-tunnel0/objdet_2019-08-17-17-14-36_35',
        '0817-ugv1-tunnel0/objdet_2019-08-17-17-16-08_36',
        '0817-ugv1-tunnel0/objdet_2019-08-17-17-17-39_37',
        '0817-ugv1-tunnel0/objdet_2019-08-17-17-19-11_38',
    ]

    ld = LaunchDescription()

    for bag in SubT0:
        rosbag_process = ExecuteProcess(cmd=['ros2', 'bag', 'play', f'{bag}', '--rate', '3'], log_cmd=True) # Note: --clock is not working. rate: Rate of msg playback
        ld.add_action(rosbag_process)

    # ld.add_action(declare_use_sim_time)
    # ld.add_action(declare_datalocation)

    return ld

