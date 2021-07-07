'''
Launches 2 nodes which publishes image and interest marker.
You may change the model save path and image topic name here
'''

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    int_dir = ('/root/ros2_ws/src/interestingness_ros/interestingness')
    model_path = os.path.join(int_dir, 'saves/vgg16.pt.SubTF.n100usage.mse')

    image_topic_list = LaunchConfiguration('image_topic_list')
    declare_image_topic_list = DeclareLaunchArgument('image_topic_list', 
                                                     default_value='[/rs_front/color/image]')

    interestingness_node = Node(package="interestingness_ros",
                                executable="interestingness_node",
                                name="interestingness_node",
                                output="screen",
                                parameters=[
                                    {"image-topic":image_topic_list},
                                    {"data-root":"/data/datasets"},
                                    {"model-save":model_path},
                                    {"crop-size":320},
                                    {"num-interest":10},
                                    {"skip-frames":1},
                                    {"window-size":1},
                                    {"save-flag":"test"},
                                    {"rr":5},
                                    {"wr":5},
                                ]
                           )

    interestmarker_node = Node(package="interestingness_ros",
                               executable="interest_marker_node",
                               name="interestmarker_node",
                               output="log",
                               # remappings=[
                               #      ("/interestingness/info", "/interestingness/info")],
                               parameters=[
                                    {"min-level":0.2}]
                          )

    ld = LaunchDescription()
    ld.add_action(declare_image_topic_list)
    ld.add_action(interestingness_node)
    ld.add_action(interestmarker_node)

    return ld

