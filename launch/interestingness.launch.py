import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    model_dir = get_package_share_directory('interestingness_ros')
    model_path = os.path.join(model_dir, 'saves/vgg16.pt.SubTF.n100usage.mse')

    interestingness_node = Node(package="interestingness_ros",
                                executable="interestingness_node.py",
                                name="interestingness_node",
                                output="screen",
                                parameters=[
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
                               executable="interest_marker.py",
                               name="interestmarker_node",
                               output="log",
                               # remappings=[
                               #      ("/interestingness/info", "/interestingness/info")],
                               parameters=[
                                    {"min-level":"0.2"}]
                          )

    ld = LaunchDescription()
    ld.add_action(interestingness_node)
    ld.add_action(interestmarker_node)

    return ld

