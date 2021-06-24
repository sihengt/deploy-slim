import os

# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    int_dir = ('/root/ros2_ws/src/interestingness_ros/interestingness')
    model_path = os.path.join(int_dir, 'saves/vgg16.pt.SubTF.n100usage.mse')

    interestingness_node = Node(package="interestingness_ros",
                                executable="interestingness_node",
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
                               executable="interest_marker_node",
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

