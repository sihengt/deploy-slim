# docker build -f ros2-foxy.dockerfile -t cmu-foxy:1.0 .
# start container with run2.sh

FROM osrf/ros:foxy-desktop
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
  vim \
  tmux \
  wget \
  python3-pip \
  python3-yaml

RUN python3 -m pip install -U \
  setuptools \
  pip \
  opencv-python \
  torch \
  torchvision \
  matplotlib 

COPY ./configs/ /root/

## Set up bashrc
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
RUN echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc # exists after the first colcon build
RUN ln -s /opt/ros/foxy/lib/libconsole_bridge.so /opt/ros/foxy/lib/libconsole_bridge.so.1.0 #required for cpp example node to run; ros2 run demo_nodes_cpp talker

## Set up ros2 workspace 
RUN mkdir -p ~/ros2_ws/src
WORKDIR /root/ros2_ws/src

## Install cvbridge
RUN git clone https://github.com/ros-perception/vision_opencv.git
RUN cd vision_opencv && git checkout ros2 # colcon build --package-select cv_bridge

## Start container, mount packages to src/ and colcon build the 3 packages.


## For new package, Set up node package. Copy out package and mount subsequently.
# RUN source /opt/ros/foxy/setup.bash && ros2 pkg create interestingness_ros --build-type ament_python --dependencies rclpy visualization_msgs sensor_msgs cv_bridge  
# RUN cd interestingness_ros && git clone -b master --single-branch https://github.com/wang-chen/interestingness.git
# edit setup.py in python package: import os from glob import glob 
# in data_files: (os.path.join('share', package_name), glob('launch/*.launch.py'))
# in console_scipts: "interestingness_node = interestingness_ros.interestingness_node:main","interest_marker_node = interestingness_ros.interest_marker:main",
# colcon build --package-select interestingness_ros

## For new package, Set up interface package.
# RUN source /opt/ros/foxy/setup.bash && ros2 pkg create interfaces --dependencies std_msgs geometry_msgs 
# RUN cd interfaces && rm -rf include/ src/
# in interfaces.package.xml, add <depend>std_msgs</depend>;<depend>geometry_msgs</depend>;<build_depend>rosidl_default_generators</build_depend>; <exec_depend>rosidl_default_runtime</exec_depend>; <member_of_group>rosidl_interface_packages</member_of_group>
# in CMakeLists.txt, delete C99 and if build testing block; add find_package(rosidl_default_generators REQUIRED); find_package(geometry_msgs REQUIRED);find_package(std_msgs REQUIRED); rosidl_generate_interfaces(${PROJECT_NAME} "msg/InterestInfo.msg" "msg/UnInterests.msg" DEPENDENCIES geometry_msgs std_msgs)
# cd ~/ros2_ws && colcon build --package-select interfaces

# cd ~/ros2_ws && colcon build --package-select my_py_pkg --symlink-install

# source ~/.bashrc