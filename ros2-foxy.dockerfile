# docker build -f ros2-foxy.dockerfile -t cmu-foxy:1.0 .
# xhost +local:docker && docker run -it --rm --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v $PWD/py_nodes:/root/ros2_ws/src/interestingness_ros/interestingness_ros/ -v $PWD/interfaces/msg:/root/ros2_ws/src/interfaces/msg  --name cmu cmu-foxy:1.0

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

COPY ~/.docker-prompt ~/.vimrc ~/.tmux.conf /root/

## Set up bashrc
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
RUN echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc # exists after the first colcon build
RUN ln -s /opt/ros/foxy/lib/libconsole_bridge.so /opt/ros/foxy/lib/libconsole_bridge.so.1.0 #required for cpp example node to run ros2 run demo_nodes_cpp talker

## Set up ros2 workspace and install cvbridge
RUN mkdir -p ~/ros2_ws/src
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/ros-perception/vision_opencv.git
RUN cd vision_opencv && git checkout ros2 && colcon build

## Build node package
RUN source /opt/ros/foxy/setup.bash && ros2 pkg create interestingness_ros --build-type ament_python --dependencies rclpy visualization_msgs sensor_msgs cv_bridge  
# edit setup.py in python package: import os from glob import glob 
# in data_files: (os.path.join('share', package_name), glob('launch/*.launch.py'))
# in console_scipts: "interestingness_node = interestingness_ros.interestingness_node:main","interest_marker_node = interestingness_ros.interest_marker:main",
# colcon build

## Set up interface package
RUN source /opt/ros/foxy/setup.bash && ros2 pkg create interfaces --dependencies rclcpp 
RUN cd interfaces && rm -rf include/ src/
# in interfaces.package.xml, add <build_depend>rosidl_default_generators</build_depend>; <exec_depend>rosidl_default_runtime</exec_depend>; <member_of_group>rosidl_interface_packages</member_of_group>
# in CMakeLists.txt, delete C99 and if build testing block; add find_package(rosidl_default_generators REQUIRED); rosidl_generate_interfaces(${PROJECT_NAME} "msg/InterestInfo.msg" "msg/UnInterests.msg")
# cd ~/ros2_ws && colcon build --package-select interfaces

# docker cp number_publisher.py {container name}:/root/ros2_ws/src/my_py_pkg/my_py_pkg/
# cd ~/ros2_ws && colcon build --package-select my_py_pkg --symlink-install

# source ~/.bashrc