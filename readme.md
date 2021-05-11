
# Running
docker run -it \
    --gpus all\
    --env DISPLAY=$DISPLAY \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --privileged \
    <dockertaghere>

# Testing docker
roscore  >& /dev/null & rosrun rviz rviz

# Post Docker
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
source /opt/ros/melodic/setup.bash
catkin_init_workspace
git clone https://github.com/shichaoy/cube_slam
cd cube_slam

# install g2o
sh install_dependenices.sh

sudo apt-get update
sudo apt install ros-melodic-pcl-ros
sudo apt install ros-melodic-image-geometry

# install pangolin
sudo apt install libglew-dev
sudo apt install pkg-config
sudo apt install libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols

cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build .

# Fix CPP files.
vim /home/developer/catkin_ws/src/cube_slam/detect_3d_cuboid/src/matrix_utils.cpp
vim /home/developer/catkin_ws/src/cube_slam/detect_3d_cuboid/include/detect_3d_cuboid/matrix_utils.h
add these:
#include <vector>
#include <numeric>

vim /home/developer/catkin_ws/src/cube_slam/orb_object_slam/src/System.cc
vim /home/developer/catkin_ws/src/cube_slam/orb_object_slam/src/Viewer.cc
vim /home/developer/catkin_ws/src/cube_slam/detect_3d_cuboid/src/box_proposal_detail.cpp
vim /home/developer/catkin_ws/src/cube_slam/orb_object_slam/src/LoopClosing.cc
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

# make
cd ~/catkin_ws && catkin_make -j4


