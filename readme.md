# A slimmer Dockerfile with Ubuntu 18 + ROS Melodic.

# Instructions for use:
1. Choose image you need and build it.
`docker build -f ros-gpu.dockerfile -t cmu-melodic-gpu:1.0 .`
2. Run using ./run.sh, or by the following command:
```
docker run -it \
    --gpus all\
    --env DISPLAY=$DISPLAY \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --privileged \
    <dockertaghere>
```
3. Verify your container's gui is working through a simple test:
roscore  >& /dev/null & rosrun rviz rviz

# CubeSLAM setup
1. Create a catkin_ws and source folder
`mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src`
2. Source the right setup.bash (/opt/ros/<ros-distribution>/setup.bash)
`source /opt/ros/melodic/setup.bash`
3. Initialize workspace
`catkin_init_workspace`
4. Get cube_slam repository and go into it
`git clone https://github.com/shichaoy/cube_slam && cd cube_slam`
5. Install g2o
`./install_dependenices.sh`
6. Install dependencies for build
```
sudo apt-get update
sudo apt install ros-melodic-pcl-ros
sudo apt install ros-melodic-image-geometry
```
7. Install Pangolin
```
sudo apt install libglew-dev
sudo apt install pkg-config
sudo apt install libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols
    
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
cmake --build .
```
8a. Add vector and numeric headers in cube_slam files
```
Go into `/home/developer/catkin_ws/src/cube_slam/detect_3d_cuboid/src/matrix_utils.cpp` and `/home/developer/catkin_ws/src/cube_slam/detect_3d_cuboid/include/detect_3d_cuboid/matrix_utils.h` and affix these to the top of the files:

```
#include <vector>
#include <numeric>
```
8b. Add unistd, stdio and stdlib headers in cube_slam files

Go into `/home/developer/catkin_ws/src/cube_slam/orb_object_slam/src/System.cc`, `vim /home/developer/catkin_ws/src/cube_slam/orb_object_slam/src/Viewer.cc`,
`vim /home/developer/catkin_ws/src/cube_slam/detect_3d_cuboid/src/box_proposal_detail.cpp, and `vim /home/developer/catkin_ws/src/cube_slam/orb_object_slam/src/LoopClosing.cc` and affix these to the top of the files:
```
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
```

9. Make
cd ~/catkin_ws && catkin_make -j4
