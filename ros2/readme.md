# ROS2 environment for running Interestingness
- A port from ROS1 to ROS2 of [this repo](https://github.com/wang-chen/interestingness_ros).
- Start a Ubuntu 20 + ROS2 Foxy container
## Requirements
- Docker, NVIDIA Container Toolkit  
## How to use
1. Add your own tmux/vim or other configs file into /configs. They will be copied to ~/ in the docker container.  
2. Build the docker image from the Dockerfile `docker build -f ros2-foxy.dockerfile -t cmu-foxy:2.2 .`  
2. Clone the interestingness repo. `cd introspkg && git clone -b master --single-branch https://github.com/wang-chen/interestingness.git`  
3. Download the PyTorch model from [here](https://github.com/wang-chen/interestingness/releases) and move it into `introspkg/interestingness/saves`. Check the model path in `interestingness.launch.py`.  
4. Download the SubT ROS bags recording from [here](https://entuedu-my.sharepoint.com/personal/cwang017_e_ntu_edu_sg/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Fcwang017%5Fe%5Fntu%5Fedu%5Fsg%2FDocuments%2FData%2FSubT&originalPath=aHR0cHM6Ly9lbnR1ZWR1LW15LnNoYXJlcG9pbnQuY29tLzpmOi9nL3BlcnNvbmFsL2N3YW5nMDE3X2VfbnR1X2VkdV9zZy9FcnJPd2VfWTNNZEVxXzJZNnhWZWlCb0JNbmNsaHBVZDFWLTBGMUdueHhnRmNRP3J0aW1lPVRCUHRVZ1JCMlVn) and move it into `introspkg/bags/`. You may test with a few files and may need to download the files individually.  
5. In `launch/subtf_bags.launch.py`, update the files to the ROS bags that you have downloaded.  
6. Make sure you are in the `ros2/` path as the run and volume mounting uses relative path. Start a docker container with `./run2.sh`  
7. In the container, run `colcon build`, followed by `source ~/.bashrc`  
8. Convert the .bags (1 at a time) with `rosbags-convert ~/ros2_ws/src/interestingness_ros/bags/{}.bag`. You should see a directory spawn with the same name containing a .yaml and a .db3. Note that you can skip this step the next time you start a container as the converted files are in your mapped volume.  
8. Test with `ros2 run interestingness_ros interestingness_subtf.launch.py`. You should see rviz2 start, similar to the gif below.  
Note: tmux is included in the docker image  
[Note2](https://stackoverflow.com/questions/68211850/typeerror-array-takes-1-positional-argument-but-2-were-given): Pillow 8.3.0 will fail when interestingness node performs self.transform. We downgrade pillow to 8.2.0.
## Details  
- interestingness_subtf.launch.py: Launches interestingness.launch.py and subtf_bags.launch.py. Also launches a Rviz node, you may change the rviz config file here. robot.launch.py is included but not tested pending a ROS2 husky gazebo implementation.  
- interestingness.launch.py: Start interestingness_node and interest_marker_node. You may change the torch model file here.  
- subtf_bags.launch.py: Starts replay of rosbags. You may change the data location here.  

- interestingness_node: Subscribes to image_topic & interaction_topic. Publishes to interestingness/image (image in rviz) & interestingness/info.  
- interest_marker_node: Subscribes to interestingness/info. Publishes to interestingness/marker (marker in rviz).  

TODO:  
- [x] add marker into rviz config
- [ ] make marker persistent like in melodic
- [x] fix msg type in interestingness node (rospy.numpy_msg)
- [ ] Argument/variable for datalocation in subtf_bags.launch.py
- [ ] Write a bash file to crawl dir and convert all .bags at once
- [ ] remove the unused topics 
- [ ] add husky/robot visualization
## Demo
<img src="https://raw.githubusercontent.com/seeeheng/deploy-slim/master/ros2/media/int2.gif" width="571" height="314" />