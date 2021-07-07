xhost +local:docker 
docker run -it --rm \
    --privileged \
    --gpus=all \
    -e "DISPLAY=$DISPLAY" \
    -e "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v $PWD/introspkg:/root/ros2_ws/src/interestingness_ros/ \
    -v $PWD/interfaces/:/root/ros2_ws/src/interfaces/ \
    --name cmu \
    cmu-foxy:2.2

#2.1: added ENV NVIDIA in dockerfile, added pip rosbags, changed WORKDIR
#2.2: added pip pillow version and source docker-prompt