xhost +local:docker 
docker run -it --rm \
    --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" \
    --gpus all
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v $PWD/introspkg:/root/ros2_ws/src/interestingness_ros/ \
    -v $PWD/interfaces/:/root/ros2_ws/src/interfaces/ \
    --name cmu \
    cmu-foxy:2.0