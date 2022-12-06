#!/bin/bash

xhost +local:

if [ ! -d "workspace" ]; then
    mkdir workspace
fi

docker run -it \
    -e DISPLAY=unix${DISPLAY} \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/.Xauthority:/home/ubuntu/.Xauthority \
    -v $PWD/workspace:/home/ubuntu/catkin_ws \
    --name=ros_noetic \
    ros_noetic \
    bash