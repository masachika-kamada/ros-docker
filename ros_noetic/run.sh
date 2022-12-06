xhost +local:

if [ ! -d "workspace" ]; then
    mkdir workspace
fi

docker run -it \
    -e DISPLAY=unix${DISPLAY} \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -v $PWD/workspace:/root/catkin_ws \
    --gpus all \
    --name=ros_noetic \
    ros_noetic \
    bash
echo "done"