xhost +local:

docker run -it \
    -e DISPLAY=unix${DISPLAY} \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/.Xauthority:/root/.Xauthority \
    --gpus all \
    --name=ros-noetic-cuda \
    ros-noetic-cuda \
    bash
echo "done"