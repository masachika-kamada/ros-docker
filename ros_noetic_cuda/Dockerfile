# nvidia/cuda:11.1.1-cudnn8-devel-ubuntu20.04をベースにする
FROM nvidia/cuda:11.1.1-cudnn8-devel-ubuntu20.04

# osrf/ros:noetic-desktop-fullをdocker historyで展開

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
# RUN sh -c 'echo "deb http://packages.ros.org/ros-testing/ubuntu focal main" > /etc/apt/sources.list.d/ros-testing.list'

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO noetic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

WORKDIR /workspace

CMD ["bash"]

# 以下必要なパッケージのインストールなど

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y x11-xserver-utils xorg x11-apps

WORKDIR /root/

RUN echo "source /opt/ros/noetic/setup.sh" >> .bashrc
RUN mkdir -p catkin_ws/src && cd catkin_ws/src
RUN . /opt/ros/noetic/setup.sh && catkin_init_workspace
RUN cd && cd catkin_ws && . /opt/ros/noetic/setup.sh && catkin_make
RUN echo "source ./catkin_ws/devel/setup.bash" >> .bashrc
