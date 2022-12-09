FROM osrf/ros:melodic-desktop-full

USER root

# Install X Window System
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y \
        x11-xserver-utils \
        xorg \
        x11-apps

RUN adduser ubuntu
WORKDIR /home/ubuntu

# ROSの環境整理
# ROSのセットアップシェルスクリプトを.bashrcファイルに追記
RUN echo "source /opt/ros/melodic/setup.sh" >> .bashrc
RUN mkdir catkin_ws

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

USER ubuntu