FROM osrf/ros:noetic-desktop-full

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
RUN echo "source /opt/ros/noetic/setup.sh" >> .bashrc
RUN mkdir catkin_ws

USER ubuntu