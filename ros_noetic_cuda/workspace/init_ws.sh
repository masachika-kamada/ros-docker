#!/bin/bash

CATKIN_WS=$HOME/catkin_ws

mkdir -p $CATKIN_WS/src && cd $CATKIN_WS/src
catkin_init_workspace
cd $CATKIN_WS && catkin_make
echo "source ./catkin_ws/devel/setup.bash" >> $HOME/.bashrc