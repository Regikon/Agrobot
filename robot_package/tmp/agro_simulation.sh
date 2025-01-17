#!/bin/bash
pkill gzserver
pkill gzclient
CATKIN_WS=/home/aida/agro_ws

DIR="$( cd $CATKIN_WS/src/agro/tmp && pwd)"
PROJECT_DIR="$(cd ${DIR}/.. && pwd)"
sudo rm -rf $PROJECT_DIR/build $PROJECT_DIR/install

source $CATKIN_WS/devel/setup.zsh --extend

mkdir $PROJECT_DIR/build
cd $PROJECT_DIR/build
cmake -DCMAKE_INSTALL_PREFIX=$PROJECT_DIR/install ..
make
make install
source $PROJECT_DIR/install/setup.zsh --extend
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$CATKIN_WS/src
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$CATKIN_WS/src/agro/build
roslaunch agro main.launch