CATKIN_WS=/home/kirill/catkin_ws/
source $CATKIN_WS/devel/setup.bash --extend
DIR="$( cd "$(dirname "$0")"&& pwd)"
PROJECT_DIR=${DIR}/..
mkdir $PROJECT_DIR/build
cd $PROJECT_DIR/build
cmake -DCMAKE_INSTALL_PREFIX=$PROJECT_DIR/install ..
make
#make install -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF
make install
source $PROJECT_DIR/install/setup.bash --extend
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$CATKIN_WS/src
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$CATKIN_WS/src/robot_simulation/build
roslaunch robot_simulation robot_control_world.launch