#!/bin/bash

# Source ROS distro environment and local catkin_ws
source "/opt/ros/$ROS_DISTRO/setup.bash"
if [ -f "$CATKIN_WS/devel/setup.bash" ]; then
    source "$CATKIN_WS/devel/setup.bash"
fi
if [ -f "$CATKIN_WS/install/setup.bash" ]; then
    source "$CATKIN_WS/install/setup.bash"
fi

HOME_DIR=/home/docker

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/python3.8

# install yolact_pkg only for vision pipeline
if [ -d "$DIR_VISION_PIPELINE" ]; then
    if ! pip3 list | grep -F yolact &> /dev/null; then
        echo "installing yolact..."
        cd $HOME_DIR/vision_pipeline/yolact_pkg && python3 -m pip install -e .
    fi
fi

# if build folder doesn't exist, run catkin build
if [ ! -d "$HOME_DIR/catkin_ws/build" ]; then
    cd $HOME_DIR/catkin_ws && catkin build

    # source the new local catkin_ws
    if [ -f "$CATKIN_WS/devel/setup.bash" ]; then
        source "$CATKIN_WS/devel/setup.bash"
    fi
    if [ -f "$CATKIN_WS/install/setup.bash" ]; then
        source "$CATKIN_WS/install/setup.bash"
    fi
fi

#############################################
# Python
#############################################

# export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.8
export PYTHONPATH="$PYTHONPATH:$HOME"
export PYTHONPATH="$PYTHONPATH:$HOME/vision_pipeline"
# export PYTHONPATH="$PYTHONPATH:$HOME/vision_pipeline/yolact_pkg"

export PYTHONPATH="$PYTHONPATH:$HOME/action_predictor"
export PYTHONPATH="$PYTHONPATH:$HOME/disassembly_pipeline"
export PYTHONPATH="$PYTHONPATH:$HOME/robotblockset_python"


exec "$@"