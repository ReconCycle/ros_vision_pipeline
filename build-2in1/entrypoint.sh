#!/bin/bash

# Source ROS distro environment and local catwin workspace
source "/opt/ros/$ROS_DISTRO/setup.bash" && source "$CATKIN_WS/devel/setup.bash" && source "$CATKIN_WS/install/setup.bash"

# ROS screws up the PYTHONPATH by also adding python2 path.
# I tried doing this, but it breaks ROS as well :(
# unset PYTHONPATH
# export PYTHONPATH="/root/catkin_ws/devel/lib/python3.7/site-packages:$PYTHONPATH"
# export PYTHONPATH="/opt/conda/envs/pipeline-v2/lib/python3.7/site-packages:$PYTHONPATH"

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/python3.8
export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.8

# if something goes wrong with activating the environment we might have to do:
echo $LD_LIBRARY_PATH
# export LD_LIBRARY_PATH="/opt/conda/envs/pipeline-v2/lib:$LD_LIBRARY_PATH"

# echo "activating pipeline-v2 conda env"
# activate our conda environment
# . /opt/conda/etc/profile.d/conda.sh
# conda activate pipeline-v2

#check if we have already cloned the project. If not do it now.
# if [ ! -d "/root/vision-pipeline" ]; then
#     echo "You need to manually clone the github.com/ReconCycle/vision-pipeline repo and add it as a volume in docker."
#     # cd $CATKIN_WS/src/ros-vision-pipeline \
#     #     && git clone https://$GITHUB_APP_PASSWORD@github.com/ReconCycle/vision-pipeline.git
# fi

#install deeplabcut if it's not installed yet
# if ! pip list | grep -F deeplabcut &> /dev/null; then
#     echo "installing deeplabcut..."
#     cd /root/vision-pipeline/dlc/DeepLabCut-2.2b8 && ./reinstall.sh
# fi

DIR_CONTEXT_ACTION_FRAMEWORK=/root/catkin_ws/src/context_action_framework
DIR_VISION_PIPELINE=/root/vision-pipeline
DIR_ACTION_PREDICTOR=/root/action_predictor

# install context_action_framework
if [ -d "$DIR_CONTEXT_ACTION_FRAMEWORK" ]; then
    # install python package
    # if ! pip3 list | grep -F context_action_framework &> /dev/null; then
    #     echo "installing context_action_framework..."
    #     cd $DIR && python3 -m pip install -e .
    # fi

    # install ros package, which also installs python package
    if ! rospack list-names | grep -F context_action_framework &> /dev/null; then
        echo "installing context_action_framework catkin package..."
        cd /root/catkin_ws && catkin build && catkin config --install
        source "/opt/ros/$ROS_DISTRO/setup.bash" && source "$CATKIN_WS/devel/setup.bash" && source "$CATKIN_WS/install/setup.bash"
        # reload rospack
        cd /root/catkin_ws && rospack profile
    fi
fi

# install yolact_pkg only for vision pipeline
if [ -d "$DIR_VISION_PIPELINE" ]; then
    if ! pip3 list | grep -F yolact &> /dev/null; then
        echo "installing yolact..."
        cd /root/vision-pipeline/yolact_pkg && python3 -m pip install -e .
    fi
fi

# now cd to the right directory
if [ -d "$DIR_VISION_PIPELINE" ]; then
    cd $DIR_VISION_PIPELINE
fi

if [ -d "$DIR_ACTION_PREDICTOR" ]; then
    cd $DIR_ACTION_PREDICTOR
fi

exec "$@"