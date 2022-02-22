#!/bin/bash

# Source ROS distro environment and local catwin workspace
source "/opt/ros/$ROS_DISTRO/setup.bash" && source "$CATKIN_WS/devel/setup.bash" && source "$CATKIN_WS/install/setup.bash"

# ROS screws up the PYTHONPATH by also adding python2 path.
# I tried doing this, but it breaks ROS as well :(
# unset PYTHONPATH
# export PYTHONPATH="/root/catkin_ws/devel/lib/python3.7/site-packages:$PYTHONPATH"
# export PYTHONPATH="/opt/conda/envs/pipeline-v2/lib/python3.7/site-packages:$PYTHONPATH"

# if something goes wrong with activating the environment we might have to do:
echo $LD_LIBRARY_PATH
export LD_LIBRARY_PATH="/opt/conda/envs/pipeline-v2/lib:$LD_LIBRARY_PATH"

echo "activating pipeline-v2 conda env"
# activate our conda environment
. /opt/conda/etc/profile.d/conda.sh
conda activate pipeline-v2

#check if we have already cloned the project. If not do it now.
if [ ! -d "/root/vision-pipeline" ]; then
    echo "You need to manually clone the github.com/ReconCycle/vision-pipeline repo and add it as a volume in docker."
    # cd $CATKIN_WS/src/ros-vision-pipeline \
    #     && git clone https://$GITHUB_APP_PASSWORD@github.com/ReconCycle/vision-pipeline.git
fi

#install deeplabcut if it's not installed yet
if ! pip list | grep -F deeplabcut &> /dev/null; then
    echo "installing deeplabcut..."
    cd /root/vision-pipeline/dlc/DeepLabCut-2.2b8 && ./reinstall.sh
fi

# install yolact_pkg
if ! pip list | grep -F yolact &> /dev/null; then
    echo "installing yolact..."
    cd /root/vision-pipeline/yolact_pkg && python -m pip install -e .
fi

cd /root/vision-pipeline

exec "$@"