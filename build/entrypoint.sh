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
conda activate pipeline-v2

#check if we have already cloned the project. If not do it now.
if [ ! -d "$CATKIN_WS/src/ros-vision-pipeline/vision-pipeline" ]; then
    echo "cloning of vision-pipeline disabled (we will develop in a volume for now)"
    # cd $CATKIN_WS/src/ros-vision-pipeline \
    #     && git clone https://$GITHUB_APP_PASSWORD@github.com/ReconCycle/vision-pipeline.git
fi

cd /root/

exec "$@"