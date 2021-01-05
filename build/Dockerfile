FROM ros:kinetic
LABEL maintainer "Sebastian Ruiz <sruiz@mailbox.org>"

SHELL ["/bin/bash","-c"]

# Install python
RUN apt-get update && apt-get install -y \
    software-properties-common
RUN add-apt-repository universe
RUN apt-get update && apt-get install -y \
    git \
    python3 \
    python3-pip

# Install python packages
RUN pip3 install pyyaml rospkg

# Create local catkin workspace
ENV CATKIN_WS=/root/catkin_ws
ENV ROS_PYTHON_VERSION=3
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS

COPY vision-pipeline $CATKIN_WS/src/vision-pipeline
RUN chmod +x $CATKIN_WS/src/vision-pipeline/scripts/talker.py
RUN chmod +x $CATKIN_WS/src/vision-pipeline/scripts/listener.py

# Initialize local catkin workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    # Install dependencies
    && cd $CATKIN_WS \
    && rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} \
    # Build catkin workspace
    && catkin_make

# Always source ros_catkin_entrypoint.sh when launching bash (e.g. when attaching to container)
RUN echo "source /entrypoint.sh" >> /root/.bashrc

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
# CMD ["bash"]

# stop docker from exiting immediately
CMD tail -f /dev/null 