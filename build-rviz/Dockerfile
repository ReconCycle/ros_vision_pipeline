FROM ros:kinetic

# update packages and install dependencies
RUN apt-get update && apt-get install -y \
    ros-kinetic-rviz

# RUN source /opt/ros/${ROS_DISTRO}/setup.bash

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

# Always source ros_catkin_entrypoint.sh when launching bash (e.g. when attaching to container)
RUN echo "source /entrypoint.sh" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]

# stop docker from exiting immediately
CMD tail -f /dev/null 