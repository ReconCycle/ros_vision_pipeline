# use ubuntu 20.04 because we want to use ROS noetic
ARG TARGET="gpu"
FROM nvidia/cuda:11.3.1-cudnn8-devel-ubuntu20.04 as gpu
FROM ubuntu:20.04 as cpu

# FROM ros:noetic
LABEL maintainer "Sebastian Ruiz <sruiz@mailbox.org>"

SHELL ["/bin/bash","-c"]

################################################################
## BEGIN: ros:core
################################################################

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt update && apt install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO noetic

################################################################
## END: ros:core
## BEGIN: python3
################################################################

RUN apt update && apt install --no-install-recommends -y \
    software-properties-common \
    build-essential
RUN add-apt-repository universe

RUN apt update --fix-missing && apt install -y wget bzip2 ca-certificates zlib1g-dev libncurses5-dev libgdbm-dev \
    libglib2.0-0 libxext6 libsm6 libxrender1 libffi-dev \
    git \
    libusb-1.0-0-dev

RUN apt update && apt install -y python3-pip
RUN pip3 install --upgrade pip

################################################################
## END: python3
## BEGIN: ros:noetic
################################################################

# install ros packages
RUN apt update && apt install --no-install-recommends -y \
    ros-noetic-ros-core=1.5.0-1* \
    ros-noetic-ros-base=1.5.0-1* \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    python3-catkin-tools \
    ros-noetic-cv-bridge \
    ros-noetic-tf \
    vim \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

################################################################
## END: ros:noetic
## BEGIN: python packages
################################################################

# RUN if [ "$TARGET" = "gpu" ] ; then \
#        pip3 install torch torchvision --extra-index-url https://download.pytorch.org/whl/cu113  \
#     else \
#        pip3 install torch torchvision \
#     fi

RUN pip3 install torch torchvision --extra-index-url https://download.pytorch.org/whl/cu113

# RUN pip3 install torch torchvision

RUN pip3 install rich matplotlib numpy pillow scikit-learn scipy tensorboard pyyaml opencv-python regex natsort shapely commentjson pycocotools cython scikit-image pyrealsense2 hdbscan joblib==1.1.0 jsonpickle
RUN pip3 install lap cython_bbox

################################################################
## END: python packages
## BEGIN: pypylon
################################################################

RUN pip3 install pypylon

################################################################
## END: pypylon
## BEGIN: ROS
################################################################

# Create local catkin workspace
ENV CATKIN_WS=/root/catkin_ws
ENV ROS_PYTHON_VERSION=3
RUN mkdir -p $CATKIN_WS/src/ros_vision_pipeline
WORKDIR $CATKIN_WS

COPY ros_vision_pipeline $CATKIN_WS/src/ros_vision_pipeline

# Initialize local catkin workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd $CATKIN_WS \
    && rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO}

# RUN update-alternatives --remove python /usr/bin/python \
#     && update-alternatives --install /usr/bin/python python /usr/local/bin/python3 10

RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 10

# required for pyrealsense2
RUN pip3 install rospkg
RUN pip3 install rospy-message-converter

################################################################
## END: ROS
## BEGIN: realsense
################################################################

RUN apt install -y apt-transport-https
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
RUN apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev usbutils

################################################################
## END: realsense
## BEGIN: ROS (again?)
################################################################

# convert ros messages to json and back
RUN cd $CATKIN_WS/src \
    && git clone https://github.com/uos/rospy_message_converter.git

# these msgs were also used for pylon camera driver, so let's also add them
RUN cd $CATKIN_WS/src \
    && git clone https://github.com/dragandbot/dragandbot_common.git

# get message and service definitions for interacting with the pylon camera driver
RUN cd $CATKIN_WS/src \
    && git clone https://github.com/basler/pylon-ros-camera.git \
    && mv pylon-ros-camera/camera_control_msgs . \
    && rm -rf pylon-ros-camera

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd $CATKIN_WS \
    && catkin init \
    # && catkin config -DPYTHON_EXECUTABLE=/usr/local/bin/python3.8 \
    # && catkin config -DPYTHON_EXECUTABLE=/usr/local/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/local/include/python3.8 -DPYTHON_LIBRARY=/usr/local/lib/libpython3.8.a \
    && catkin config --install
    # && cd src \
    # && git clone -b noetic https://github.com/ros-perception/vision_opencv.git

# cv_bridge fix
# RUN sed -i 's/python3/python-py38/g' src/vision_opencv/cv_bridge/CMakeLists.txt

# RUN apt install -y python3-empy
RUN pip3 install empy

# RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
#     && cd $CATKIN_WS \
#     && catkin build cv_bridge -DSETUPTOOLS_DEB_LAYOUT=OFF \ 
#     # I was getting some error without -DSETUPTOOLS_DEB_LAYOUT=OFF
#     && source install/setup.bash --extend

# install ros_vision_pipeline
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && catkin build
    # && catkin install
    # catkin install doesn't seem to do anything?

# ROS breaks opencv because python3 will try and default to the python2.7 version of opencv. Delete it.
# RUN rm /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so

################################################################
## END: ROS (again?)
################################################################

# Always source ros_catkin_entrypoint.sh when launching bash (e.g. when attaching to container)
RUN echo "source /entrypoint.sh" >> /root/.bashrc

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

# # stop docker from exiting immediately
CMD tail -f /dev/null
