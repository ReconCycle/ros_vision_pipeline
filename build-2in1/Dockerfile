# use ubuntu 16.04 because we want to use ROS kinetic
ARG TARGET="gpu"
FROM nvidia/cuda:10.2-cudnn7-devel-ubuntu16.04 as gpu
FROM ubuntu:16.04 as cpu

# FROM ros:kinetic
LABEL maintainer "Sebastian Ruiz <sruiz@mailbox.org>"

SHELL ["/bin/bash","-c"]

################################################################
## BEGIN: ros:core
################################################################

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros1-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO kinetic

################################################################
## END: ros:core
## BEGIN: ros:kinetic
################################################################

# install ros packages
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-kinetic-ros-core=1.3.2-0* \
    build-essential \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    python-catkin-tools \
    ros-kinetic-cv-bridge \
    vim \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kinetic-ros-base=1.3.2-0* \
    && rm -rf /var/lib/apt/lists/*

################################################################
## END: ros:kinetic
## BEGIN: anaconda3
################################################################

ENV PATH /opt/conda/bin:$PATH

RUN apt-get update --fix-missing && apt-get install -y wget bzip2 ca-certificates \
    libglib2.0-0 libxext6 libsm6 libxrender1 \
    git mercurial subversion

RUN wget --quiet https://repo.anaconda.com/archive/Anaconda3-2020.11-Linux-x86_64.sh -O ~/anaconda.sh && \
    /bin/bash ~/anaconda.sh -b -p /opt/conda && \
    rm ~/anaconda.sh && \
    ln -s /opt/conda/etc/profile.d/conda.sh /etc/profile.d/conda.sh && \
    echo ". /opt/conda/etc/profile.d/conda.sh" >> ~/.bashrc && \
    echo "conda activate base" >> ~/.bashrc

RUN conda config --add channels conda-forge
RUN conda config --set channel_priority false
# RUN conda config --set allow_conda_downgrades true # not working

################################################################
## END: anaconda3
################################################################

RUN apt-get update && apt-get install -y \
    software-properties-common
RUN add-apt-repository universe

# Create local catkin workspace
ENV CATKIN_WS=/root/catkin_ws
ENV ROS_PYTHON_VERSION=3
RUN mkdir -p $CATKIN_WS/src/ros_vision_pipeline
WORKDIR $CATKIN_WS

# COPY environment.yml $CATKIN_WS/src/ros_vision_pipeline
# RUN cd $CATKIN_WS/src/ros_vision_pipeline && conda env create -f environment.yml --debug

COPY spec-file.txt $CATKIN_WS/src/ros_vision_pipeline
RUN cd $CATKIN_WS/src/ros_vision_pipeline && conda create -n pipeline-v2 --file spec-file.txt --debug

# install pypylon from wheel
COPY pypylon-1.6.0-cp37-cp37m-linux_x86_64.whl /root
RUN conda run -n pipeline-v2 pip3 install /root/pypylon-1.6.0-cp37-cp37m-linux_x86_64.whl

COPY ros_vision_pipeline $CATKIN_WS/src/ros_vision_pipeline
RUN chmod +x $CATKIN_WS/src/ros_vision_pipeline/scripts/*.py

RUN conda run -n pipeline-v2 pip3 install empy

# Initialize local catkin workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd $CATKIN_WS \
    && rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO}

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd $CATKIN_WS \
    && catkin init \
    && catkin config -DPYTHON_EXECUTABLE=/opt/conda/envs/pipeline-v2/bin/python3 -DPYTHON_INCLUDE_DIR=/opt/conda/envs/pipeline-v2/include/python3.7m -DPYTHON_LIBRARY=/opt/conda/envs/pipeline-v2/lib/libpython3.7m.so \
    && catkin config --install \
    && cd src \
    && git clone -b kinetic https://github.com/ros-perception/vision_opencv.git

# cv_bridge fix
RUN sed -i 's/python3/python-py35/g' src/vision_opencv/cv_bridge/CMakeLists.txt

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd $CATKIN_WS \
    && catkin build cv_bridge -DSETUPTOOLS_DEB_LAYOUT=OFF \ 
    # I was getting some error without -DSETUPTOOLS_DEB_LAYOUT=OFF
    && source install/setup.bash --extend

# install ros_vision_pipeline
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && catkin build ros_vision_pipeline
    # && catkin install
    # catkin install doesn't seem to do anything?

# ROS breaks opencv because python3 will try and default to the python2.7 version of opencv. Delete it.
RUN rm /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so

# Always source ros_catkin_entrypoint.sh when launching bash (e.g. when attaching to container)
RUN echo "source /entrypoint.sh" >> /root/.bashrc

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

# stop docker from exiting immediately
CMD tail -f /dev/null 
