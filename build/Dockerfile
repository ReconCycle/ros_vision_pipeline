# use ubuntu 16.04 because we want to use ROS kinetic
FROM nvidia/cuda:10.2-cudnn7-devel-ubuntu16.04 

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

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kinetic-ros-core=1.3.2-0* \
    && rm -rf /var/lib/apt/lists/*

################################################################
## END: ros:core
## BEGIN: ros:kinetic
################################################################

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
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
RUN mkdir -p $CATKIN_WS/src/vision-pipeline
WORKDIR $CATKIN_WS

COPY environment.yml $CATKIN_WS/src/vision-pipeline
RUN cd $CATKIN_WS/src/vision-pipeline && conda env create -f environment.yml --debug

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