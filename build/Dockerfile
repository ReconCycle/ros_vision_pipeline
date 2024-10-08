# use ubuntu 20.04 because we want to use ROS noetic
FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04
ARG UID=1000
ARG GID=1000

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

# install newer version of git
RUN add-apt-repository ppa:git-core/ppa -y

RUN apt update --fix-missing && apt install -y wget bzip2 ca-certificates zlib1g-dev libncurses5-dev libgdbm-dev \
    libglib2.0-0 libxext6 libsm6 libxrender1 libffi-dev \
    git \
    bash-completion

# BEGIN: python 3.9
# RUN add-apt-repository ppa:deadsnakes/ppa
# RUN apt update && apt install -y python3.9
# RUN apt install -y python3-pip
# RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.9 1
# RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 10
# END: python 3.9

# BEGIN: python3 default
RUN apt update && apt install -y python3-pip
RUN pip3 install --upgrade pip
# END: python3 default

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
    ros-noetic-ros-numpy \
    ros-noetic-image-view \
    ros-noetic-image-pipeline \
    ros-noetic-image-transport-plugins \
    ros-noetic-libfranka \
    ros-noetic-franka-ros \
    vim \
    screen \
    libx11-dev \
    python3-tk \
    git-lfs \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# for showing matplotlib windows, we need: libx11-dev, python3-tk

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

################################################################
## END: ros:noetic
## BEGIN: user setup
################################################################

# https://stackoverflow.com/questions/27701930/how-to-add-users-to-docker-container
RUN groupadd -g $GID docker
RUN useradd -rm -d /home/docker -s /bin/bash -g docker -G sudo -u $UID docker -p "$(openssl passwd -1 docker)"

RUN touch /home/docker/.sudo_as_admin_successful

# enable case-insensitive tab completion
RUN echo 'set completion-ignore-case On' >> /etc/inputrc

################################################################
## END: user setup
## BEGIN: ROS
################################################################

# Create local catkin workspace
ENV CATKIN_WS=/home/docker/catkin_ws
ENV ROS_PYTHON_VERSION=3
WORKDIR $CATKIN_WS

# Initialize local catkin workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd $CATKIN_WS \
    && rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO}

################################################################
## END: ROS
## BEGIN: python
################################################################

# symlink python to python3
# RUN update-alternatives --remove python /usr/bin/python \
#     && update-alternatives --install /usr/bin/python python /usr/local/bin/python3 10
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 10
RUN pip3 install --upgrade pip

# install neural network packages
RUN pip3 install torch==1.12.1+cu113 torchvision==0.13.1+cu113 torchaudio==0.12.1 lightning torchsummary numpy tensorboard cython_bbox git+https://github.com/openai/CLIP.git ultralytics --extra-index-url https://download.pytorch.org/whl/cu113 --ignore-installed PyYAML

# install other packages
# set numpy==1.23.5 to avoid "module 'numpy' has no attribute 'float' error"
RUN pip3 install rich matplotlib numpy==1.23.5 pandas pillow scikit-learn scipy pyyaml opencv-contrib-python regex ftfy natsort shapely commentjson pycocotools cython==0.29.37 scikit-image pyrealsense2 hdbscan joblib==1.1.0 jsonpickle ipykernel probreg albumentations seaborn seaborn-image imutils tol_colors lap

# cython is problematic for some reason, with hdbscan. Reinstall:
RUN pip3 install --upgrade --force-reinstall cython

RUN pip3 install cmake
RUN pip3 install rospkg
RUN pip3 install rospy-message-converter
RUN pip3 install empy
RUN pip3 install imagesize

RUN pip3 install pyperplan
RUN pip3 install unified-planning
RUN pip3 install unified-planning[engines]
RUN pip3 install unified-planning[pyperplan]

RUN pip3 install quaternionic numpy-quaternion

# ROS breaks opencv because python3 will try and default to the python2.7 version of opencv. Delete it.
# RUN rm /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so

################################################################
## END: python
################################################################

COPY bashrc.sh /home/docker/.bashrc

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

RUN mkdir /home/docker/vision_pipeline
WORKDIR /home/docker/vision_pipeline

ENTRYPOINT ["/entrypoint.sh"]

# stop docker from exiting immediately
CMD tail -f /dev/null

USER docker