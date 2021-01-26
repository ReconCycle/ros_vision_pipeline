# ROS Package for Vision Pipeline

ROS package that encapsulates the vision pipeline.

## Pylon Basler Camera Software Suite on Docker Host Machine

The Basler camera we are using is: Basler acA4600-7gc

1. Download and install the Pylon camera software suite.
 Link: [pylon 6.1.1 Camera Software Suite Linux x86 (64 Bit) - Debian Installer Package](https://www.baslerweb.com/de/vertrieb-support/downloads/downloads-software/#type=pylonsoftware;language=all;version=all;os=linuxx8664bit)

2. To connect to the Basler camera over ethernet, create a new ethernet profile with settings:

- IPv4 Method: Manual
- Address: 192.168.1.200
- Netmask: 255.255.255.0
- Gateway: 192.168.1.1

![basler_ethernet_profile](./notes/basler_ethernet_profile.png)

3. Open the pylon viewer (on the host machine) and check that the Basler Camera appears here.

4. Settings to set for the Basler camera in pylon Viewer, see images in notes folder.

!todo, screenshot all the settings

The camera uses the [pypylon api](https://github.com/basler/pypylon) and this is installed in this docker image.

## Nvidia GPU Support


### CUDA Install on docker host machine

!todo

Prequisites:
```
sudo apt install build-essential libglvnd-dev pkg-config
```
Now [download here the nvidia drivers](https://www.nvidia.com/Download/index.aspx) and run as root to install.


Installation guide for CUDA from nvidia [here](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#runfile-installation).

- You may need to disable the built in display driver: [disable display driver](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#runfile-nouveau)

- You may need gcc version 8 to run CUDA 10.2. If so, run: `sudo apt install gcc-8 g++-8`. [Guide here](https://linuxize.com/post/how-to-install-gcc-on-ubuntu-20-04/) on how to switch gcc versions.

Install CUDA 10.2 on your host system. Go to [Cuda Toolkit Archive](https://developer.nvidia.com/cuda-toolkit-archive) then click on **CUDA Toolkit 10.2**. Select your operating system and download the runfile. Run the runfile using `sudo`.


CUDA says:
```
Please make sure that
 -   PATH includes /usr/local/cuda-10.2/bin
 -   LD_LIBRARY_PATH includes /usr/local/cuda-10.2/lib64, or, add /usr/local/cuda-10.2/lib64 to /etc/ld.so.conf and run ldconfig as root

```

I'm missing the driver: do this: `sudo <CudaInstaller>.run --silent --driver`


### Docker running as root

First you need to install nvidia-container-runtime, [instructions here](https://nvidia.github.io/nvidia-container-runtime/) and run:
```
sudo apt-get install nvidia-container-runtime
```

Edit `/etc/docker/daemon.json` to contain:

```
{
    "default-runtime":"nvidia",
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    }
}
```

Then: `sudo systemctl daemon-reload`

Check if runtime is added sucessfully:
`docker info|grep -i runtime`

### Docker running rootless

Do the same except put the file here: `~/.config/docker/daemon.json`.

Then: `systemctl --user daemon-reload`

Check if runtime is added sucessfully:
`docker info|grep -i runtime`

## Running Docker container

Run:
```
$ cd ros-vision-pipeline
$ docker-compose up -d
```

## Resources

Python with ROS:

- http://www.artificialhumancompanions.com/structure-python-based-ros-package/
- https://docs.freedomrobotics.ai/docs/ros-development-in-docker-on-mac-and-windows

ROS series on Python Package:

- http://wiki.ros.org/ROS/Tutorials/CreatingPackage
- http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber
- http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

ROS docker setup:

- https://github.com/gramaziokohler/ros_docker
- https://github.com/ReconCycle/docker_examples/blob/master/compose_files/ros1_echo/docker-compose.yml

Alternative method without requiring to use ROS directly:

- https://github.com/gramaziokohler/roslibpy

ROS opencv cv_bridge:

- https://stackoverflow.com/questions/49221565/unable-to-use-cv-bridge-with-ros-kinetic-and-python3

Publish/Subscribe to Images in ROS:

- https://stackoverflow.com/questions/55377442/how-to-subscribe-and-publish-images-in-ros
- https://stackoverflow.com/questions/58590277/how-to-use-opencv-python-in-ros

ROS incompatibility with OpenCV Python3:

- https://stackoverflow.com/questions/43019951/after-install-ros-kinetic-cannot-import-opencv 

ROS Yolact:

- https://github.com/Eruvae/yolact_ros

Supervisors

- https://github.com/just-containers/s6-overlay#quickstart

Reduce docker size:

TODO

- https://jcristharif.com/conda-docker-tips.html
