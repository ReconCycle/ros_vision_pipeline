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