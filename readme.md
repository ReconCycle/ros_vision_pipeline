# ROS Package for Vision Pipeline

ROS package that encapsulates the vision pipeline.

## Nvidia GPU Support

### Docker running as root

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