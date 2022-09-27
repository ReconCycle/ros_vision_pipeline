# ROS Package for Vision Pipeline

Docker container that offers GPU support and a configured python environment for [vision-pipeline](https://github.com/ReconCycle/vision-pipeline) and [action-predictor](https://github.com/ReconCycle/action_predictor).

![RViz](./readme_rviz.png)

## Installation

1. `git clone git@github.com:ReconCycle/ros-vision-pipeline.git`
2. `git clone git@github.com:ReconCycle/vision-pipeline.git`
3. `git clone git@github.com:ReconCycle/context_action_framework.git`
4. `cd ros-vision-pipeline`
5. `docker-compose build --build-arg 'TARGET=gpu'`
6. `cp docker-compose.example.yml docker-compose.yml`
7. `docker-compose up -d`


To run the [vision-pipeline](https://github.com/ReconCycle/vision-pipeline) follow the installation instructions there.

To run the [action-predictor](https://github.com/ReconCycle/action_predictor) follow the installation instructions there.

## Nvidia GPU Support

These steps are only necessary if the PC has an Nvidia graphics card.

You need to install the Nvidia graphics drivers and the CUDA toolkit. The Nvidia drivers are also bundled with CUDA, but I had trouble installing it this way.

### **Installing Nvidia graphics drivers**

Prequisites:
```
sudo apt install build-essential libglvnd-dev pkg-config
```
Now [download here the nvidia drivers](https://www.nvidia.com/Download/index.aspx) and run as root to install. The Nvidia drivers require gcc-9 which is what ubuntu ships with by default.

- You may need to disable the built in display driver: [disable display driver](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#runfile-nouveau)

### **Installing CUDA toolkit**

Install **CUDA 11.3** on your host system. Go to [Cuda Toolkit Archive](https://developer.nvidia.com/cuda-toolkit-archive) then click on **CUDA Toolkit 11.3**. Select your operating system and download the runfile. Run the runfile using `sudo`.

- You may need gcc version 8 to run CUDA 11.3. If so, run: `sudo apt install gcc-8 g++-8`. [Guide here](https://linuxize.com/post/how-to-install-gcc-on-ubuntu-20-04/) on how to switch gcc versions.

There is an installation guide for CUDA from nvidia [here](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#runfile-installation). You can have a look at it for reference.

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

### Docker running rootless (probably not what you want to do)

Do the same except put the file here: `~/.config/docker/daemon.json`.

Then: `systemctl --user daemon-reload`

Check if runtime is added sucessfully:
`docker info|grep -i runtime`

ONLY INSTALL DOCKER AFTER DOING THESE STEPS IF YOU ARE RUNNING ROOTLESS!

## Running Docker container

Make sure you have everything set up from the previous sections. Install [docker-compose](https://docs.docker.com/compose/install/).

Edit the `docker-compose.yml` file and remove the ROS master and Rviz if you already have these running elsewhere. In principle the `docker-compose.yml` file in its current state will provide you with a ROS master, and Rviz that can be accessed via the browser through the `novnc` container.

The container is running in `host` mode because this is the easiest way to give it access to the Basler camera. The `ROS_IP` needs to be set correctly. Do this by running `$ hostname -I` on the host and setting the `ROS_IP` to this IP (take the first one if it gives multiple IP addresses).

### Cloning [Vision Pipeline Project](https://github.com/ReconCycle/vision-pipeline)

Clone the [https://github.com/ReconCycle/vision-pipeline](https://github.com/ReconCycle/vision-pipeline) project. Set the path of the project in the `volumes` section of the `docker-compose.yml`. This is the path before the ":" sign. Do not change `/root/vision-pipeline`.

```yaml
volumes:
  - $HOME/projects/vision-pipeline:/root/vision-pipeline
```

### Specifying CPU or GPU target

If you are running a Nvidia GPU and you have configured **nvidia-container-runtime** then you can use the GPU target. If not, you should use CPU.

To set the target, change the following in `docker-compose.yml`:

```yaml
build:
    context: ./build-2in1
    args: 
    TARGET: cpu # cpu or gpu
```

## Running The Pipeline and the Camera Publisher

Run:
```bash
$ cd ros-vision-pipeline
$ docker-compose up -d
```

