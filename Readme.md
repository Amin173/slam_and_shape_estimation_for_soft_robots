# slam_and_shape_estimation_for_soft_robots

A ROS package for the under-preparation paper "Localization, mapping, and control with a mobile soft robot based on boundary-constrained modular units".

## Dependencies
- [Docker](https://www.docker.com)

## Installation

For the container to be able to clone the private repository, you have to generate and ssh key and add it to your github account as explained [here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent).

```sh
export SSH_PRIVATE_KEY="$(cat ~/.ssh/id_rsa)"
```

Additionally, make sure your `DISPLAY` parameter is set. This will allow the docker container to route its graphics to the host computer. To give docker access to your display server run:

```sh 
xhost +local:docker
```

You can revoke this permission afterwards by running:

```sh
xhost -local:docker
```

More information on the docker remote display can be found [here](http://wiki.ros.org/docker/Tutorials/GUI).

Now clone the repository and launch the docker container as follows:

```sh
git clone https://github.com/Amin173/slam_and_shape_estimation_for_soft_robots
cd ./slam_and_shape_estimation_for_soft_robots/docker
docker compose up
```

## Usage 

After the docker container is launched, you can lookup the container by running `docker ps`. Using the docker `container_id`, you can login to the container and use it as a remote server as follows:

```sh
docker exec -it <container_id> bash
```

To run the ROS application run the following commands inside the container:

```sh
cd /opt/ros/overlay_ws
source devel/setup.bash
/bin/bash ./src/slam_and_shape_estimation_for_soft_robots/bash/slam.sh
```

This should launch the application and RVIZ should show up in your host display.

## Remote containers

To make it easier to use docker containers, you can use the [remote container](https://code.visualstudio.com/docs/remote/containers) VSCode extention. This will launch a VSCode session inside your container, which gives you better access to the container file system and lets you directly edit the code using the VSCode from within the container.
