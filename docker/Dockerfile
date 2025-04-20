# global args
ARG FROM_IMAGE=osrf/ros:noetic-desktop
ARG OVERLAY_WS=/opt/ros/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS updated

RUN apt update -y && apt upgrade -y \
    && apt-get -yqq install ssh \
    && apt autoremove -y && apt autoclean -y 

FROM updated AS cacher

ARG CACHBUST="3"

# clone workspace source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
COPY ./pckgs.rosinstall ../
RUN apt update && apt install git -y\
    && rm -rf /var/lib/apt/lists/*
RUN wstool init . ../pckgs.rosinstall 
RUN wstool update -t . 

RUN git clone https://github.com/Amin173/slam_and_shape_estimation_for_soft_robots.git

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "CATKIN_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# template build container for multi-stage building
FROM updated AS builder_temp

# install catkin tools
RUN apt update \
    && apt install ros-$ROS_DISTRO-catkin python3-catkin-tools python3-osrf-pycommon python3-pip -y \
    && apt install nano -y \
    && apt install git -y \
    && apt upgrade python3-pip -y \
    && python3 -m pip install -U pip \
    && python3 -m pip install --upgrade setuptools pip \
    && python3 -m pip install numpy \
    && python3 -m pip install netifaces \
    && python3 -m pip install pyparticleio \
    && python3 -m pip install pandas \
    && python3 -m pip install matplotlib \
    && python3 -m pip install scikit-build \
    && python3 -m pip install opencv-python \
    && python3 -m pip install open3d --ignore-installed PyYAML \
    && python3 -m pip install scipy \
    && python3 -m pip install ruamel.yaml \
    && python3 -m pip install pathlib \
    && rm -rf /var/lib/apt/lists/*

# final build container
FROM builder_temp AS builder

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN rosdep update
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -y \
      --from-paths \
        src\
      --ignore-src \
      --rosdistro $ROS_DISTRO\
    && rm -rf /var/lib/apt/lists/*

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    catkin build 

# source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/devel/setup.bash"' \
      /ros_entrypoint.sh

CMD [ "bash"]
