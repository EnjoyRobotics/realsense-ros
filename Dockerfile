ARG FROM_IMAGE=ros:galactic
ARG OVERLAY_WS=/opt/ros/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# clone overlay source
ARG OVERLAY_WS
WORKDIR /$OVERLAY_WS/src

COPY . ./realsense-ros

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true


# multi-stage for building
FROM $FROM_IMAGE AS builder

ENV DEBIAN_FRONTEND noninteractive
ENV export MAKEFLAGS="-j12"

# realsense dependencies
RUN apt update --fix-missing && apt install -y ros-galactic-librealsense2 libssl-dev \ 
  libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \ 
  ros-galactic-cv-bridge ros-galactic-message-filters ros-galactic-image-transport


# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -y \
      --from-paths src\
      --ignore-src\
      --skip-keys find_object_2d

RUN rm -rf /var/lib/apt/lists/*

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --symlink-install \
      --mixin $OVERLAY_MIXINS \
      | echo 0


# source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

# run launch file
CMD ["ros2", "launch", "realsense2_camera", "rs_bringup.launch.py"]
