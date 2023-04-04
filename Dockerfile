ARG ROS_DISTRO=humble
ARG PREFIX=

# Additional build stage responsible for cloning and building just the MPPI controller from sources

FROM husarnet/ros:${ROS_DISTRO}-ros-base AS controller_builder
RUN apt update && apt install -y \
        git && \
    git clone --branch $ROS_DISTRO https://github.com/ros-planning/navigation2.git src/ && \
    rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    git -C src/ sparse-checkout set nav2_mppi_controller/** && \
    rosdep init && rosdep update && \
    rosdep install -y -r -q --from-paths src --rosdistro $ROS_DISTRO && \
    source /opt/ros/humble/setup.bash && \
    colcon build

FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-core

ARG PREFIX
ENV SLAM_MODE=localization
ENV PREFIX_ENV=$PREFIX

SHELL ["/bin/bash", "-c"]

RUN apt update && apt upgrade -y && apt install -y \
        ros-$ROS_DISTRO-navigation2 \
        ros-$ROS_DISTRO-nav2-bringup \
        ros-$ROS_DISTRO-pointcloud-to-laserscan && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY ./nav2_params /nav2_params

COPY healthcheck_* /

COPY --from=controller_builder /ros2_ws/install /ros2_ws/install

RUN echo $(dpkg -s ros-$ROS_DISTRO-navigation2 | grep 'Version' | sed -r 's/Version:\s([0-9]+.[0-9]+.[0-9]*).*/\1/g') >> /version.txt

HEALTHCHECK --interval=10s --timeout=10s --start-period=5s --retries=6  \
    CMD bash -c "MYDISTRO=${PREFIX_ENV:-ros}; MYDISTRO=${MYDISTRO//-/} && source /opt/$MYDISTRO/$ROS_DISTRO/setup.bash && /healthcheck_$SLAM_MODE.py"

