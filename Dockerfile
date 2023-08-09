ARG ROS_DISTRO=humble
ARG PREFIX=

FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-core

SHELL ["/bin/bash", "-c"]

ARG PREFIX
ENV SLAM_MODE=localization
ENV PREFIX_ENV=$PREFIX

RUN apt update && apt upgrade -y && apt install -y \
        ros-$ROS_DISTRO-navigation2 \
        ros-$ROS_DISTRO-nav2-bringup \
        ros-$ROS_DISTRO-pointcloud-to-laserscan && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY ./nav2_params /nav2_params

COPY --chmod=755 healthcheck_* /

RUN echo $(dpkg -s ros-$ROS_DISTRO-navigation2 | grep '<version>' | sed -r 's/.*<version>([0-9]+.[0-9]+.[0-9]+)<\/version>/\1/g') > /version.txt

HEALTHCHECK --interval=10s --timeout=10s --start-period=5s --retries=6  \
    CMD bash -c "source /opt/$([ -n "${PREFIX_ENV}" ] && echo "${PREFIX_ENV//-/}" || echo "ros")/$ROS_DISTRO/setup.bash && /healthcheck_$SLAM_MODE.py"

#tip: gathering logs from healthcheck: docker inspect b39 | jq '.[0].State.Health.Log'