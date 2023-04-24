ARG ROS_DISTRO=humble
ARG PREFIX=

FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-core

SHELL ["/bin/bash", "-c"]

ARG PREFIX
ENV SLAM_MODE=localization
ENV PREFIX_ENV=$PREFIX

RUN apt update && apt upgrade -y && \
    # install build tools
    apt install -y \
        git \
        build-essential \
        python3-rosdep \
        python3-colcon-common-extensions \
        ros-$ROS_DISTRO-pointcloud-to-laserscan && \
    # building nav2
    source /opt/$([ -n "${PREFIX_ENV}" ] && echo "${PREFIX_ENV//-/}" || echo "ros")/$ROS_DISTRO/setup.bash && \
    git clone --branch $ROS_DISTRO https://github.com/ros-planning/navigation2.git src/ && \
    rm -rf  /etc/ros/rosdep/sources.list.d/20-default.list && \
    git -C src/ sparse-checkout set \
        navigation2/** \
        nav2_bringup/** \
        nav2_msgs/** \
        nav2_mppi_controller/**  && \
    rosdep init && \
    rosdep install -y -r -q --from-paths src --rosdistro $ROS_DISTRO && \
    colcon build --symlink-install && \
    # clean to make the image smaller
    export SUDO_FORCE_REMOVE=yes && \
	apt remove -y \
        git \
        build-essential \
        python3-rosdep \
        python3-colcon-common-extensions && \
    apt autoremove -y && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

COPY ./nav2_params /nav2_params

COPY --chmod=755 healthcheck_* /

RUN echo $(cat /ros2_ws/src/navigation2/package.xml | grep '<version>' | sed -r 's/.*<version>([0-9]+.[0-9]+.[0-9]+)<\/version>/\1/g') > /version.txt

HEALTHCHECK --interval=10s --timeout=10s --start-period=5s --retries=6  \
    CMD bash -c "source /opt/$([ -n "${PREFIX_ENV}" ] && echo "${PREFIX_ENV//-/}" || echo "ros")/$ROS_DISTRO/setup.bash && /healthcheck_$SLAM_MODE.py"

#tip: gathering logs from healthcheck: docker inspect b39 | jq '.[0].State.Health.Log'