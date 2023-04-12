ARG ROS_DISTRO=humble
ARG PREFIX=

FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-core

ARG PREFIX
ENV SLAM_MODE=localization
ENV PREFIX_ENV=$PREFIX

SHELL ["/bin/bash", "-c"]

RUN apt update && apt upgrade -y && \
    # install build tools
    apt install -y \
        git \
        build-essential \
        python3-rosdep \
        python3-colcon-common-extensions \
        ros-$ROS_DISTRO-pointcloud-to-laserscan && \
    # building nav2
    git clone --branch $ROS_DISTRO https://github.com/ros-planning/navigation2.git src/ && \
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install -y -r -q --from-paths src --rosdistro $ROS_DISTRO && \
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    # clean to make the image smaller
	apt remove -y \
        git \
        build-essential \
        python3-rosdep \
        python3-colcon-common-extensions && \
    apt autoremove -y && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

COPY ./nav2_params /nav2_params

COPY healthcheck_* /

RUN echo $(dpkg -s ros-$ROS_DISTRO-navigation2 | grep 'Version' | sed -r 's/Version:\s([0-9]+.[0-9]+.[0-9]*).*/\1/g') >> /version.txt

HEALTHCHECK --interval=10s --timeout=10s --start-period=5s --retries=6  \
    CMD bash -c "MYDISTRO=${PREFIX_ENV:-ros}; MYDISTRO=${MYDISTRO//-/} && source /opt/$MYDISTRO/$ROS_DISTRO/setup.bash && /healthcheck_$SLAM_MODE.py"

