ARG ROS_DISTRO=humble
ARG PREFIX=

FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-core

ENV SLAM_MODE=navigation
ENV PREFIX_ENV=$PREFIX

WORKDIR /ros2_ws

COPY ./healthcheck /healthcheck

# Install everything needed
RUN MYDISTRO=${PREFIX:-ros}; MYDISTRO=${MYDISTRO//-/} && \
    apt-get update --fix-missing && \
    apt upgrade -y && \
    apt-get install -y \
        ros-dev-tools && \
    # Clone source
    git clone -b $ROS_DISTRO https://github.com/ros-planning/navigation2.git src/navigation2 && \
    # Build MPPI
    cp -r src/navigation2/nav2_mppi_controller src/ &&\
    rm -rf src/navigation2/ && \
    # Install dependencies
    rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    # Create healthcheck package
    cd src/ && \
    source "/opt/$MYDISTRO/$ROS_DISTRO/setup.bash" && \
    ros2 pkg create healthcheck_pkg --build-type ament_cmake --dependencies rclcpp lifecycle_msgs nav2_msgs && \
    sed -i '/find_package(nav2_msgs REQUIRED)/a \
            add_executable(healthcheck_localization src/healthcheck_localization.cpp)\n \
            ament_target_dependencies(healthcheck_localization rclcpp lifecycle_msgs)\n \
            add_executable(healthcheck_navigation src/healthcheck_navigation.cpp)\n \
            ament_target_dependencies(healthcheck_navigation rclcpp lifecycle_msgs)\n \
            add_executable(healthcheck_slam src/healthcheck_slam.cpp)\n \
            ament_target_dependencies(healthcheck_slam rclcpp nav2_msgs)\n \
            install(TARGETS \
                healthcheck_localization \
                healthcheck_navigation \
                healthcheck_slam  \
                DESTINATION lib/${PROJECT_NAME})' \
            /ros2_ws/src/healthcheck_pkg/CMakeLists.txt && \
    mv /healthcheck/* /ros2_ws/src/healthcheck_pkg/src/ && \
    rm -r /healthcheck && \
    cd .. && \
    # Build
    colcon build && \
    apt install -y \
        ros-$ROS_DISTRO-navigation2 \
        ros-$ROS_DISTRO-nav2-bringup && \
    # Make the image smaller
    export SUDO_FORCE_REMOVE=yes && \
    apt-get remove -y \
        ros-dev-tools && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY ./nav2_params /nav2_params

RUN echo $(dpkg -s ros-$ROS_DISTRO-navigation2 | grep 'Version' | sed -r 's/Version:\s([0-9]+.[0-9]+.[0-9]+).*/\1/g') > /version.txt

RUN sed -i '/test -f "\/ros2_ws\/install\/setup.bash" && source "\/ros2_ws\/install\/setup.bash"/a \
        ros2 run healthcheck_pkg "healthcheck_$SLAM_MODE" &' \
        /ros_entrypoint.sh

COPY ./healthcheck/healthcheck.sh /
HEALTHCHECK --interval=7s --timeout=2s  --start-period=5s --retries=5 \
    CMD ["/healthcheck.sh"]

# #tip: gathering logs from healthcheck: docker inspect b39 | jq '.[0].State.Health.Log'
