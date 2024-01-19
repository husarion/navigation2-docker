ARG ROS_DISTRO=humble
ARG PREFIX=

FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-core

ARG PREFIX

WORKDIR /ros2_ws

COPY ./husarion_utils/healthcheck.cpp /husarion_utils/healthcheck.cpp

# In version 1.0.12 MPPI doesn't work on RPi4 (build from source)
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
    # Create healthcheck package
    cd src/ && \
    source "/opt/$MYDISTRO/$ROS_DISTRO/setup.bash" && \
    ros2 pkg create healthcheck_pkg --build-type ament_cmake --dependencies rclcpp lifecycle_msgs nav_msgs nav2_msgs && \
    sed -i '/find_package(nav2_msgs REQUIRED)/a \
            add_executable(healthcheck_node src/healthcheck.cpp)\n \
            ament_target_dependencies(healthcheck_node rclcpp lifecycle_msgs nav_msgs nav2_msgs)\n \
            install(TARGETS \
                healthcheck_node \
                DESTINATION lib/${PROJECT_NAME})' \
            /ros2_ws/src/healthcheck_pkg/CMakeLists.txt && \
    mv /husarion_utils/healthcheck.cpp /ros2_ws/src/healthcheck_pkg/src/ && \
    cd .. && \
    # Install dependencies
    rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    # Build
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    apt install -y \
        ros-$ROS_DISTRO-navigation2 \
        ros-$ROS_DISTRO-nav2-bringup && \
    # Save version
    echo $(dpkg -s ros-$ROS_DISTRO-navigation2 | grep 'Version' | sed -r 's/Version:\s([0-9]+.[0-9]+.[0-9]+).*/\1/g') > /version.txt && \
    # Size optimalization
    rm -rf build log src && \
    export SUDO_FORCE_REMOVE=yes && \
    apt-get remove -y \
        ros-dev-tools && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY ./husarion_utils /husarion_utils
HEALTHCHECK --interval=2s --timeout=1s --start-period=45s --retries=2 \
    CMD ["/husarion_utils/healthcheck.sh"]
