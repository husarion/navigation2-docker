ARG ROS_DISTRO=humble
ARG PREFIX=

FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-base

SHELL ["/bin/bash", "-c"]

ARG PREFIX
ENV SLAM_MODE=navigation
ENV PREFIX_ENV=$PREFIX

RUN apt update && apt upgrade -y && apt install -y \
        ros-$ROS_DISTRO-navigation2 \
        ros-$ROS_DISTRO-nav2-bringup \
        ros-$ROS_DISTRO-pointcloud-to-laserscan && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY ./nav2_params /nav2_params

# Create healthcheck package
WORKDIR /ros2_ws

RUN mkdir src && cd src/ && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    ros2 pkg create healthcheck_pkg --build-type ament_cmake --dependencies rclcpp std_msgs && \
    sed -i '/find_package(std_msgs REQUIRED)/a \
            find_package(geometry_msgs REQUIRED)\n \
            find_package(nav2_msgs REQUIRED)\n \
            find_package(bond REQUIRED)\n \
            add_executable(healthcheck_localization src/healthcheck_localization.cpp)\n \
            ament_target_dependencies(healthcheck_localization rclcpp geometry_msgs)\n \
            add_executable(healthcheck_navigation src/healthcheck_navigation.cpp)\n \
            ament_target_dependencies(healthcheck_navigation rclcpp bond)\n \
            add_executable(healthcheck_slam src/healthcheck_slam.cpp)\n \
            ament_target_dependencies(healthcheck_slam rclcpp nav2_msgs)\n \
            install(TARGETS \
                healthcheck_localization \
                healthcheck_navigation \
                healthcheck_slam  \
                DESTINATION lib/${PROJECT_NAME})' \
            /ros2_ws/src/healthcheck_pkg/CMakeLists.txt

COPY ./healthcheck /ros2_ws/src/healthcheck_pkg/src

# Build
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release

RUN echo $(dpkg -s ros-$ROS_DISTRO-navigation2 | grep 'Version' | sed -r 's/Version:\s([0-9]+.[0-9]+.[0-9]+).*/\1/g') > /version.txt

COPY ./healthcheck/healthcheck.sh /
HEALTHCHECK --interval=10s --timeout=2s --start-period=5s --retries=5 \
    CMD ["/healthcheck.sh"]

#tip: gathering logs from healthcheck: docker inspect b39 | jq '.[0].State.Health.Log'
