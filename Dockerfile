ARG ROS_DISTRO=humble
ARG PREFIX=

FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-base

SHELL ["/bin/bash", "-c"]

ARG PREFIX
ENV SLAM_MODE=
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
            add_executable(healthcheck_localization src/healthcheck_localization.cpp)\n \
            ament_target_dependencies(healthcheck_localization rclcpp std_msgs geometry_msgs)\n \
            add_executable(healthcheck_slam src/healthcheck_slam.cpp)\n \
            ament_target_dependencies(healthcheck_slam rclcpp std_msgs nav2_msgs)\n \
            add_executable(healthcheck_ src/healthcheck_navigation.cpp)\n \
            ament_target_dependencies(healthcheck_ rclcpp std_msgs geometry_msgs)\n \
            install(TARGETS \
                healthcheck_localization \
                healthcheck_slam  \
                healthcheck_ \
                DESTINATION lib/${PROJECT_NAME})' \
            /ros2_ws/src/healthcheck_pkg/CMakeLists.txt

COPY ./healthcheck_localization.cpp /ros2_ws/src/healthcheck_pkg/src/
COPY ./healthcheck_slam.cpp /ros2_ws/src/healthcheck_pkg/src/
COPY ./healthcheck_navigation.cpp /ros2_ws/src/healthcheck_pkg/src/

# Build
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release

RUN echo $(dpkg -s ros-$ROS_DISTRO-navigation2 | grep 'Version' | sed -r 's/Version:\s([0-9]+.[0-9]+.[0-9]+).*/\1/g') > /version.txt

HEALTHCHECK --interval=15s --timeout=5s --start-period=10s --retries=3 \
    CMD bash -c "/ros_entrypoint.sh ros2 run healthcheck_pkg healthcheck_$SLAM_MODE"

#tip: gathering logs from healthcheck: docker inspect b39 | jq '.[0].State.Health.Log'
