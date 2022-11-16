ARG ROS_DISTRO=galactic

FROM ros:$ROS_DISTRO-ros-core

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
        ros-$ROS_DISTRO-rmw-fastrtps-cpp \
        ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
        ros-$ROS_DISTRO-navigation2 \
        ros-$ROS_DISTRO-nav2-bringup && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY ./nav2_params /nav2_params

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

RUN if [[ $ROS_DISTRO == "humble" ]] ; then sed -i 's/robot_model_type: "differential"/robot_model_type: nav2_amcl::DifferentialMotionModel/g' /nav2_params/rosbot2_amcl.yaml ; fi

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc