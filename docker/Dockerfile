##############
# Base image #
##############
FROM ros:jazzy-ros-base AS base
# FROM osrf/ros:jazzy-desktop-full AS base   # With RViz

ARG ROS2_WORKSPACE_DIR=/ros2_ws

SHELL ["/bin/bash", "-c"]

COPY eddyfi_ectane ${ROS2_WORKSPACE_DIR}/src/eddyfi_ectane_ros/eddyfi_ectane

WORKDIR ${ROS2_WORKSPACE_DIR}

RUN source /opt/ros/jazzy/setup.bash \
 && apt-get update \
 && rosdep update --rosdistro jazzy \
 && rosdep install -i --from-path src --rosdistro jazzy -y \
 && rm -rf /var/lib/apt/lists/* \
 && colcon build --event-handlers console_direct+ \
 && echo "source ${ROS2_WORKSPACE_DIR}/install/setup.bash" >> /ros_entrypoint.sh

RUN sed -i '/exec "$@"/d' /ros_entrypoint.sh \
 && echo 'exec "$@"' >> /ros_entrypoint.sh

ENTRYPOINT [ "/ros_entrypoint.sh" ]


##############
# Test image #
##############
FROM base AS test

ARG ROS2_WORKSPACE_DIR=/ros2_ws

# The tests are run at container start, by the docker-compose test service
ENV TEST_LOG_DIR=/test_logs
