##############
# Base image #
##############
FROM ros:noetic-ros-base AS base
# FROM osrf/ros:noetic-desktop-full AS base   # With RViz

ARG CATKIN_WORKSPACE_DIR=/catkin_ws

SHELL ["/bin/bash", "-c"]

COPY eddyfi_ectane ${CATKIN_WORKSPACE_DIR}/src/eddyfi_ectane_ros/eddyfi_ectane

WORKDIR ${CATKIN_WORKSPACE_DIR}

RUN source /opt/ros/noetic/setup.bash \
 && apt-get update \
 && rosdep update --rosdistro noetic \
 && rosdep install -i --from-path src --rosdistro noetic -y \
 && rm -rf /var/lib/apt/lists/* \
 && catkin_make \
 && echo "source ${CATKIN_WORKSPACE_DIR}/devel/setup.bash" >> /ros_entrypoint.sh

RUN sed -i '/exec "$@"/d' /ros_entrypoint.sh \
 && echo 'exec "$@"' >> /ros_entrypoint.sh

ENTRYPOINT [ "/ros_entrypoint.sh" ]


##############
# Test image #
##############
FROM base AS test

ARG CATKIN_WORKSPACE_DIR=/catkin_ws

# The tests are run at container start, by the docker-compose test service
ENV TEST_LOG_DIR=/test_logs
