cmake_minimum_required(VERSION 3.20)
project(eddyfi_ectane)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################
##              Static  Analysis              ##
################################################

option(ENABLE_CLANGTIDY "Run clang-tidy" OFF)

if(ENABLE_CLANGTIDY)
  find_program(CLANG_TIDY_EXE NAMES "clang-tidy")
  set(CLANG_TIDY_COMMAND "${CLANG_TIDY_EXE}" "-checks=-*,modernize-*")
endif(ENABLE_CLANGTIDY)

option(ENABLE_CPPCHECK "Run cppcheck" OFF)

if(ENABLE_CPPCHECK)
    find_program(cppcheck cppcheck)
    message(STATUS "cppcheck                   ${cppcheck}")
    if(NOT (cppcheck MATCHES "NOTFOUND"))
        set(CMAKE_CXX_CPPCHECK "${cppcheck}"
            "--enable=all"
            "--inconclusive"
            "--inline-suppr"
            "--quiet"
            "--suppress=unmatchedSuppression"
            "--suppress=unusedFunction"
            "--template='{file}:{line}: warning: {id} ({severity}): {message}'")
    endif()
endif(ENABLE_CPPCHECK)

################################################
##               Dependencies                 ##
################################################

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(Boost REQUIRED CONFIG)
find_package(Threads REQUIRED)

# Socket classes for the C SDK's connections, pinned to a release so the build cannot change underneath the driver
include(FetchContent)
FetchContent_Declare(
  simple_boost_asio_sockets
  GIT_REPOSITORY https://github.com/MShields1986/boost_asio_socket_wrappers
  GIT_TAG        v0.1.0
)
FetchContent_MakeAvailable(simple_boost_asio_sockets)

################################################
##          Messages and Services             ##
################################################

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Frequency.msg"
  "msg/Channel.msg"
  "msg/TimeSlot.msg"
  "msg/Observation.msg"
  "srv/BalanceDevice.srv"
  "srv/StreamData.srv"
  DEPENDENCIES std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

################################################
##                Components                  ##
################################################

# Components replace the ROS 1 nodelets. Each one can be loaded into a component container,
# or run on its own with the executable registered for it below.
add_library(${PROJECT_NAME}_components SHARED
  src/ectane_handler.cpp
  src/ectane_node.cpp
  src/ec_data_to_pointcloud_node.cpp
  src/reconstruction_node.cpp
)

target_include_directories(${PROJECT_NAME}_components PRIVATE include)

target_link_libraries(${PROJECT_NAME}_components
  "${cpp_typesupport_target}"
  rclcpp::rclcpp
  rclcpp_components::component
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  Boost::headers
  Threads::Threads
  BoostSocketWrappers
)

# Humble's tf2_sensor_msgs exports its include directories but no target
ament_target_dependencies(${PROJECT_NAME}_components tf2_sensor_msgs)

if(ENABLE_CLANGTIDY)
  set_target_properties(${PROJECT_NAME}_components PROPERTIES CXX_CLANG_TIDY "${CLANG_TIDY_COMMAND}")
endif(ENABLE_CLANGTIDY)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "ectane_namespace::EctaneNode"
  EXECUTABLE ectane_node
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "ec_data_to_pointcloud_namespace::EcDataToPointCloudNode"
  EXECUTABLE ec_data_to_pointcloud_node
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "reconstruction_namespace::ReconstructionNode"
  EXECUTABLE reconstruction_node
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
