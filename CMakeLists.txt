cmake_minimum_required(VERSION 2.8.3)
project(rtimulib_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  rosflight_msgs
  std_srvs
)

catkin_package(
    LIBRARIES rtimulib_ros
    CATKIN_DEPENDS roscpp sensor_msgs rosflight_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rtimulib_ros src/rtimulib_ros.cpp)

target_link_libraries(rtimulib_ros
  ${catkin_LIBRARIES}
  RTIMULib
)

add_executable(rtimu_node src/rtimu_node.cpp src/RTIMU_Node.cpp)

target_link_libraries(rtimu_node
  ${catkin_LIBRARIES}
  RTIMULib
)
