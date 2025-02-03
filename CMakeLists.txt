cmake_minimum_required(VERSION 3.0.2)
project(vehicle_diagnostics_tool)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  NodeStatus.msg
  # Message1.msg
  # Message2.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vehicle_diag
#  CATKIN_DEPENDS rospy_std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
)

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
  /usr/local/cuda-12.6/targets/x86_64-linux/include  # NVML header path
)

link_directories(
  /usr/local/cuda-12.6/targets/x86_64-linux/lib/stubs  # NVML library path
)


add_executable(sys_status_node scripts/cpp/sys_status.cpp)
add_executable(sys_status_def_node scripts/cpp/sys_status_def.cpp)
add_executable(random_float_pub_node scripts/cpp/random_float_pub.cpp)

add_executable(total_resource_cpp scripts/cpp/total_resource.cpp)

target_link_libraries(sys_status_node ${catkin_LIBRARIES} nvidia-ml)
target_link_libraries(sys_status_def_node ${catkin_LIBRARIES} nvidia-ml)
target_link_libraries(random_float_pub_node ${catkin_LIBRARIES})

target_link_libraries(total_resource_cpp ${catkin_LIBRARIES} nvidia-ml)
