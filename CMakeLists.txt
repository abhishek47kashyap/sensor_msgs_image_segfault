cmake_minimum_required(VERSION 3.14.4)
project(segfault_pkg)

# https://github.com/ros2/python_cmake_module/issues/18
# https://github.com/ros2/python_cmake_module/issues/16#issuecomment-2363866530
if(POLICY CMP0148) 
  cmake_policy(SET CMP0148 OLD)
endif()

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-g) # for gdb: https://docs.nav2.org/tutorials/docs/get_backtrace.html

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
# for stack trace pretty-printing: https://github.com/pal-robotics/backward_ros/tree/foxy-devel
find_package(backward_ros REQUIRED)

set(dependencies
  std_msgs
  geometry_msgs
  sensor_msgs
  message_filters
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetRGBD.srv"
  DEPENDENCIES ${dependencies}
 )
ament_export_dependencies(rosidl_default_runtime)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})

# https://github.com/ros2/ros2/issues/284#issuecomment-2548345003
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}") 

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
