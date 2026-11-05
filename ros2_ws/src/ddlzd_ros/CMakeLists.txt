cmake_minimum_required(VERSION 3.8)
project(ddlzd_ros)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ddlzd_core REQUIRED)
find_package(ddlzd_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Ubuntu 22.04's libgeographic-dev provides headers and a library but no
# GeographicLibConfig.cmake, so discover those two artifacts directly.
find_path(GEOGRAPHICLIB_INCLUDE_DIR GeographicLib/Geodesic.hpp REQUIRED)
find_library(GEOGRAPHICLIB_LIBRARY NAMES GeographicLib Geographic REQUIRED)

add_library(ddlzd_ros_support
  src/geodesy.cpp
  src/sticky_selector.cpp
)
target_include_directories(ddlzd_ros_support PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GEOGRAPHICLIB_INCLUDE_DIR}
)
target_link_libraries(ddlzd_ros_support
  ${GEOGRAPHICLIB_LIBRARY}
)
ament_target_dependencies(ddlzd_ros_support ddlzd_core)
target_compile_options(ddlzd_ros_support PRIVATE -Wall -Wextra -Wpedantic)

add_executable(live_landing_zone_node
  src/live_landing_zone_main.cpp
  src/live_landing_zone_node.cpp
)
target_include_directories(live_landing_zone_node PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(live_landing_zone_node
  ddlzd_ros_support
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)
ament_target_dependencies(live_landing_zone_node
  ddlzd_core ddlzd_msgs diagnostic_msgs geographic_msgs geometry_msgs pcl_conversions
  rclcpp rclcpp_lifecycle sensor_msgs std_msgs tf2_eigen tf2_geometry_msgs tf2_ros tf2_sensor_msgs
  visualization_msgs
)
target_compile_options(live_landing_zone_node PRIVATE
  ${PCL_DEFINITIONS} -Wall -Wextra -Wpedantic)

install(TARGETS ddlzd_ros_support live_landing_zone_node
  EXPORT export_ddlzd_ros_support
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/detect_bag_sensor.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sticky_selector test/test_sticky_selector.cpp)
  target_link_libraries(test_sticky_selector ddlzd_ros_support)
  ament_target_dependencies(test_sticky_selector ddlzd_core)

  ament_add_gtest(test_geodesy test/test_geodesy.cpp)
  target_link_libraries(test_geodesy ddlzd_ros_support ${GEOGRAPHICLIB_LIBRARY})
  target_include_directories(test_geodesy PRIVATE ${GEOGRAPHICLIB_INCLUDE_DIR})
endif()

ament_export_targets(export_ddlzd_ros_support HAS_LIBRARY_TARGET)

ament_package()
