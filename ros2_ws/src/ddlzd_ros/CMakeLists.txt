cmake_minimum_required(VERSION 3.8)
project(ddlzd_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ddlzd_core REQUIRED)
find_package(ddlzd_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(live_landing_zone_node
  src/live_landing_zone_main.cpp
  src/live_landing_zone_node.cpp
  src/semantic_projector.cpp
)
target_include_directories(live_landing_zone_node PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(live_landing_zone_node
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
ament_target_dependencies(live_landing_zone_node
  cv_bridge ddlzd_core ddlzd_msgs diagnostic_msgs geometry_msgs pcl_conversions
  rclcpp rclcpp_lifecycle sensor_msgs std_msgs tf2_eigen tf2_ros tf2_sensor_msgs
  visualization_msgs
)
target_compile_options(live_landing_zone_node PRIVATE
  ${PCL_DEFINITIONS} -Wall -Wextra -Wpedantic)

install(TARGETS live_landing_zone_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_semantic_projector
    test/test_semantic_projector.cpp
    src/semantic_projector.cpp
  )
  target_include_directories(test_semantic_projector PRIVATE
    include ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_semantic_projector
    Eigen3::Eigen ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
  target_compile_options(test_semantic_projector PRIVATE ${PCL_DEFINITIONS})
  ament_target_dependencies(test_semantic_projector ddlzd_core sensor_msgs)
endif()

ament_package()
