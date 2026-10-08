cmake_minimum_required(VERSION 3.15)
project(tf_manager)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(calibration_tf_broadcaster
  src/calibration_tf_broadcaster.cpp)
ament_target_dependencies(calibration_tf_broadcaster
  geometry_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  yaml-cpp)
target_link_libraries(calibration_tf_broadcaster yaml-cpp)

add_executable(tf_matrix_consumer src/tf_matrix_consumer.cpp)
ament_target_dependencies(tf_matrix_consumer rclcpp tf2 tf2_ros)

add_executable(pointcloud_tf_transformer
  src/pointcloud_tf_transformer.cpp)
ament_target_dependencies(pointcloud_tf_transformer
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  yaml-cpp)
target_link_libraries(pointcloud_tf_transformer yaml-cpp)

install(TARGETS
  calibration_tf_broadcaster
  tf_matrix_consumer
  pointcloud_tf_transformer
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pointcloud_consumer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # ament_uncrustify enforces ROS's Allman-brace profile; this package uses
  # the Google C++ profile declared in .clang-format instead.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
