cmake_minimum_required(VERSION 3.8)
project(ros2_depth_map_pcl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Qhull REQUIRED)  # Add Qhull dependency

# Include PCL, OpenCV, and Qhull directories
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${QHULL_INCLUDE_DIRS})

# Add executable for the depth map node
add_executable(depth_map src/depth_map.cpp)

# Link libraries and dependencies
target_link_libraries(depth_map ${PCL_LIBRARIES} ${OpenCV_LIBS} ${QHULL_LIBRARIES})

# Specify compile features and standards
target_compile_features(depth_map PUBLIC c_std_99 cxx_std_17)

# Specify dependencies for ROS2 and PCL
ament_target_dependencies(
  depth_map
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  PCL
  pcl_conversions
  pcl_ros
)

# Install the target
install(TARGETS depth_map
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
