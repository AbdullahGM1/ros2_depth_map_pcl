cmake_minimum_required(VERSION 3.8)
project(ros2_depth_map_pcl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Disable pedantic warnings for anonymous structs
add_compile_options(-Wno-gnu-anonymous-struct -Wno-nested-anon-types)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)

# Include Eigen3 directory
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${pcl_conversions_INCLUDE_DIRS})

add_executable(depth_map src/depth_map.cpp)
target_include_directories(depth_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(depth_map PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  depth_map
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
  "OpenCV"
  "PCL"
)

install(TARGETS depth_map
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
