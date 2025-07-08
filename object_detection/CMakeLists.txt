cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW) # Suppress PCL_ROOT warning

project(object_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io search filters)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(cone_detection_pcl src/cone_detection_pcl.cpp)
ament_target_dependencies(cone_detection_pcl rclcpp sensor_msgs visualization_msgs tf2 tf2_ros pcl_conversions pcl_msgs)
target_link_libraries(cone_detection_pcl ${PCL_LIBRARIES})
install(TARGETS cone_detection_pcl DESTINATION lib/${PROJECT_NAME})

add_executable(cone_detection_node src/cone_detection_node.cpp)
ament_target_dependencies(cone_detection_node rclcpp sensor_msgs visualization_msgs tf2_ros)
install(TARGETS cone_detection_node DESTINATION lib/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
