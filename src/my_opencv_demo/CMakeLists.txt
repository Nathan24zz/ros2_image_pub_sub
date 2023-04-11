cmake_minimum_required(VERSION 3.5)
project(my_opencv_demo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(my_opencv_demo_interfaces REQUIRED)
find_package(shisen_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(minimal_opencv_ros2_node src/minimal_opencv_ros2_node.cpp)
ament_target_dependencies(minimal_opencv_ros2_node rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV my_opencv_demo_interfaces shisen_interfaces)

add_executable(shisen_pub src/shisen_pub.cpp)
ament_target_dependencies(shisen_pub rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV my_opencv_demo_interfaces shisen_interfaces)

add_executable(my_sub src/my_sub.cpp)
ament_target_dependencies(my_sub rclcpp std_msgs sensor_msgs my_opencv_demo_interfaces OpenCV cv_bridge shisen_interfaces)

install(TARGETS
   minimal_opencv_ros2_node
   my_sub
   shisen_pub
   DESTINATION lib/${PROJECT_NAME}
 )

ament_package()
