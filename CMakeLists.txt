cmake_minimum_required(VERSION 3.8)
project(cigarette_dispenser_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(tf2_geometry_msgs REQUIRED)

# Include Cpp “include” directory
include_directories(include)


# src下のソースコードはTARGETSで
# scripts下のソースコードはPROGRAMSで
# launch下のlaunchファイルはDIRECTORYで

# Create Cpp executable and link with dependencies
##add_executable(cpp_executable src/cpp_node.cpp)
##ament_target_dependencies(cpp_executable rclcpp)

add_executable(cpp_listener src/subscriber_cpp.cpp)
target_link_libraries(cpp_listener PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS})

# Install Cpp executables in the ros2_ws/install/cpp_py_pkg/lib/cpp_py_pkg/ folder
install(TARGETS#cpp_executable
  cpp_listener
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(motion src/motion.cpp)
ament_target_dependencies(motion
    cv_bridge
    geometry_msgs
    image_geometry
    moveit_ros_planning_interface
    OpenCV
    rclcpp
    tf2_geometry_msgs
    std_msgs
)
install(TARGETS
  motion 
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables in the ros2_ws/install/cpp_py_pkg/lib/cpp_py_pkg/ folder
install(PROGRAMS
  scripts/detection.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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
