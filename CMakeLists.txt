cmake_minimum_required(VERSION 3.8)
project(bots)

# Muthanna Alwahash 2022

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# Common Packages
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# C++ packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

# Python packages
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY 
    config 
    gazebo
    include 
    launch 
    meshes 
    models 
    r2c 
    rviz 
    scripts 
    urdf 
    worlds 
    xacro

  DESTINATION 
    share/${PROJECT_NAME}
)

# C++ Executables
add_executable(gripper_effort src/gripper_effort.cpp)
ament_target_dependencies( gripper_effort
  rclcpp
  std_msgs
)

add_executable(cpp_node src/cpp_node.cpp)
ament_target_dependencies( cpp_node 
rclcpp
)

add_executable(cart_position src/cart_position.cpp)
ament_target_dependencies( cart_position
  rclcpp
  rclcpp_action
  control_msgs
)

add_executable(cart_velocity src/cart_velocity.cpp)
ament_target_dependencies( cart_velocity
  rclcpp
  std_msgs
)

add_executable(cart_effort src/cart_effort.cpp)
ament_target_dependencies( cart_effort
  rclcpp
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

# Install C++ Modules
install(TARGETS 
  
  cpp_node
  gripper_effort
  cart_position 
  cart_velocity 
  cart_effort 
  
  DESTINATION 
    lib/${PROJECT_NAME}
)

# Install Python Modules
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS

  scripts/py_node.py
  scripts/gripper_effort.py
  scripts/cart_effort.py
  scripts/cart_velocity.py

  DESTINATION 
    lib/${PROJECT_NAME}
)


ament_package()
