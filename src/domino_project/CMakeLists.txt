cmake_minimum_required(VERSION 3.5)
project(domino_project)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --- NODO C++ ---
# Usiamo il percorso dove hai attualmente il file
add_executable(robot_mover_cpp domino_project/robot_mover.cpp)
ament_target_dependencies(robot_mover_cpp
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
)

# --- QUESTA È LA PARTE CHE MANCAVA! ---
install(TARGETS
  robot_mover_cpp
  DESTINATION lib/${PROJECT_NAME}
)
# --------------------------------------

# --- SCRIPT PYTHON (Visione) ---
install(PROGRAMS
  domino_project/vision_processor.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- FILE CONFIGURAZIONE E LAUNCH ---
install(DIRECTORY
  launch
  models
  worlds
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()