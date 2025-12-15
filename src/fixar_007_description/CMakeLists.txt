cmake_minimum_required(VERSION 3.8)
project(fixar_007_description VERSION 1.0)

# ==============================================================================
# 1. GLOBAL SETTINGS
# ==============================================================================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==============================================================================
# 2. DEPENDENCIES
# ==============================================================================
# ROS 2 Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)

# Shared Dependency (JSON)
find_package(nlohmann_json REQUIRED)

# ==============================================================================
# 3. EXECUTABLE A: THE BRIDGE (ROS Node)
# ==============================================================================
# This connects Gazebo to the Network. It NEEDS ROS.
add_executable(fixar_windows_bridge_cpp
  src/fixar_windows_bridge.cpp
)

target_include_directories(fixar_windows_bridge_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link ROS libraries
ament_target_dependencies(fixar_windows_bridge_cpp
  rclcpp
  sensor_msgs
  nav_msgs
  actuator_msgs
)

# Link JSON
target_link_libraries(fixar_windows_bridge_cpp
  nlohmann_json::nlohmann_json
)

# ==============================================================================
# 4. EXECUTABLE B: THE CLIENT (Standalone C++)
# ==============================================================================
# This simulates the Windows App. It does NOT use ROS.
add_executable(fixar_flight_controller
  src/windows_flight_controller.cpp
)

# FIXED: Added '/include' so the compiler can find windows_flight_controller.hpp
target_include_directories(fixar_flight_controller PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link JSON only (No ROS dependencies!)
target_link_libraries(fixar_flight_controller
  nlohmann_json::nlohmann_json
)

# Platform-specific linking (Winsock for Windows, Pthread for Linux)
if(WIN32)
  target_link_libraries(fixar_flight_controller ws2_32)
else()
  target_link_libraries(fixar_flight_controller pthread)
endif()

# ==============================================================================
# 5. INSTALLATION
# ==============================================================================

# Install BOTH executables to the package lib folder
install(TARGETS 
  fixar_windows_bridge_cpp
  fixar_flight_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install standard folders
install(DIRECTORY
  launch
  urdf
  meshes
  config
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

# Install Python Scripts
install(PROGRAMS
  scripts/motor_controller.py
  scripts/motor_command_translator.py
  scripts/pwm_to_thrust.py
  scripts/windows_bridge.py
  scripts/test_windows_client.py
  DESTINATION lib/${PROJECT_NAME}
)

# ==============================================================================
# 6. TESTING & PACKAGING
# ==============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
