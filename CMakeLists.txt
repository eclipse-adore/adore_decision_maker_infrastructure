cmake_minimum_required(VERSION 3.8)
project(decision_maker_infrastructure VERSION 0.1.0 LANGUAGES CXX)

# C++20 strict mode
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build flags
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(adore_math REQUIRED)
find_package(adore_planning REQUIRED)

find_package(adore_map REQUIRED)
find_package(adore_ros2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(adore_dynamics_conversions REQUIRED)


# Add any additional adore_* or ROS 2 dependencies here

# Executable node (main + infra)
add_executable(${PROJECT_NAME}
  src/main.cpp
  src/decision_maker_infrastructure.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  adore_math
  adore_map
  adore_ros2_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  adore_dynamics_conversions
  adore_planning
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Component library (without main.cpp)
add_library(${PROJECT_NAME}_component SHARED
  src/decision_maker_infrastructure.cpp
)

target_include_directories(${PROJECT_NAME}_component
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  adore_math
  adore_map
  adore_ros2_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  adore_dynamics_conversions
  adore_planning
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "adore::DecisionMakerInfrastructure"
  EXECUTABLE decisionmakerinfrastructure
)

install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
