cmake_minimum_required(VERSION 3.8)
project(gazebo_4d_radar_plugin)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(radar_physics_core REQUIRED)

# --- Gazebo libraries ---------------------------------------------------------
# ROS 2 Jazzy+ ships Gazebo through *_vendor packages, which expose unversioned
# CMake packages (gz-sim, gz-common, ...). Fall back to versioned system installs.
find_package(gz_sim_vendor QUIET)
if(gz_sim_vendor_FOUND)
  find_package(gz_common_vendor REQUIRED)
  find_package(gz_plugin_vendor REQUIRED)
  find_package(gz_math_vendor REQUIRED)
  find_package(sdformat_vendor REQUIRED)
  find_package(gz-sim REQUIRED)
  find_package(gz-common REQUIRED COMPONENTS graphics)
  find_package(gz-plugin REQUIRED COMPONENTS register)
  find_package(gz-math REQUIRED)
  find_package(sdformat REQUIRED)
  set(GZ_LIBS gz-sim::gz-sim gz-common::graphics gz-plugin::register gz-math::gz-math
      sdformat::sdformat)
else()
  find_package(gz-sim8 QUIET)
  if(gz-sim8_FOUND)
    find_package(gz-common5 REQUIRED COMPONENTS graphics)
    find_package(gz-plugin2 REQUIRED COMPONENTS register)
    find_package(gz-math7 REQUIRED)
    find_package(sdformat14 REQUIRED)
    set(GZ_LIBS gz-sim8::gz-sim8 gz-common5::graphics gz-plugin2::register gz-math7::gz-math7
        sdformat14::sdformat14)
  else()
    message(FATAL_ERROR "Gazebo Sim not found: install gz_sim_vendor (ROS 2 Jazzy) or gz-sim8 (Harmonic)")
  endif()
endif()

add_library(${PROJECT_NAME} SHARED
  src/Gazebo4DRadarPlugin.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_link_libraries(${PROJECT_NAME}
  ${GZ_LIBS}
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${radar_msgs_TARGETS}
  radar_physics_core::radar_physics_core
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY models worlds launch rviz DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(PROGRAMS
  scripts/radar_ego_velocity_estimator.py
  scripts/validate_radar_plugin.py
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Make `gz sim` find the plugin and the bundled models once the workspace is sourced.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp std_msgs sensor_msgs geometry_msgs radar_msgs radar_physics_core)
ament_package()
