cmake_minimum_required(VERSION 3.8)
project(gazebo_4d_radar_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(radar_physics_core REQUIRED)

# Gazebo Sim discovery (supporting gz-sim8 / gz-sim7 / gz-sim6)
find_package(gz-sim8 QUIET)
if(NOT gz-sim8_FOUND)
  find_package(gz-sim7 QUIET)
  if(NOT gz-sim7_FOUND)
    find_package(gz-sim QUIET)
  endif()
endif()

find_package(gz-plugin2 QUIET)
if(NOT gz-plugin2_FOUND)
  find_package(gz-plugin QUIET)
endif()

find_package(gz-math7 QUIET)
if(NOT gz-math7_FOUND)
  find_package(gz-math QUIET)
endif()

add_library(${PROJECT_NAME} SHARED
  src/Gazebo4DRadarPlugin.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  radar_msgs
  radar_physics_core
)

# Link Gazebo libraries
if(TARGET gz-sim8::gz-sim8)
  target_link_libraries(${PROJECT_NAME} gz-sim8::gz-sim8 gz-plugin2::gz-plugin2 gz-math7::gz-math7)
elseif(TARGET gz-sim7::gz-sim7)
  target_link_libraries(${PROJECT_NAME} gz-sim7::gz-sim7 gz-plugin2::gz-plugin2 gz-math7::gz-math7)
elseif(TARGET gz-sim::gz-sim)
  target_link_libraries(${PROJECT_NAME} gz-sim::gz-sim gz-plugin::gz-plugin gz-math::gz-math)
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY models worlds launch rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp std_msgs sensor_msgs radar_msgs radar_physics_core)
ament_package()
