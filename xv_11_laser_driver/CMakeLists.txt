cmake_minimum_required(VERSION 3.5)
project(xv_11_laser_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencias de ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Boost para manejo de puerto serie
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

# Ejecutable principal
add_executable(neato_laser_publisher
  src/neato_laser_publisher.cpp
  src/xv11_laser.cpp
)

ament_target_dependencies(neato_laser_publisher
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(neato_laser_publisher
  ${Boost_LIBRARIES}
)

# Instalación del ejecutable
install(TARGETS neato_laser_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Instalación de headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

ament_package()
