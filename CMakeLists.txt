cmake_minimum_required(VERSION 3.5)
project(seam_synchronizer)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
  rclcpp
  message_filters
  sensor_msgs
)

add_executable(validation_seam src/validation_seam.cpp)
target_include_directories(validation_seam PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
ament_target_dependencies(validation_seam
  "rclcpp"
  "message_filters"
  "sensor_msgs"
  )

add_executable(validation_seam_computetime src/validation_seam_computetime.cpp)
target_include_directories(validation_seam_computetime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
ament_target_dependencies(validation_seam_computetime
  "rclcpp"
  "message_filters"
  "sensor_msgs"
  )



install(TARGETS 
  validation_seam
  validation_seam_computetime
  DESTINATION lib/${PROJECT_NAME})


# Install files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
