cmake_minimum_required(VERSION 3.5)
project(ros2_orbslam)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ORB_SLAM2 REQUIRED)

add_executable(mono 
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)
target_include_directories(mono PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

target_link_libraries(mono PRIVATE
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
  cv_bridge::cv_bridge
  ${OpenCV_LIBS}
  ${ORB_SLAM2_LIBRARIES}
  ${Pangolin_LIBRARIES}
)

# Treat external ORB_SLAM2 headers as system includes to reduce third-party warning noise.
target_include_directories(mono SYSTEM PRIVATE ${ORB_SLAM2_INCLUDE_DIRS})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(mono PRIVATE
    -Wno-overloaded-virtual
    -Wno-deprecated-declarations
    -Wno-pedantic
  )
endif()

install(TARGETS mono
  DESTINATION lib/${PROJECT_NAME})

ament_package()
