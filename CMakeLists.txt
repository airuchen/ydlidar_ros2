cmake_minimum_required(VERSION 3.5)
project(ydlidar)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

#add_subdirectory(sdk)

set(SDK_PATH "./sdk/")

FILE(GLOB SDK_SRC 
  "${SDK_PATH}/src/impl/unix/*.cpp"
  "${SDK_PATH}/src/*.cpp"
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/sdk/include
  ${PROJECT_SOURCE_DIR}/sdk/src
)

add_executable(ydlidar_node src/ydlidar_node.cpp ${SDK_SRC})
add_executable(ydlidar_client src/ydlidar_client.cpp)

ament_target_dependencies(ydlidar_node rclcpp std_msgs sensor_msgs)
ament_target_dependencies(ydlidar_client rclcpp std_msgs sensor_msgs)

#ydlidar_node
install(TARGETS ydlidar_node ydlidar_client
  DESTINATION lib/${PROJECT_NAME})
install(
	DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}/
	)

ament_package()
