cmake_minimum_required(VERSION 3.8)
project(image_flip)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(flip_node src/flip_node.cpp)
ament_target_dependencies(flip_node rclcpp sensor_msgs)
target_include_directories(flip_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(flip_node ${OpenCV_LIBS})

install(
  TARGETS flip_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
