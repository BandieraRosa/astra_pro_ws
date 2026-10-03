cmake_minimum_required(VERSION 3.8)
project(image_flip)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_library(image_flip_component SHARED src/flip_node.cpp)
ament_target_dependencies(image_flip_component rclcpp rclcpp_components sensor_msgs)
target_include_directories(image_flip_component PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(image_flip_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(image_flip_component ${OpenCV_LIBS})
rclcpp_components_register_node(image_flip_component
  PLUGIN "ImageFlip"
  EXECUTABLE flip_node)

install(
  TARGETS image_flip_component flip_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_camera_info_flip test/test_camera_info_flip.cpp)
  ament_target_dependencies(test_camera_info_flip sensor_msgs)
  target_include_directories(test_camera_info_flip PRIVATE include)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
