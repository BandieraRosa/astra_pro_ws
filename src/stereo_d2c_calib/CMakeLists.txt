cmake_minimum_required(VERSION 3.8)
project(stereo_d2c_calib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_library(d2c_capture_component SHARED src/capture_node.cpp)
ament_target_dependencies(d2c_capture_component
  rclcpp rclcpp_components sensor_msgs std_msgs message_filters OpenCV)
target_include_directories(d2c_capture_component PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(d2c_capture_component ${OpenCV_LIBS})
rclcpp_components_register_node(d2c_capture_component
  PLUGIN "stereo_d2c_calib::D2CCapture"
  EXECUTABLE d2c_capture)

add_executable(d2c_solve src/solve.cpp)
ament_target_dependencies(d2c_solve sensor_msgs camera_calibration_parsers OpenCV)
target_include_directories(d2c_solve PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(d2c_solve ${OpenCV_LIBS})

install(
  TARGETS d2c_capture_component d2c_capture d2c_solve
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
