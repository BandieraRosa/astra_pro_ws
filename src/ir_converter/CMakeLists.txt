cmake_minimum_required(VERSION 3.8)
project(ir_converter)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_library(ir_converter_component SHARED src/ir_converter_node.cpp)
ament_target_dependencies(ir_converter_component rclcpp rclcpp_components sensor_msgs)
target_include_directories(ir_converter_component PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ir_converter_component ${OpenCV_LIBS})
rclcpp_components_register_node(ir_converter_component
  PLUGIN "ir_converter::IrY10Converter"
  EXECUTABLE ir_converter)

install(
  TARGETS ir_converter_component ir_converter
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
