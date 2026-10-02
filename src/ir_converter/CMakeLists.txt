cmake_minimum_required(VERSION 3.8)
project(ir_converter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(ir_converter src/ir_converter_node.cpp)

ament_target_dependencies(ir_converter rclcpp sensor_msgs)
target_include_directories(ir_converter PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ir_converter ${OpenCV_LIBS})

install(TARGETS ir_converter DESTINATION lib/${PROJECT_NAME})

ament_package()
