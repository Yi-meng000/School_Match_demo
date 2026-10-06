cmake_minimum_required(VERSION 3.8)
project(robot_comm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

add_executable(usb2uart_node src/usb2uart_node.cpp)
target_compile_features(usb2uart_node PRIVATE cxx_std_17)
target_include_directories(usb2uart_node PRIVATE include)
ament_target_dependencies(usb2uart_node rclcpp robot_interfaces)

add_executable(usb2topic_node src/usb2topic_node.cpp)
target_compile_features(usb2topic_node PRIVATE cxx_std_17)
ament_target_dependencies(usb2topic_node rclcpp robot_interfaces)

add_executable(wheel_velocity_node src/wheel_velocity_node.cpp)
target_compile_features(wheel_velocity_node PRIVATE cxx_std_17)
ament_target_dependencies(
  wheel_velocity_node
  rclcpp
  geometry_msgs
  robot_interfaces
)

install(TARGETS
  usb2uart_node
  usb2topic_node
  wheel_velocity_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(
    chassis_feedback_protocol_test
    test/chassis_feedback_protocol_test.cpp
  )
  target_compile_features(chassis_feedback_protocol_test PRIVATE cxx_std_17)
  target_include_directories(chassis_feedback_protocol_test PRIVATE include)
endif()

ament_package()
