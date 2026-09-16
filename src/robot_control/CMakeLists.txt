cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(commmux_node src/commmux_node.cpp)
ament_target_dependencies(commmux_node rclcpp geometry_msgs robot_interfaces)

add_library(translational_trajectory src/translational_trajectory.cpp)
target_include_directories(translational_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(translational_trajectory PUBLIC cxx_std_14)

add_executable(tracing_node src/tracing_node.cpp)
target_include_directories(tracing_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(tracing_node rclcpp geometry_msgs robot_interfaces nav_msgs)
target_link_libraries(tracing_node
  translational_trajectory
  Eigen3::Eigen
  OsqpEigen::OsqpEigen
)

install(TARGETS
  commmux_node
  tracing_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS translational_trajectory
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(translational_trajectory_test
    test/translational_trajectory_test.cpp
  )
  if(TARGET translational_trajectory_test)
    target_link_libraries(translational_trajectory_test translational_trajectory)
    target_compile_features(translational_trajectory_test PUBLIC cxx_std_14)
  endif()

  ament_add_gtest(tracing_adapter_test
    test/tracing_adapter_test.cpp
  )
  if(TARGET tracing_adapter_test)
    target_include_directories(tracing_adapter_test PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_compile_features(tracing_adapter_test PUBLIC cxx_std_14)
  endif()
endif()

ament_package()
