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
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(navigation REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(commmux_node src/commmux_node.cpp)
# 添加依赖的包，并使用这些包导出的依赖信息
ament_target_dependencies(commmux_node rclcpp geometry_msgs robot_interfaces)

add_library(translational_trajectory src/translational_trajectory.cpp) # 构建可被引用的轨迹库（默认静态库）
# 设置头文件搜索目录  翻译"robot_control/translational_trajectory.hpp"
target_include_directories(translational_trajectory PUBLIC    # PUBLIC 编辑这个库以及链接这个库的其他目标都需要这些目录；     
                                                              # PRIVATE 只有这个库需要这些目录；
                                                              # INTERFACE 只有链接这个库的其他目标需要这些目录；
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>      # ${}取变量值   CMAKE_CURRENT_SOURCE_DIR当前处理的源码目录src/robot_control
  $<INSTALL_INTERFACE:include>                                # $<...>生成器表达式根据场景展开    使用安装后的库，头文件从安装前缀下的include中找
)
target_compile_features(translational_trajectory PUBLIC cxx_std_14)     # 设置C++标准至少为C++14

add_executable(tracing_node src/tracing_node.cpp)
target_include_directories(tracing_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(tracing_node rclcpp geometry_msgs robot_interfaces nav_msgs std_msgs navigation)
# 链接所需库
arget_link_libraries(tracing_node
  translational_trajectory
  Eigen3::Eigen
)
# 这里的DESTINATION是相对路径，相对于安装前缀CMAKE_INSTALL_PREFIX，工作区/install/robot_control，非源码目录
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

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) # 公开该集合内目标
ament_export_include_directories(include) # 公开头文件搜索目录，作为包信息导出


if(BUILD_TESTING) # 测试配置 默认开启可构建时传参关闭 colcon build --packages-select robot_control --cmake-args -DBUILD_TESTING=OFF
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

  ament_add_gtest(tracing_pipeline_test
    test/tracing_pipeline_test.cpp
  )
  if(TARGET tracing_pipeline_test)
    target_include_directories(tracing_pipeline_test PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_link_libraries(tracing_pipeline_test
      translational_trajectory
      Eigen3::Eigen
    )
    target_compile_features(tracing_pipeline_test PUBLIC cxx_std_14)
  endif()
endif()

ament_package()
