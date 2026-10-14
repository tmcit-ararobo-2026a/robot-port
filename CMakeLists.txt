cmake_minimum_required(VERSION 3.10)
project(robot_port)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

option(BUILD_RUNTIME_NODES "Build UDP runtime nodes" ON)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotCommand.msg"
  "msg/RobotOperation.msg"
  "msg/RobotFeedback.msg"
  "msg/BridgeHealth.msg"
  DEPENDENCIES builtin_interfaces)

add_library(wire_codec src/wire_codec.cpp)
target_include_directories(wire_codec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(BUILD_RUNTIME_NODES)
add_library(udp_channel src/simple_udp.cpp)
target_include_directories(udp_channel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(udp_node src/udp_node.cpp)
target_link_libraries(udp_node wire_codec udp_channel)
rosidl_get_typesupport_target(cpp_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(udp_node "${cpp_typesupport}")
ament_target_dependencies(udp_node rclcpp std_msgs geometry_msgs)

target_include_directories(udp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  udp_node
  DESTINATION lib/${PROJECT_NAME})

endif()

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(wire_codec_test test/wire_codec_test.cpp)
  target_link_libraries(wire_codec_test wire_codec)
  if(BUILD_RUNTIME_NODES)
    ament_add_gtest(udp_channel_test test/udp_channel_test.cpp)
    target_link_libraries(udp_channel_test udp_channel)
  endif()
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS wire_codec EXPORT wire_codec_targets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
ament_export_targets(wire_codec_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
