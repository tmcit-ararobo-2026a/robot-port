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

add_library(operation_watchdog src/operation_watchdog.cpp)
target_link_libraries(operation_watchdog PUBLIC wire_codec)

if(BUILD_RUNTIME_NODES)
add_library(udp_channel src/simple_udp.cpp)
target_include_directories(udp_channel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(udp_node src/udp_node.cpp)
target_link_libraries(udp_node wire_codec udp_channel operation_watchdog)
rosidl_get_typesupport_target(cpp_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(udp_node "${cpp_typesupport}")
ament_target_dependencies(udp_node rclcpp std_msgs geometry_msgs)

target_include_directories(udp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  udp_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS udp_channel EXPORT udp_channel_targets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
ament_export_targets(udp_channel_targets HAS_LIBRARY_TARGET)

endif()

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(wire_codec_test test/wire_codec_test.cpp)
  target_link_libraries(wire_codec_test wire_codec)
  ament_add_gtest(operation_watchdog_test test/operation_watchdog_test.cpp)
  target_link_libraries(operation_watchdog_test operation_watchdog)
  if(BUILD_RUNTIME_NODES)
    ament_add_gtest(udp_channel_test test/udp_channel_test.cpp)
    target_link_libraries(udp_channel_test udp_channel)
    ament_add_pytest_test(udp_bridge_integration_test test/udp_bridge_integration_test.py
      ENV UDP_EXECUTABLE=$<TARGET_FILE:udp_node>
      TIMEOUT 60)
  endif()
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # The repository uses Google-based .clang-format, not ROS uncrustify defaults.
  set(ament_cmake_uncrustify_FOUND TRUE)
  find_program(CLANG_FORMAT_BIN NAMES clang-format-14 clang-format REQUIRED)
  add_test(NAME clang_format COMMAND ${CLANG_FORMAT_BIN} --dry-run --Werror
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_port/wire_codec.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_port/robot_config.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_port/simple_udp.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_port/operation_watchdog.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/wire_codec.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/simple_udp.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/operation_watchdog.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/udp_node.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/wire_codec_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/udp_channel_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/operation_watchdog_test.cpp)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS wire_codec operation_watchdog EXPORT wire_codec_targets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
ament_export_targets(wire_codec_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
