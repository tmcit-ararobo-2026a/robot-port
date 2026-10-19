cmake_minimum_required(VERSION 3.10)
project(robot_port)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(wire_codec src/wire_codec.cpp)
add_library(udp_channel src/simple_udp.cpp)
foreach(target wire_codec udp_channel)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
endforeach()

add_executable(udp_node src/udp_node.cpp)
target_link_libraries(udp_node wire_codec udp_channel)
ament_target_dependencies(udp_node rclcpp std_msgs geometry_msgs)
install(TARGETS udp_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS wire_codec udp_channel EXPORT robot_port_targets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
ament_export_targets(robot_port_targets HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  # Additional tests stay local; public checkouts must also build without them.
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/navigation_state_test.cpp")
    add_executable(navigation_state_test test/navigation_state_test.cpp)
    target_include_directories(navigation_state_test PRIVATE include)
    add_test(NAME navigation_state_test COMMAND navigation_state_test)
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/wire_codec_test.cpp")
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(wire_codec_test test/wire_codec_test.cpp)
    target_link_libraries(wire_codec_test wire_codec)
    ament_add_gtest(udp_channel_test test/udp_channel_test.cpp)
    target_link_libraries(udp_channel_test udp_channel)
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/standard_udp_integration_test.py")
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(standard_udp_integration_test test/standard_udp_integration_test.py
      ENV UDP_EXECUTABLE=$<TARGET_FILE:udp_node>
      TIMEOUT 120)
  endif()
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # The repository uses clang-format instead of ROS uncrustify defaults.
  set(ament_cmake_uncrustify_FOUND TRUE)
  find_program(CLANG_FORMAT_BIN NAMES clang-format-14 clang-format REQUIRED)
  add_test(NAME clang_format COMMAND ${CLANG_FORMAT_BIN} --dry-run --Werror
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_port/wire_codec.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_port/robot_config.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_port/simple_udp.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_port/navigation_state.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/wire_codec.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/simple_udp.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/udp_node.cpp)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
