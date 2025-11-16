cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(catch_ros2 REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Enable compile_commands.json
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")


if(BUILD_TESTING)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  add_executable(integration_test_node
    test/integration_test_node.cpp
  )

  target_link_libraries(integration_test_node
    catch_ros2::catch_ros2_with_node_main
  )
  ament_target_dependencies(integration_test_node
    rclcpp std_srvs std_msgs
  )
  install(TARGETS
    integration_test_node
    DESTINATION lib/${PROJECT_NAME}
  )

  catch_ros2_add_integration_test (ExampleIntegration_TestYAML
    LAUNCH_FILE integration_test.launch.yaml
  )
endif()

add_library(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs std_srvs)

target_include_directories(talker
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

add_executable(talker_node src/talker_node.cpp)
ament_target_dependencies(talker_node rclcpp)
target_link_libraries(talker_node talker)

add_library(listener src/listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

target_include_directories(listener
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

add_executable(listener_node src/listener_node.cpp)
ament_target_dependencies(listener_node rclcpp std_msgs)
target_link_libraries(listener_node listener)

install(TARGETS
  talker_node
  listener_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS talker listener
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(some_dependency)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
