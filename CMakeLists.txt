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

add_library(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

target_include_directories(talker
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

add_executable(talker_node src/talker_node.cpp)
ament_target_dependencies(talker_node rclcpp)
target_link_libraries(talker_node talker)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS talker
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
