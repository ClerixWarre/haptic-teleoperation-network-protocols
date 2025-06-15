cmake_minimum_required(VERSION 3.8)
project(quic_haptic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(omni_msgs REQUIRED)

# Add include directories
include_directories(include)

# Server executable
add_executable(quic_haptic_server src/quic_haptic_server.cpp)
ament_target_dependencies(quic_haptic_server
  rclcpp
  std_msgs
  omni_msgs
)

# Client executable
add_executable(quic_haptic_client src/quic_haptic_client.cpp)
ament_target_dependencies(quic_haptic_client
  rclcpp
  std_msgs
  omni_msgs
)

# Link against msquic library if needed
target_link_libraries(quic_haptic_server msquic)
target_link_libraries(quic_haptic_client msquic)

# Install executables
install(TARGETS
  quic_haptic_server
  quic_haptic_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
