cmake_minimum_required(VERSION 3.8)
project(dccp_haptic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(omni_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# DCCP Haptic Client
add_executable(dccp_haptic_client
  src/dccp_haptic_client.cpp
)
target_include_directories(dccp_haptic_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dccp_haptic_client PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  dccp_haptic_client
  "rclcpp"
  "omni_msgs"
  "std_msgs"
  "geometry_msgs"
)
target_link_libraries(dccp_haptic_client
  pthread
)

# DCCP Haptic Server
add_executable(dccp_haptic_server
  src/dccp_haptic_server.cpp
)
target_include_directories(dccp_haptic_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dccp_haptic_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  dccp_haptic_server
  "rclcpp"
  "omni_msgs"
  "std_msgs"
  "geometry_msgs"
)
target_link_libraries(dccp_haptic_server
  pthread
)

# Install executables
install(TARGETS dccp_haptic_client dccp_haptic_server
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
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

ament_package()
