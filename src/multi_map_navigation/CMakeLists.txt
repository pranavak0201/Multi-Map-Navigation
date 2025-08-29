cmake_minimum_required(VERSION 3.8)
project(multi_map_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find all necessary ROS 2 and system dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find SQLite3 using PkgConfig
find_package(PkgConfig REQUIRED)
pkg_check_modules(SQLITE3 REQUIRED sqlite3)

# Generate C++ code from the .action file
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateTo.action"
  DEPENDENCIES geometry_msgs
)

# This is crucial - ensures the generated interfaces are built before using them
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Define the C++ executable
add_executable(multi_map_navigator_node 
  src/multi_map_navigator_node.cpp 
  src/navigator.cpp
)

# Tell CMake where to find the header files for this target
target_include_directories(multi_map_navigator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Specify ROS 2 package dependencies
ament_target_dependencies(multi_map_navigator_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
)

# Link against system libraries and generated interfaces
target_link_libraries(multi_map_navigator_node
  ${SQLITE3_LIBRARIES}
  ${cpp_typesupport_target}
)

# Install the executable
install(TARGETS
  multi_map_navigator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  maps
  config
  worlds
  db
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()