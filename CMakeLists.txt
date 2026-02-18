cmake_minimum_required(VERSION 3.8)
project(gnss_multipath_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Add the mesages dependencies required for the messages:
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(gz-sim7 REQUIRED) 
find_package(gz-math7 REQUIRED)
find_package(gz-plugin2 REQUIRED)

# Add the libpredict library for the Satellite simualtion:
find_package(PkgConfig REQUIRED)
set(ENV{PKG_CONFIG_PATH} "${CMAKE_INSTALL_PREFIX}/../libpredict/lib/pkgconfig/\
:/usr/share/pkg-config\
:/usr/lib/pkg-config
:$ENV{PKG_CONFIG_PATH}")
pkg_check_modules(PREDICT REQUIRED IMPORTED_TARGET predict)

# Make the message avaliable for the plugin:
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GNSSMultipathFix.msg"
  DEPENDENCIES std_msgs sensor_msgs builtin_interfaces
)

# inlcude the libraries in the package
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${PREDICT_INCLUDE_DIRS}
)

# BUild the multipath plugin;
add_library(gnss_multipath_plugin_node SHARED
  src/multipath_sensor.cpp
)

ament_target_dependencies(gnss_multipath_plugin_node
  rclcpp
  sensor_msgs
  gz-sim7
  gz-math7
  gz-plugin2
)

target_link_libraries(gnss_multipath_plugin_node
  PkgConfig::PREDICT
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(gnss_multipath_plugin_node "${cpp_typesupport_target}")

# Install shared library to lib/
install(TARGETS gnss_multipath_plugin_node
  LIBRARY DESTINATION lib
)

# Install header files (optional if you use them externally)
install(DIRECTORY include/
  DESTINATION include
)

# Export the plugin:
ament_export_include_directories(include)
ament_export_libraries(gnss_multipath_plugin_node)


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


ament_export_dependencies(rosidl_default_runtime)
ament_package()
