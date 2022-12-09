cmake_minimum_required(VERSION 3.5)
project(multipath_sim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_plugins REQUIRED)
include_directories(include ${GAZEBO_INCLUDE_DIRS})
add_library(MultipathSensorPlugin SHARED src/multipath_sensor.cpp)
add_library(SphereModelPlugin SHARED src/sphere_model_plugin.cpp)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MultipathOffset.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
#ament_export_libraries(multipath_sim)
target_include_directories(MultipathSensorPlugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(SphereModelPlugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  

target_link_libraries(MultipathSensorPlugin ${GAZEBO_LIBRARIES} ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})
ament_target_dependencies(MultipathSensorPlugin
 "gazebo_ros"
 "rclcpp"
 "sensor_msgs"
 "std_msgs")

target_link_libraries(SphereModelPlugin ${GAZEBO_LIBRARIES} ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})
ament_target_dependencies(SphereModelPlugin
 "gazebo_ros"
 "rclcpp"
 "sensor_msgs"
 "std_msgs"
 )

rosidl_target_interfaces(MultipathSensorPlugin
 ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS MultipathSensorPlugin
  DESTINATION lib)

rosidl_target_interfaces(SphereModelPlugin
${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS SphereModelPlugin
  DESTINATION lib)


install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)  

install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}/
) 

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
) 
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
) 

install(DIRECTORY
  data
  DESTINATION lib/
) 

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
  scripts/set_model_state.py
  scripts/set_model_state_heatmap.py
  scripts/stream_skycam.py
  scripts/broadcast_pos.py
  scripts/broadcast_pos_heatmap.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(std_msgs)
ament_package()
