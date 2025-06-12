cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Paketler
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

# .srv dosyasını işler
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDirection.srv"
  "action/GoToPose.action"
  DEPENDENCIES geometry_msgs sensor_msgs
)

# Executables
add_executable(go_to_pose_action src/go_to_pose_action.cpp)
ament_target_dependencies(go_to_pose_action
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_dependencies(go_to_pose_action ${PROJECT_NAME}__rosidl_typesupport_cpp)

target_include_directories(go_to_pose_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(go_to_pose_action ${cpp_typesupport_target})

install(TARGETS go_to_pose_action DESTINATION lib/${PROJECT_NAME})


add_executable(patrol src/patrol.cpp)
ament_target_dependencies(patrol rclcpp sensor_msgs geometry_msgs)
install(TARGETS patrol DESTINATION lib/${PROJECT_NAME})

add_executable(test_service src/test_service.cpp)
ament_target_dependencies(test_service
  rclcpp
  sensor_msgs
  rosidl_typesupport_cpp
)
add_dependencies(test_service ${PROJECT_NAME}__rosidl_typesupport_cpp)

target_include_directories(test_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(test_service ${cpp_typesupport_target})

install(TARGETS test_service DESTINATION lib/${PROJECT_NAME})



###############
add_executable(patrol_with_service src/patrol_with_service.cpp)
ament_target_dependencies(patrol_with_service
  rclcpp
  sensor_msgs
  rosidl_typesupport_cpp
)
add_dependencies(patrol_with_service ${PROJECT_NAME}__rosidl_typesupport_cpp)

target_include_directories(patrol_with_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(patrol_with_service ${cpp_typesupport_target})

install(TARGETS patrol_with_service DESTINATION lib/${PROJECT_NAME})
###############

add_executable(direction_service src/direction_service.cpp)
ament_target_dependencies(direction_service
  rclcpp
  sensor_msgs
  geometry_msgs
  rosidl_typesupport_cpp
)
add_dependencies(direction_service ${PROJECT_NAME}__rosidl_typesupport_cpp)
target_include_directories(direction_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)


# Link C++ typesupport
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(direction_service ${cpp_typesupport_target})

install(TARGETS direction_service DESTINATION lib/${PROJECT_NAME})

# launch + rviz
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
