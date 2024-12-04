cmake_minimum_required(VERSION 3.8)
project(delivery_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(delivery_bot REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(
  include
)

set(srv_files
  "srv/PlaceOrder.srv"
  "srv/CancelOrder.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES std_msgs
)

add_executable(order_management src/order_management.cpp)
ament_target_dependencies(order_management rclcpp std_msgs nav2_msgs geometry_msgs rclcpp_action delivery_bot)

install(TARGETS 
  order_management
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  description
  rviz
  src 
  params
  launch 
  worlds
  maps
  DESTINATION share/${PROJECT_NAME} )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_export_dependencies(rosidl_default_runtime)

ament_package()
