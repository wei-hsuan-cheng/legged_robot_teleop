cmake_minimum_required(VERSION 3.22)
project(legged_robot_teleop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ROS-independent deadman FSM, tested on its own.
add_library(teleop_control SHARED src/control.cpp)
target_include_directories(teleop_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# The node, also loadable as a component in a container.
add_library(base_twist_teleop SHARED src/base_twist_teleop.cpp)
target_link_libraries(base_twist_teleop PUBLIC
  teleop_control
  ${ocs2_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS})
rclcpp_components_register_nodes(base_twist_teleop "legged_robot_teleop::BaseTwistTeleop")

# Own main: publishes one zero command on SIGINT/SIGTERM before shutting down.
add_executable(base_twist src/base_twist_main.cpp)
target_link_libraries(base_twist base_twist_teleop)

install(TARGETS teleop_control base_twist_teleop
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS base_twist DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
# The launch file reads robot height bounds through legged_robot_teleop.profile.
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_control test/test_control.cpp)
  target_link_libraries(test_control teleop_control)

  foreach(test_name profile launch ament_style)
    ament_add_pytest_test(test_${test_name} test/test_${test_name}.py)
  endforeach()
  # ROS-level tests drive the built executable on isolated topics and domains.
  foreach(test_name ros_transport posture_toggle gait_buttons)
    ament_add_pytest_test(test_${test_name} test/test_${test_name}.py
      ENV TELEOP_EXECUTABLE=$<TARGET_FILE:base_twist>
      TIMEOUT 120)
  endforeach()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ocs2_msgs rclcpp rclcpp_components sensor_msgs std_msgs std_srvs)
ament_package()
