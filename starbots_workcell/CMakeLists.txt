cmake_minimum_required(VERSION 3.8)
project(starbots_workcell)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_dispenser_msgs REQUIRED)
find_package(rmf_ingestor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(DIRECTORY
  images
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/manual_dispenser_v2.py
  scripts/ros2_human_interface.py
  scripts/manual_ingestor_v2.py
  scripts/robot_arm_panda_dispenser.py
  DESTINATION lib/${PROJECT_NAME}
)