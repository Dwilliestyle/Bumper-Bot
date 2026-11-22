cmake_minimum_required(VERSION 3.8)
project(bumperbot_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS
  ${PROJECT_NAME}/ball_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY behavior_tree config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()