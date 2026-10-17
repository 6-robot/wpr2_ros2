cmake_minimum_required(VERSION 3.8)
project(wpr2_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/trajectory_to_wpr2.py scripts/get_pose.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(moveit_config test/test_moveit_config.py TIMEOUT 60)
  ament_add_pytest_test(trajectory_bridge test/test_trajectory_bridge.py TIMEOUT 60)
endif()

ament_package()
