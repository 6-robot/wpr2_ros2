cmake_minimum_required(VERSION 3.8)
project(wpr2_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(description test/test_description.py TIMEOUT 60)
endif()

ament_package()
